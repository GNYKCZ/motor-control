"""一次连接的开口诊断：默认不打开端口；可先被动接收，再显式发送一次STATUS。"""
import argparse
import json
import math
from pathlib import Path
import re
import time
from command_link import make_command
from sensor_capture import save_chunks


def capture_session(connection, output, metadata, before=2.0, after=2.0, status_seq=None, clock=time.monotonic):
    """connection已打开；不flush/reset，不重发。clock可替换以离线核对时间与字节。"""
    if (not all(math.isfinite(v) and 0.1 <= v <= 60 for v in (before, after))
            or (status_seq is not None and not 0 <= status_seq <= 0xFFFFFFFF)):
        raise ValueError("before/after需要0.1..60秒，序号需要uint32")
    output = Path(output)
    if output.exists():
        raise FileExistsError("output already exists")
    frame = make_command(status_seq, "status") if status_seq is not None else None
    result = {"status_requested": frame is not None, "write_attempted": False,
              "write_completed": False, "received_bytes": 0, "error": None}
    start = clock()
    events = []
    owned_output = False

    def chunks():
        nonlocal owned_output
        owned_output = True  # save_chunks已成功创建本次目录后才开始迭代。
        end = start + before + after
        # RV-001：预接收延迟不能让显式STATUS被总期限静默跳过。
        # 只允许一次写入；写完后重新建立after窗口，异常仍退出且不重发。
        while (frame is not None and not result["write_attempted"]) or clock() < end:
            if frame is not None and not result["write_attempted"] and clock() - start >= before:
                result["write_attempted"] = True  # 写入异常/部分写入也不重试。
                event = {"operation": "write", "rx_offset": result["received_bytes"],
                         "tx_offset": 0, "requested_length": len(frame), "raw_hex": frame.hex(),
                         "started_monotonic": clock()}
                events.append(event)
                try:
                    count = connection.write(frame)
                    event["length"] = count
                    result["write_completed"] = count == len(frame)
                    if count != len(frame):
                        raise RuntimeError("partial STATUS write; no retry")
                except Exception as error:
                    event["error"] = str(error)
                    raise
                finally:
                    event["finished_monotonic"] = clock()
                end = clock() + after
            event = {"operation": "read", "rx_offset": result["received_bytes"],
                     "requested_length": 256, "started_monotonic": clock()}
            events.append(event)
            try:
                data = connection.read(256)
                event["length"] = len(data)
                result["received_bytes"] += len(data)
            except Exception as error:
                event["error"] = str(error)
                raise
            finally:
                event["finished_monotonic"] = clock()
            if data:
                yield data

    try:
        info = save_chunks(chunks(), output, {**metadata, "diagnostic": "session_probe_v1",
                           "before_seconds": before, "after_seconds": after,
                           "status_seq": status_seq, "start_monotonic": start,
                           "timing_note": "host read/write calls; not USB packet or MCU timestamps"})
        result["valid_lines"] = info["valid_lines"]
        result["invalid_lines"] = info["invalid_lines"]
    except Exception as error:
        result["error"] = str(error)
        raise
    finally:
        # 只写本次成功创建的目录；现有采集目录绝不能被覆盖。
        if owned_output:
            (output / "events.jsonl").write_text("".join(json.dumps(e, ensure_ascii=False) + "\n" for e in events), encoding="utf-8")
            (output / "session_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--status-seq", type=lambda value: int(value, 0), help="显式发送一次新序号STATUS；省略则全程只读")
    parser.add_argument("--before", type=float, default=2.0)
    parser.add_argument("--after", type=float, default=2.0)
    parser.add_argument("--firmware-sha")
    parser.add_argument("--board")
    parser.add_argument("--motor")
    parser.add_argument("--supply")
    args = parser.parse_args()
    if not all(math.isfinite(v) and 0.1 <= v <= 60 for v in (args.before, args.after)):
        parser.error("before/after需要0.1..60秒")
    if args.status_seq is not None and not 0 <= args.status_seq <= 0xFFFFFFFF:
        parser.error("status-seq需要uint32新序号")
    if not args.port:
        print(f"预览：先只读{args.before}秒；" + ("不发送命令；" if args.status_seq is None else f"一次STATUS seq={args.status_seq}；") + f"继续接收{args.after}秒。未打开串口。")
        return
    if (not args.output or args.output.exists() or not args.firmware_sha
            or not re.fullmatch(r"[0-9a-fA-F]{40}", args.firmware_sha) or int(args.firmware_sha, 16) == 0
            or not all(v and v.strip() for v in (args.board, args.motor, args.supply))):
        parser.error("现场需要全新output、完整非零firmware-sha与真实board/motor/supply")
    try:
        import serial
    except ImportError:
        parser.error("现场串口需要pyserial3.5；无--port的预览不需要该依赖")
    if serial.__version__ != "3.5":
        parser.error("需要pyserial3.5")
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.25)
    connection.dtr = False
    connection.rts = False
    connection.port = args.port
    connection.open()
    try:
        result = capture_session(connection, args.output,
                                 {"source_kind": "unverified_capture", "source": args.port,
                                  "firmware_sha": args.firmware_sha, "board": args.board,
                                  "motor": args.motor, "supply": args.supply, "baud": 115200},
                                 args.before, args.after, args.status_seq)
        print(json.dumps(result, ensure_ascii=False))
    finally:
        connection.close()


if __name__ == "__main__":
    main()
