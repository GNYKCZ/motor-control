"""P1a-1一次性命令工具：默认只显示帧，只有显式--port才打开串口；不自动重发或运行电机。"""
import argparse
import binascii
import json
import math
from pathlib import Path
import time
from sensor_capture import save_chunks

COMMANDS = {"status": 1, "stop": 2, "arm": 3, "pulse": 4}


def make_command(sequence, command):
    """固定23B；CRC只覆盖P1C1前17B；不支持任意占空比/电流/时间参数。"""
    if not 0 <= sequence <= 0xFFFFFFFF or command not in COMMANDS:
        raise ValueError("invalid sequence or command")
    prefix = f"P1C1,{sequence:08X},{COMMANDS[command]:02X},".encode("ascii")
    return prefix + f"{binascii.crc_hqx(prefix, 0xFFFF):04X}\r\n".encode("ascii")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command", choices=COMMANDS, required=True)
    parser.add_argument("--seq", type=lambda text: int(text, 0), required=True)
    parser.add_argument("--port", help="现场安排后才填写，如COM9；先断开VOFA对同端口的占用")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--firmware-sha")
    parser.add_argument("--board")
    parser.add_argument("--motor")
    parser.add_argument("--supply")
    parser.add_argument("--seconds", type=float, default=1.0)
    args = parser.parse_args()
    try:
        frame = make_command(args.seq, args.command)
    except ValueError as error:
        parser.error(str(error))
    print("待发送（默认不打开串口）：", repr(frame.decode("ascii")))
    if not args.port:
        return
    if (not args.output or args.output.exists() or not math.isfinite(args.seconds)
            or not 0.2 <= args.seconds <= 10.0):
        parser.error("串口需要全新output目录，seconds在0.2..10范围")
    import re
    if (not args.firmware_sha or not re.fullmatch(r"[0-9a-fA-F]{40}", args.firmware_sha)
            or int(args.firmware_sha, 16) == 0
            or not all(value and value.strip() for value in (args.board, args.motor, args.supply))):
        parser.error("现场必须填写真实完整firmware-sha和board/motor/supply")
    try:
        import serial
    except ImportError:
        parser.error("现场串口入口需要pyserial3.5；默认显示命令不需要")
    if serial.__version__ != "3.5":
        parser.error("现场入口固定pyserial3.5")
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.25)
    connection.dtr = False
    connection.rts = False
    connection.port = args.port
    connection.open()
    metadata = {"source_kind": "unverified_capture", "firmware_sha": args.firmware_sha,
                "board": args.board, "motor": args.motor, "supply": args.supply,
                "source": args.port, "baud": 115200, "command_hex": frame.hex(),
                "requested_seq": args.seq, "command": args.command,
                "command_sent": False, "tool_note": "one request; no automatic retry/arm/pulse"}
    try:
        # 在原流目录存在后才实际写入；异常也由save_chunks保存已经收到的字节和元数据。
        def received():
            count = connection.write(frame)
            metadata["command_sent"] = count == len(frame)
            if count != len(frame):
                raise RuntimeError("partial serial write; no automatic retry")
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                data = connection.read(256)
                if data:
                    yield data
        # save_chunks在迭代前复制元数据，所以写入成功以独立result.json记录，不能冒充预先发送成功。
        save_chunks(received(), args.output, metadata)
    finally:
        connection.close()
        if args.output.exists():
            (args.output / "send_result.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    records = [json.loads(line) for line in (args.output / "records.jsonl").read_text(encoding="utf-8").splitlines()]
    replies = [r["fields"] for r in records if r["fields"] and r["fields"]["format"] == "P1R1"
               and r["fields"]["q"] == args.seq and r["fields"]["c"] == COMMANDS[args.command]]
    if not replies:
        raise SystemExit("未收到匹配回执；保留raw.bin，不自动重发。STOP无回执不代表未执行，也不能当已停机证明。")
    print("匹配回执：", json.dumps(replies, ensure_ascii=False))


if __name__ == "__main__":
    main()
