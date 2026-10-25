"""保存M2C1/M1串口原字节与逐行解析，不发送控制命令，不改变板端参数。

--input只处理文件；--port需另装pyserial3.5并由现场明确安排后执行。
输出为新建目录，绝不覆盖旧采集；元数据由现场填写，不能据软件自动确认实测/校准。
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import time

RAW_KEYS = "s f n t e es ec u v a mg ok bad skip pwm power".split()
SENSOR_KEYS = "cn cb ca cd cv ou_mc ov_mc umin umax vmin vmax mv iu_mA iv_mA ma_mrad scale_nA".split()


def parse_line(raw):
    """严格解析有版本的ASCII行；异常交给保存层，原字节仍保留。设备时间保持原32位值。"""
    if not raw.endswith(b"\n"):
        raise ValueError("unterminated line")
    text = raw[:-1].removesuffix(b"\r").decode("ascii")
    pieces = text.split(",")
    version = pieces[0]
    if version not in {"M1", "M2C1"}:
        raise ValueError("unknown format version")
    fields = {}
    for piece in pieces[1:]:
        key, value = piece.split("=", 1)
        if key in fields:
            raise ValueError("duplicate field: " + key)
        if key in {"pwm", "power"}:
            fields[key] = value
        else:
            if re.fullmatch(r"-?[0-9]+", value) is None:
                raise ValueError("not an integer: " + key)
            fields[key] = int(value)
    expected = set(RAW_KEYS + (SENSOR_KEYS if version == "M2C1" else []))
    if set(fields) != expected or fields["pwm"] != "blocked" or fields["power"] != "unverified":
        raise ValueError("field set or output boundary mismatch")
    for key, value in fields.items():
        if key in {"pwm", "power"}:
            continue
        if key in {"iu_mA", "iv_mA"}:
            if not -(2**31) <= value < 2**31:
                raise ValueError("signed field out of range")
        elif not 0 <= value < 2**32:
            raise ValueError("unsigned field out of range")
    if fields["s"] > 2 or fields["ok"] > 1 or fields["mg"] > 15:
        raise ValueError("state/validity out of range")
    if max(fields["u"],fields["v"],fields["a"]) > 65535:
        raise ValueError("raw storage out of range")
    if fields["ok"] and (max(fields["u"],fields["v"]) > 4095 or fields["a"] > 16383):
        raise ValueError("valid frame contradicts raw ranges")
    if version == "M2C1":
        if fields["cn"] > 512 or fields["cb"] > 1 or any(fields[k] > 1 for k in ("ca","cd","cv","mv")):
            raise ValueError("candidate flags/count out of range")
        if fields["cv"] != 0 or fields["cd"] != int(fields["cn"] == 512):
            raise ValueError("candidate is not verified calibration")
        if max(fields[k] for k in ("umin","umax","vmin","vmax")) > 4095 or max(fields["ou_mc"],fields["ov_mc"]) > 4095000:
            raise ValueError("candidate count value out of range")
        if fields["ma_mrad"] > 6283 or fields["scale_nA"] <= 0:
            raise ValueError("mechanical angle/nominal scale out of range")
        if fields["mv"] and (fields["s"] != 1 or fields["ca"] or not fields["cd"] or not fields["ok"]):
            raise ValueError("measurement validity contradicts snapshot")
        if not fields["mv"] and any(fields[k] != 0 for k in ("iu_mA","iv_mA","ma_mrad")):
            raise ValueError("invalid measurement carries numeric result")
    return {"format": version, **fields}


def save_chunks(chunks, output, metadata):
    """逐块保存原始流，逐行解析；超长/半行/非ASCII/非法行也保存，不过滤失败证据。"""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    info = {**metadata, "tool": "sensor_capture_v1", "python": platform.python_version(),
            "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "started_at_utc": datetime.now(timezone.utc).isoformat(), "valid_lines": 0, "invalid_lines": 0}
    pending = b""
    with (output / "raw.bin").open("wb") as original, (output / "records.jsonl").open("w", encoding="utf-8") as records:
        def store_line(line):
            """原字节hex与可读替代文本共同保留；解析失败不会删除或改写原始数据。"""
            record = {"host_time_utc": datetime.now(timezone.utc).isoformat(), "raw_hex": line.hex(),
                      "raw_text": line.decode("ascii", errors="replace"), "fields": None, "error": None}
            try:
                if len(line) > 2048:
                    raise ValueError("line exceeds 2048-byte parser bound")
                record["fields"] = parse_line(line)
                record["device_time_us"] = record["fields"]["t"]
                info["valid_lines"] += 1
            except (ValueError, UnicodeError) as error:
                record["error"] = str(error)
                info["invalid_lines"] += 1
            records.write(json.dumps(record, ensure_ascii=False) + "\n")
            records.flush()
        try:
            for chunk in chunks:
                original.write(chunk); original.flush(); pending += chunk
                while b"\n" in pending or len(pending) > 2048:
                    end = pending.find(b"\n") + 1
                    if end == 0 or end > 2049:
                        end = 2049
                    line, pending = pending[:end], pending[end:]
                    store_line(line)
        except KeyboardInterrupt:
            info["stopped_by_operator"] = True
        finally:
            if pending:
                store_line(pending)
            info["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            (output / "metadata.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return info


def main():
    """现场显式选择文件或串口，元数据不设伪造默认值；串口只读，不发送字符。"""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path)
    source.add_argument("--port")
    parser.add_argument("--kind", choices=["synthetic", "unverified_capture"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--firmware-sha", required=True)
    parser.add_argument("--board", required=True)
    parser.add_argument("--motor", required=True)
    parser.add_argument("--supply", required=True, help="现场记录电压、限流值、母线状态")
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()
    if args.output.exists() or not all(value.strip() for value in (args.board,args.motor,args.supply)):
        parser.error("输出目录必须全新，现场board/motor/supply不可空白")
    if re.fullmatch(r"[0-9a-fA-F]{40}", args.firmware_sha) is None or not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("完整40位firmware SHA和有限正seconds是必填条件")
    if args.port and args.kind != "unverified_capture":
        parser.error("串口数据不能标记为合成数据")
    if args.kind == "unverified_capture" and int(args.firmware_sha,16) == 0:
        parser.error("现场采集必须填写真实完整固件SHA，不能用合成零值")
    metadata = {"source_kind": args.kind, "firmware_sha": args.firmware_sha, "board": args.board,
                "motor": args.motor, "supply": args.supply, "source": str(args.input or args.port), "baud": 115200}
    if args.input:
        with args.input.open("rb") as stream:
            info = save_chunks(iter(lambda: stream.read(1024), b""), args.output, metadata)
    else:
        try:
            import serial
        except ImportError:
            parser.error("现场串口需要pyserial3.5：python -m pip install pyserial==3.5；文件解析无需该依赖")
        metadata["pyserial_version"] = serial.__version__
        if serial.__version__ != "3.5":
            parser.error("现场入口固定pyserial3.5，请先核对环境版本")
        connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
        connection.dtr = False; connection.rts = False; connection.port = args.port
        connection.open()
        try:
            def received():
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    data = connection.read(1024)
                    if data:
                        yield data
            info = save_chunks(received(), args.output, metadata)
        finally:
            connection.close()
    print(f"saved raw bytes and records: valid={info['valid_lines']}, invalid={info['invalid_lines']}")


if __name__ == "__main__":
    main()
