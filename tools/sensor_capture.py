"""保存M3B3/M3B2/M3B1/M2C1/M1串口原字节与逐行解析，不发送控制命令，不改变板端参数。

--input只处理文件；--port需另装pyserial3.5并由现场明确安排后执行。
输出为新建目录，绝不覆盖旧采集；元数据由现场填写，不能据软件自动确认实测/校准。
"""
import argparse
import binascii
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
FEEDBACK_KEYS = "fp fd fw ff fe ft fr fm fdt fn fc fa_mrad vd_mrad_s vw_mrad_s wd_count wdt_us fo fs dir dir_ok".split()
TIMING_KEYS = "fb_poll_us fb_step_us mb_wait_us mb_max_us mb_overrun_us".split() + [f"tp{i}_us" for i in range(14)]
COMPACT_KEYS = "s f n t u v a ok fp fd fw fo fb_us mb_us log_us log_id".split()


def parse_compact(fields):
    """M3B3只含本轮故障验收的uint32/原始值；不伪造旧协议已删的名义量字段。"""
    if set(fields) != set(COMPACT_KEYS):
        raise ValueError("compact field set mismatch")
    if any(not isinstance(v, int) or not 0 <= v < 2**32 for v in fields.values()):
        raise ValueError("compact unsigned field out of range")
    if fields["s"] > 2 or fields["log_id"] > 16 or any(fields[k] > 1 for k in ("ok", "fp", "fd", "fw")):
        raise ValueError("compact state/flag/id out of range")
    if max(fields[k] for k in ("u", "v", "a")) > 65535:
        raise ValueError("compact raw storage out of range")
    if fields["ok"] and (max(fields["u"], fields["v"]) > 4095 or fields["a"] > 16383):
        raise ValueError("compact valid raw range contradiction")
    if fields["fp"] and (fields["s"] != 1 or fields["f"] or not fields["ok"]):
        raise ValueError("compact feedback validity contradicts runtime")
    if (fields["fd"] and not fields["fp"]) or (fields["fw"] and not fields["fd"]):
        raise ValueError("compact feedback progress contradiction")
    if fields["fo"] and (fields["s"] != 2 or not (fields["f"] & 8)):
        raise ValueError("compact overrun without latched feedback fault")
    return {"format": "M3B3", **fields}


def parse_line(raw):
    """严格解析有版本的ASCII行；异常交给保存层，原字节仍保留。设备时间保持原32位值。"""
    if not raw.endswith(b"\n"):
        raise ValueError("unterminated line")
    if raw.startswith((b"P1Z1,", b"P1I1,", b"P1V1,")):
        current = not raw.startswith(b"P1Z1,")
        widths = [8, 2, 2, 2, 8, 8, 8, 4, 8, 8, 4, 4, 4, 4, 2, 8, 8, 8, 8]
        keys = "q c r s f n t cn su sv umin umax vmin vmax fl scale beg end cfg".split()
        if current:
            widths += [4, 8, 8, 4, 4, 4, 4, 8, 8, 4, 4, 2, 8]
            keys += "ic isu isv iumin iumax ivmin ivmax ibeg iend a0 a1 overflow iid".split()
        widths += [4]
        keys += ["crc"]
        pattern = raw[:5] + b",".join(rb"([0-9A-F]{" + str(w).encode() + rb"})" for w in widths) + rb"\r\n"
        match = re.fullmatch(pattern, raw)
        if not match or len(raw) != (221 if current else 138):
            raise ValueError("P1Z1 layout mismatch")
        values = [int(value, 16) for value in match.groups()]
        if binascii.crc_hqx(raw[:-6], 0xFFFF) != values[-1]:
            raise ValueError("P1Z1 CRC mismatch")
        fields = dict(zip(keys, values))
        if fields["c"] != (7 if current else 5) or fields["s"] > 5 or fields["r"] & 0x7F > 7 or fields["cn"] > 512:
            raise ValueError("P1Z1 command/state/count out of range")
        count, flags = fields["cn"], fields["fl"]
        if flags & ~31 or not flags & 8 or bool(flags & 1) != (count == 512):
            raise ValueError("P1Z1 candidate flags mismatch")
        if not fields["scale"] or not fields["cfg"]:
            raise ValueError("P1Z1 missing configuration or nominal scale")
        if not count and any(fields[k] for k in ("su", "sv", "umin", "umax", "vmin", "vmax", "beg", "end")):
            raise ValueError("P1Z1 empty candidate contradicts data")
        for channel in ("u", "v"):
            low, high, total = fields[channel + "min"], fields[channel + "max"], fields["s" + channel]
            if not 0 <= low <= high <= 4095 or not count * low <= total <= count * high:
                raise ValueError("P1Z1 candidate sum/range mismatch")
        if count and (fields["end"] - fields["beg"]) & 0xFFFFFFFF > (count - 1) * 3000:
            raise ValueError("P1Z1 candidate time order/gap mismatch")
        if count and (fields["t"] - fields["end"]) & 0xFFFFFFFF >= 0x80000000:
            raise ValueError("P1Z1 snapshot predates candidate")
        if current:
            count = fields["ic"]
            if count > 32 or fields["overflow"] > 1 or max(fields["a0"], fields["a1"]) > 16383:
                raise ValueError("P1I1 count/angle/overflow mismatch")
            if fields["overflow"] and count != 32:
                raise ValueError("P1I1 overflow without full buffer")
            if count and not fields["iid"]:
                raise ValueError("P1I1 missing action identity")
            if not count and any(fields[k] for k in keys if k.startswith("i") or k in ("a0", "a1", "overflow")):
                raise ValueError("P1I1 empty capture contradicts data")
            for channel in ("u", "v"):
                lo, hi, total = (fields[k] for k in ("i"+channel+"min", "i"+channel+"max", "is"+channel))
                if not 0 <= lo <= hi <= 4095 or not count*lo <= total <= count*hi:
                    raise ValueError("P1I1 sum/range mismatch")
            if count and ((fields["iend"]-fields["ibeg"]) & 0xFFFFFFFF) > (count-1)*3000:
                raise ValueError("P1I1 capture time/gap mismatch")
            if count and ((fields["t"]-fields["iend"]) & 0xFFFFFFFF) >= 0x80000000:
                raise ValueError("P1I1 snapshot predates capture")
        return {"format": raw[:4].decode("ascii"), **fields}
    if raw.startswith(b"P1R2,"):
        widths = [8, 2, 2, 2, 8, 8, 8, 8, 8, 8, 8, 2, 4]
        pattern = b"P1R2," + b",".join(rb"([0-9A-F]{" + str(w).encode() + rb"})" for w in widths) + rb"\r\n"
        match = re.fullmatch(pattern, raw)
        if not match or len(raw) != 95:
            raise ValueError("P1R2 layout mismatch")
        values = [int(value, 16) for value in match.groups()]
        if binascii.crc_hqx(raw[:89], 0xFFFF) != values[-1]:
            raise ValueError("P1R2 CRC mismatch")
        fields = dict(zip("q c r s f n t id req start end why crc".split(), values))
        if fields["s"] > 5 or fields["r"] & 0x7F > 7 or fields["why"] & 0x3F > 6:
            raise ValueError("P1R2 state/result/reason out of range")
        reason = fields["why"] & 0x3F
        if fields["id"] == 0 and any(fields[k] for k in ("req", "start", "end", "why")):
            raise ValueError("P1R2 missing action contradicts times")
        if fields["id"] and reason == 0:
            raise ValueError("P1R2 action without phase")
        if not fields["why"] & 0x80 and fields["start"] != 0:
            raise ValueError("P1R2 start validity contradicts time")
        if not fields["why"] & 0x40 and fields["end"] != 0:
            raise ValueError("P1R2 end validity contradicts time")
        flags = fields["why"] & 0xC0
        if ((reason == 1 and flags != 0x40) or (reason == 2 and flags != 0)
                or (reason == 3 and flags != 0x80) or (reason in {4, 5, 6} and not flags & 0x40)):
            raise ValueError("P1R2 action phase contradicts timestamp flags")
        return {"format": "P1R2", **fields}
    if raw.startswith(b"P1R1,"):
        # 固定56字节、CRC覆盖0..49含最后逗号；独立使用Python库，不照搬C的CRC循环。
        pattern = rb"P1R1,([0-9A-F]{8}),([0-9A-F]{2}),([0-9A-F]{2}),([0-9A-F]{2}),([0-9A-F]{8}),([0-9A-F]{8}),([0-9A-F]{8}),([0-9A-F]{4})\r\n"
        match = re.fullmatch(pattern, raw)
        if not match or len(raw) != 56:
            raise ValueError("reply frame layout mismatch")
        values = [int(value, 16) for value in match.groups()]
        if binascii.crc_hqx(raw[:50], 0xFFFF) != values[-1]:
            raise ValueError("reply CRC mismatch")
        fields = dict(zip("q c r s f n t crc".split(), values))
        if fields["s"] > 2 or fields["r"] & 0x7F > 7:
            raise ValueError("reply state/result out of range")
        return {"format": "P1R1", **fields}
    text = raw[:-1].removesuffix(b"\r").decode("ascii")
    pieces = text.split(",")
    version = pieces[0]
    if version not in {"M1", "M2C1", "M3B1", "M3B2", "M3B3"}:
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
    if version == "M3B3":
        return parse_compact(fields)
    expected = set(RAW_KEYS + (SENSOR_KEYS if version != "M1" else [])
                   + (FEEDBACK_KEYS if version in {"M3B1", "M3B2"} else [])
                   + (TIMING_KEYS if version == "M3B2" else []))
    if set(fields) != expected or fields["pwm"] != "blocked" or fields["power"] != "unverified":
        raise ValueError("field set or output boundary mismatch")
    for key, value in fields.items():
        if key in {"pwm", "power"}:
            continue
        if key in {"fc", "fa_mrad", "wd_count"}:
            if not -(2**63) <= value < 2**63:
                raise ValueError("signed 64-bit field out of range")
        elif key == "wdt_us":
            if not 0 <= value < 2**64:
                raise ValueError("unsigned 64-bit field out of range")
        elif key in {"iu_mA", "iv_mA", "vd_mrad_s", "vw_mrad_s", "dir"}:
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
    if version != "M1":
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
    if version in {"M3B1", "M3B2"}:
        if any(fields[k] > 1 for k in ("fp", "fd", "fw", "ff")) or fields["dir_ok"] != 0:
            raise ValueError("feedback flags or direction verification invalid")
        if fields["fs"] != 10 or fields["dir"] != 1 or fields["fn"] > 9 or fields["fr"] > 16383:
            raise ValueError("feedback configuration or raw range invalid")
        if fields["fp"]:
            if fields["s"] != 1 or fields["f"] != 0 or not fields["ok"] or not fields["fn"]:
                raise ValueError("feedback validity contradicts runtime")
            if fields["ff"] != int(not fields["fd"]) or bool(fields["fw"]) != (fields["fn"] == 9):
                raise ValueError("feedback warmup contradiction")
            if (fields["ff"] and fields["fn"] != 1) or (fields["fd"] and fields["fn"] < 2):
                raise ValueError("feedback first/sample count contradiction")
            if fields["fd"] and not 0 < fields["fdt"] <= 3000:
                raise ValueError("feedback interval invalid")
        elif any(fields[k] != 0 for k in FEEDBACK_KEYS if k not in {"fo", "fs", "dir", "dir_ok"}):
            raise ValueError("invalid feedback carries numeric result")
        if not fields["fd"] and any(fields[k] for k in ("vd_mrad_s", "fdt")):
            raise ValueError("invalid direct speed carries numeric result")
        if fields["ff"] and any(fields[k] for k in ("fc", "fa_mrad")):
            raise ValueError("first sample has nonzero origin")
        if fields["fw"] and not fields["wdt_us"]:
            raise ValueError("window interval missing")
        if not fields["fw"] and any(fields[k] for k in ("vw_mrad_s", "wd_count", "wdt_us")):
            raise ValueError("invalid window carries numeric result")
    if version == "M3B2":
        if fields["mb_wait_us"] > fields["mb_max_us"] or fields["fb_step_us"] > fields["fb_poll_us"]:
            raise ValueError("timing maximum contradicts observation")
    return {"format": version, **fields}


def save_chunks(chunks, output, metadata):
    """逐块保存原始流，逐行解析；超长/半行/非ASCII/非法行也保存，不过滤失败证据。"""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    info = {**metadata, "tool": "sensor_capture_v1", "python": platform.python_version(),
            "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "started_at_utc": datetime.now(timezone.utc).isoformat(), "valid_lines": 0, "invalid_lines": 0}
    info["invalid_count_unit"] = "error record or overlong physical-line fragment"
    pending = b""
    overlong_line = False
    physical_line = 1
    with (output / "raw.bin").open("wb") as original, (output / "records.jsonl").open("w", encoding="utf-8") as records:
        def store_line(line, forced_error=None):
            """原字节hex与可读替代文本共同保留；解析失败不会删除或改写原始数据。"""
            record = {"host_time_utc": datetime.now(timezone.utc).isoformat(), "raw_hex": line.hex(),
                      "raw_text": line.decode("ascii", errors="replace"), "fields": None, "error": None,
                      "physical_line": physical_line, "is_line_fragment": forced_error is not None}
            try:
                if forced_error is not None:
                    raise ValueError(forced_error)
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
                    # RV-001：固定长度分片不是物理行边界。一旦超长，该行所有尾片段都
                    # 保持错误归属；仅真实换行后恢复正常解析，避免合法格式尾部被提升为采样。
                    if end == 0 or end > 2048:
                        overlong_line = True
                    if end == 0 or end > 2049:
                        end = 2049
                    line, pending = pending[:end], pending[end:]
                    store_line(line, "overlong physical line or continuation fragment" if overlong_line else None)
                    if line.endswith(b"\n"):
                        overlong_line = False
                        physical_line += 1
        except KeyboardInterrupt:
            info["stopped_by_operator"] = True
        finally:
            if pending:
                store_line(pending, "unterminated overlong physical-line fragment" if overlong_line else None)
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
