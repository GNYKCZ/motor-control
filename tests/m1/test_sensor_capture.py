"""只读合成串口输入：验证版本、字段/错误保留、时间回绕及文件元数据；不打开串口。"""
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from sensor_capture import parse_line, save_chunks, TIMING_KEYS


def require(condition):
    if not condition:
        raise RuntimeError("synthetic capture contract failed")


def check_overlong_boundaries(good, temporary, metadata):
    """RV-001：物理坏行内部的合法尾部不是新行，真实换行后才可重新接受。"""
    poisoned = b"x" * 2049 + good + good
    for chunk_size in (len(poisoned), 37, 1, 2048, 2049):
        output = temporary / ("overlong_" + str(chunk_size))
        info = save_chunks([poisoned[i:i+chunk_size] for i in range(0,len(poisoned),chunk_size)], output, metadata)
        records = [json.loads(line) for line in (output / "records.jsonl").read_text(encoding="utf-8").splitlines()]
        require(info["valid_lines"] == 1)
        require([r["physical_line"] for r in records] == [1,1,2])
        require(all(r["fields"] is None and r["is_line_fragment"] for r in records[:-1]))
        require(records[-1]["fields"] is not None and not records[-1]["is_line_fragment"])
        require((output / "raw.bin").read_bytes() == poisoned)
        require(b"".join(bytes.fromhex(r["raw_hex"]) for r in records) == poisoned)
    padded = [good.replace(b"skip=0,", b"skip=" + b"0" * (size-len(good)+1) + b",") for size in (2047,2048,2049)]
    require([len(line) for line in padded] == [2047,2048,2049])
    data = b"".join(padded) + good
    output = temporary / "size_edges"
    info = save_chunks([data[i:i+17] for i in range(0,len(data),17)], output, metadata)
    require(info["valid_lines"] == 3 and info["invalid_lines"] == 1)
    records = [json.loads(line) for line in (output / "records.jsonl").read_text(encoding="utf-8").splitlines()]
    require([r["physical_line"] for r in records] == [1,2,3,4])
    require(records[2]["fields"] is None and records[3]["fields"] is not None)
    require((output / "raw.bin").read_bytes() == data)
    print("PASS RV-001: overlong physical-line ownership, 2048/2049 edges, chunk/newline boundaries and recovery")


def main():
    lines = (ROOT / "tests/m1/synthetic_m2c.txt").read_bytes().splitlines(keepends=True)
    parsed = parse_line(lines[0])
    require(parsed["t"] == 51300 and parsed["iu_mA"] == -822 and parsed["ou_mc"] == 2255500)
    require(parsed["cv"] == 0 and parsed["format"] == "M2C1")
    require(parse_line(lines[1])["u"] == 65535)  # 原始失败证据不能被范围过滤删掉。
    bad = [lines[0].replace(b"cv=0", b"cv=1"), lines[0].replace(b"mv=1", b"mv=0"),
           lines[0].replace(b"M2C1", b"M2C9"), lines[0].replace(b"n=513", b"n=513,n=513"),
           b"\xff\x00\n", lines[0][:-1]]
    for raw in bad:
        try:
            parse_line(raw)
        except (ValueError, UnicodeError):
            pass
        else:
            raise RuntimeError("invalid synthetic line accepted")
    old = b"M1,s=1,f=0,n=1,t=1,e=1,es=0,ec=0,u=0,v=4095,a=16383,mg=0,ok=1,bad=0,skip=0,pwm=blocked,power=unverified\r\n"
    require(parse_line(old)["v"] == 4095)
    # 新版本保留旧格式；64位连续计数/时间不按32位截断，有效标志逐项核验。
    extra = (b"fp=1,fd=1,fw=1,ff=0,fe=513,ft=51272,fr=513,fm=51261,fdt=1000,fn=9,"
             b"fc=-4294967297,fa_mrad=-1647099,vd_mrad_s=-383,vw_mrad_s=-383,"
             b"wd_count=-8,wdt_us=8000,fo=0,fs=10,dir=1,dir_ok=0,")
    new = lines[0].replace(b"M2C1", b"M3B1").replace(b"pwm=blocked,", extra + b"pwm=blocked,")
    require(parse_line(new)["fc"] == -4294967297)
    timing = b"".join((key + "=0,").encode() for key in TIMING_KEYS)
    timed = new.replace(b"M3B1", b"M3B2").replace(b"pwm=blocked,", timing + b"pwm=blocked,")
    require(parse_line(timed)["tp13_us"] == 0)
    compact = (b"M3B3,s=1,f=0,n=17,t=1700,u=2027,v=2028,a=16372,ok=1,fp=1,fd=1,fw=1,"
               b"fo=0,fb_us=110,mb_us=16,log_us=20,log_id=4\r\n")
    require(parse_line(compact)["format"] == "M3B3" and parse_line(compact)["a"] == 16372)
    compact_bad = [compact.replace(b"log_id=4", b"log_id=17"), compact.replace(b"fo=0", b"fo=1"),
                   compact.replace(b"n=17", b"n=4294967296"), compact.replace(b"fp=1", b"fp=0"),
                   compact.replace(b"u=2027", b"u=65535"), compact.replace(b"fd=1", b"fd=0"),
                   compact.replace(b"t=1700", b"t=-1"), compact.replace(b"s=1", b"s=2"),
                   compact.replace(b"log_id=4", b"log_id=4,pwm=blocked")]
    for invalid in compact_bad:
        try: parse_line(invalid)
        except ValueError: pass
        else: raise RuntimeError("invalid compact frame accepted")
    failed = compact.replace(b"s=1,f=0", b"s=2,f=8").replace(b"ok=1,fp=1,fd=1,fw=1", b"ok=0,fp=0,fd=0,fw=0")
    failed = failed.replace(b"u=2027", b"u=65535").replace(b"fo=0", b"fo=1")
    require(parse_line(failed)["u"] == 65535)  # 无效原始证据不因删减协议被抹去。
    for invalid in (timed.replace(b"mb_wait_us=0", b"mb_wait_us=1"),
                    timed.replace(b"fb_step_us=0", b"fb_step_us=1"),
                    timed.replace(b"tp0_us=0", b"tp0_us=4294967296")):
        try: parse_line(invalid)
        except ValueError: pass
        else: raise RuntimeError("invalid M3B2 timing accepted")
    bad_new = [new.replace(b"fp=1", b"fp=0"), new.replace(b"fd=1", b"fd=0"),
               new.replace(b"fw=1", b"fw=0"), new.replace(b"fn=9", b"fn=1"),
               new.replace(b"fs=10", b"fs=1"), new.replace(b"dir_ok=0", b"dir_ok=1"),
               new.replace(b"wdt_us=8000", b"wdt_us=0"),
               new.replace(b"fc=-4294967297", b"fc=-9223372036854775809")]
    for raw in bad_new:
        try: parse_line(raw)
        except ValueError: pass
        else: raise RuntimeError("invalid M3B1 accepted")
    wrap = [old.replace(b"t=1,", b"t=4294967295,"), old.replace(b"t=1,", b"t=0,")]
    stream = b"".join(lines + wrap + [bad[2], b"x" * 3000 + b"\n", bad[-1]])
    metadata = {"source_kind": "synthetic", "firmware_sha": "0" * 40,
                "board": "synthetic", "motor": "synthetic/disconnected", "supply": "none"}
    with tempfile.TemporaryDirectory(dir=ROOT / "build", prefix="capture_test_") as temporary:
        check_overlong_boundaries(lines[0], Path(temporary), metadata)
        mixed = old + new + lines[0] + timed + compact
        mixed_info = save_chunks([mixed[i:i+11] for i in range(0,len(mixed),11)], Path(temporary)/"mixed", metadata)
        require(mixed_info["valid_lines"] == 5 and mixed_info["invalid_lines"] == 0)
        output = Path(temporary) / "saved"
        info = save_chunks([stream[i:i+37] for i in range(0,len(stream),37)], output, metadata)
        require((output / "raw.bin").read_bytes() == stream)
        records = [json.loads(line) for line in (output / "records.jsonl").read_text(encoding="utf-8").splitlines()]
        require(b"".join(bytes.fromhex(record["raw_hex"]) for record in records) == stream)
        require(info["valid_lines"] == 4 and info["invalid_lines"] >= 3)
        require([r["device_time_us"] for r in records if r["fields"]][-2:] == [4294967295,0])
        require(json.loads((output / "metadata.json").read_text(encoding="utf-8"))["source_kind"] == "synthetic")
        try:
            save_chunks([], output, metadata)
        except FileExistsError:
            pass
        else:
            raise RuntimeError("capture overwritten")
        def interrupted():
            yield lines[0][:25]
            raise KeyboardInterrupt
        stopped = Path(temporary) / "stopped"
        final = save_chunks(interrupted(), stopped, metadata)
        require(final["stopped_by_operator"] and final["invalid_lines"] == 1)
        require((stopped / "raw.bin").read_bytes() == lines[0][:25])
    print("PASS synthetic capture: versioned fields, raw/error/partial retention, wrap, metadata, bounded lines, stop/no overwrite")


if __name__ == "__main__":
    main()
