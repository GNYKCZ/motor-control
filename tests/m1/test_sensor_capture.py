"""只读合成串口输入：验证版本、字段/错误保留、时间回绕及文件元数据；不打开串口。"""
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from sensor_capture import parse_line, save_chunks


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
    wrap = [old.replace(b"t=1,", b"t=4294967295,"), old.replace(b"t=1,", b"t=0,")]
    stream = b"".join(lines + wrap + [bad[2], b"x" * 3000 + b"\n", bad[-1]])
    metadata = {"source_kind": "synthetic", "firmware_sha": "0" * 40,
                "board": "synthetic", "motor": "synthetic/disconnected", "supply": "none"}
    with tempfile.TemporaryDirectory(dir=ROOT / "build", prefix="capture_test_") as temporary:
        check_overlong_boundaries(lines[0], Path(temporary), metadata)
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
