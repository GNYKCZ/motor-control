"""只读零偏候选报告：独立手算均值/样本方差、回绕及整段拒绝，不连接硬件。"""
from pathlib import Path
import math
import binascii
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from zero_offset_report import summarize, compare_reports


def line(i, u=2000, v=2100, angle=16383, state=1, fault=0, interval_us=100000):
    return (f"M3B3,s={state},f={fault},n={i+1},t={(0xFFFF0000+i*interval_us)&0xFFFFFFFF},"
            f"u={u},v={v},a={angle},ok=1,fp=1,fd=1,fw=1,fo=0,fb_us=1,mb_us=1,log_us=1,log_id=1\r\n").encode()


raw = b"".join(line(i, 2000 + i % 2, 2100 + 2 * (i % 2), i % 2 * 16383) for i in range(32))
result = summarize(raw)
slow_raw = b"".join(line(i, interval_us=1000000) for i in range(32))
assert summarize(slow_raw)["records"] == 32  # 每秒一条，跨设备时间回绕仍可分析。
assert result["records"] == 32 and result["angle_span_count"] == 1
assert result["channels"]["u"]["mean_count"] == 2000.5
assert result["channels"]["v"]["mean_count"] == 2101
assert abs(result["channels"]["u"]["sample_stddev_count"] - math.sqrt(8 / 31)) < 1e-12
assert not result["calibration_verified"] and not result["parameters_applied"]
for invalid in (raw + line(32, state=2, fault=32), raw + b"bad\r\n", raw[:-1],
                raw + line(32, angle=40), raw + line(31), b"".join(line(i) for i in range(31))):
    try:
        summarize(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("invalid whole capture was accepted")
print("PASS offline zero candidates: independent means/variance, clock/angle wrap, whole-capture rejection; hardware NOT RUN")


def batch(flags=0x1D, count=512, total=1024000):
    # 独立库CRC，显式字段宽度；512*2000与512*2100为手算常量。
    prefix = (f"P1Z1,00000003,05,00,01,00000000,00000200,00080000,{count:04X},{total:08X},00106800,"
              f"07D0,07D0,0834,0834,{flags:02X},000C4B20,FFFFFF00,{(0xFFFFFF00+511000)&0xFFFFFFFF:08X},00020001,").encode()
    return prefix + f"{binascii.crc_hqx(prefix,0xFFFF):04X}\r\n".encode()


full = summarize(batch())
assert full["kind"] == "mcu_512_zero_candidate" and full["mean_u_count"] == 2000 and full["mean_v_count"] == 2100
assert not full["calibration_verified"] and not full["parameters_applied"]
for invalid in (batch(flags=0x0D), batch(flags=0x1F), batch(count=511), batch(total=1024001)):
    try:
        summarize(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("invalid full candidate accepted")
print("PASS P1Z1 full MCU candidate: explicit batch, wrap, sums, incomplete/aborted/auto rejection")

import copy
full["capture_metadata"] = dict(firmware_sha="a" * 40, board="synthetic", motor="synthetic", supply="synthetic")
second = copy.deepcopy(full)
second["full_mcu_batch"]["beg"] = (full["full_mcu_batch"]["beg"] + 2000000) & 0xFFFFFFFF
second["full_mcu_batch"]["end"] = (full["full_mcu_batch"]["end"] + 2000000) & 0xFFFFFFFF
second["full_mcu_batch"]["n"] += 2000
second["mean_u_count"] += 0.5
comparison = compare_reports([full, second])
assert comparison["mean_span_u_count"] == 0.5 and comparison["mean_span_v_count"] == 0
for invalid in ([full, full], [second, full]):
    try:
        compare_reports(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("repeated or backwards candidates accepted")
second["capture_metadata"]["supply"] = "different"
try:
    compare_reports([full, second])
except ValueError:
    pass
else:
    raise AssertionError("different supply silently combined")
print("PASS repetition: independent same-condition batches, duplicate/order/config rejection; no automatic calibration pass")

# 带流查询的独立手算帧：19帧U2100/V1990，零偏2027/2028，差值+73/-38 count。
from sensor_capture import parse_line
prefix = ("P1I1,00000004,07,00,01,00000000,000002BC,000AAE60,0200,000FD600,000FD800,"
          "07EB,07EB,07EC,07EC,19,000C4B20,00000000,0007CC18,00020001,0013,"
          f"{19*2100:08X},{19*1990:08X},0834,0834,07C6,07C6,00092C0C,0009725C,0FFF,0FFF,00,00000001,").encode()
frame = prefix + f"{binascii.crc_hqx(prefix,0xFFFF):04X}\r\n".encode()
current = parse_line(frame)
assert len(frame) == 221 and current["ic"] == 19
assert current["isu"]/19-current["su"]/512 == 73
assert current["isv"]/19-current["sv"]/512 == -38
for bad_prefix in (prefix.replace(b"0013,",b"0021,"), prefix.replace(b"0834,0834,",b"0835,0834,"),
                   prefix.replace(b"0FFF,0FFF,00,00000001,",b"4000,0FFF,00,")):
    try:
        parse_line(bad_prefix + f"{binascii.crc_hqx(bad_prefix,0xFFFF):04X}\r\n".encode())
    except ValueError:
        pass
    else:
        raise AssertionError("invalid current capture accepted despite recomputed CRC")
print("PASS P1I1: independent nonempty sums/signs, count/range/angle rejection")
