"""独立几何/帧布局、时间回绕与错误输入；不运行硬件。"""
import binascii
import copy
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"tools"))
from sensor_capture import parse_line
from alignment_report import analyze, combine
from commutation_report import circle_difference


def frame(p=7, d=1, side=1, origin=16300, updates=None, displacement=None):
    # 两个独立稳定位置，目标机械count为origin+200；预置相差1/6电周。
    move = side*d*round(16384/(6*p)) if displacement is None else displacement
    req=0xffff0000; start=(req+1000)&0xffffffff
    end0=(start+200000)&0xffffffff; start1=(end0+1999)&0xffffffff; end1=(start1+200000)&0xffffffff
    values=[1,8,0,1,0,1,side&255,1,1,1,3,origin,start,end1,0x20001,req]
    widths=[8,2,2,2,8,8,2,2,2,2,2,4,8,8,8,8]
    for position,beg,end in ((200+move,start,end0),(200,start1,end1)):
        values += [50,(position*50)&0xffffffff,position&65535,position&65535,beg,end]
        widths += [2,8,4,4,8,8]
    if updates:
        for index,value in updates.items(): values[index]=value
    prefix=b"P1A2,"+b"".join(f"{v:0{w}X},".encode() for v,w in zip(values,widths))
    return prefix+f"{binascii.crc_hqx(prefix,0xffff):04X}\r\n".encode()


for p in (1,5,7,11,32):
    for d in (-1,1):
        reports=[]
        for side in (-1,1):
            raw=frame(p,d,side)
            assert len(raw)==183
            record=parse_line(raw)
            r=analyze(record,p,d)
            expected=(-d*p*(16300+200)*2*math.pi/16384)%(2*math.pi)
            assert abs(circle_difference(r["electrical_zero_rad"],expected))<1e-12
            assert r["candidate_derived"] and not r["repeatability_checked"]
            assert not r["parameters_applied"] and not r["calibration_verified"]
            reports.append(r)
        combined=combine(*reports)
        assert combined["repeatability_checked"] and combined["difference_electrical_rad"]==0
        assert not combined["calibration_verified"] and not combined["parameters_applied"]


def rejects(action):
    try: action()
    except ValueError: return
    raise AssertionError("invalid alignment accepted")


for updates in ({14:0x10001},{7:2},{10:127},{16:39},{16:65},{18:100,19:116},
                {17:0x7fffffff},{21:100},{27:0x10000},{13:0x100000},{6:0},{2:6}):
    rejects(lambda: parse_line(frame(updates=updates)))
for displacement in (0,32,-390):
    rejects(lambda: analyze(parse_line(frame(displacement=displacement)),7,1))
record=parse_line(frame())
for p,d in ((0,1),(33,1),(7,0)):
    rejects(lambda: analyze(record,p,d))
first=analyze(record,7,1)
second=analyze(parse_line(frame(side=-1)),7,1)
rejects(lambda: combine(first,first))
wrong=copy.deepcopy(second);wrong["electrical_zero_rad"]+=math.radians(15.001)
rejects(lambda: combine(first,wrong))
wrong=copy.deepcopy(second);wrong["pole_pairs_assumed"]=5
rejects(lambda: combine(first,wrong))

# 已有实测：U首末圆差小，与全七点拟合FAIL必须同时保留。
raw=("P1A1,00000003,08,00,01,00000000,00000001,01,06,01,01,7F,372F,05756AFD,058ADCE3,00010001,"
     "32,00000DDE,0047,0047,32,0000814C,0296,0296,32,0000C1F2,03E1,03E1,32,0000F712,04F1,04F1,"
     "32,000169EA,073D,073D,32,0001B328,08B4,08B4,32,0001D3F8,095C,095C,7B44\r\n").encode()
observed=analyze(parse_line(raw),7,1)
assert observed["candidate_derived"] and observed["full_seven_point_analysis"]["status"]=="FAIL"
assert abs(math.degrees(observed["electrical_zero_rad"])-337.423095703125)<1e-9
assert abs(math.degrees(observed["same_action_difference_electrical_rad"])-2.39501953125)<1e-9
assert not observed["repeatability_checked"]
rejects(lambda: combine(observed,observed))
print("PASS fixed-U conditional zero: p1/5/7/11/32, d+/-, both presets, encoder/time wrap, malformed timing/statistics/config, no-motion/sign/duplicate rejection; original seven-point FAIL preserved")

# 固定提示只是人工观察辅助，不带CRC，必须标为diagnostic_only。
for hint in ("WAIT_STILL","COLLECT_ZERO","READY_PRESS_PA12","FAILED_INPUT","FAILED_NOT_STILL",
             "FAILED_ZERO_INVALID","FAILED_ZERO_TIMEOUT","FAILED_STOP","ACTION_DONE","ACTION_FAILED"):
    item=parse_line(("P1F1,"+hint+"\r\n").encode())
    assert item["diagnostic_only"] and item["hint"]==hint
for raw in (b"P1F1,READY\r\n",b"P1F1,READY_PRESS_PA12\n",b"P1F1,ACTION_DONE,extra\r\n"):
    rejects(lambda: parse_line(raw))
print("PASS automatic diagnostic hints: exact finite layout; no CRC/timestamp authority claimed")

assert parse_line(b"FW1,31-13-24,ALIGN_POS\r\n")["version"]=="31-13-24"
for raw in (b"FW1,00-13-24,ALIGN_POS\r\n",b"FW1,31-24-24,ALIGN_POS\r\n",b"FW1,31-13-60,ALIGN_POS\r\n",b"FW1,31-13-24,UNKNOWN\r\n"):
    rejects(lambda: parse_line(raw))
