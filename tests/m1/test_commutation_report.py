"""独立几何轨迹、非默认极对数、正反/零位/时间回绕与失败拒绝；不运行电机。"""
import binascii
import copy
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from sensor_capture import parse_line
from commutation_report import analyze,combine,circle_difference


def frame(pairs=7,mapping=1,direction=1,origin=16000):
    # 几何关系：一电周对应16384/p机械计数；独立round到整数，50个±1噪声相消。
    values=[1,8,0,1,0,1,direction & 255,6,1,1,127,origin,0xfffff000,(0xfffff000+1400600)&0xffffffff,0x10001]
    widths=[8,2,2,2,8,8,2,2,2,2,2,4,8,8,8]
    for k in range(7):
        position=round(100+mapping*direction*k*16384/(6*pairs))
        values += [50,(50*position)&0xffffffff,(position-1)&65535,(position+1)&65535]
        widths += [2,8,4,4]
    prefix=b"P1A1,"+b"".join(f"{v:0{w}X},".encode() for v,w in zip(values,widths))
    return prefix+f"{binascii.crc_hqx(prefix,0xffff):04X}\r\n".encode()


for pairs in (5,7,11):
    for mapping in (-1,1):
        reports=[]
        for direction in (-1,1):
            r=analyze(parse_line(frame(pairs,mapping,direction)))
            assert r["pole_pairs"]==pairs and r["encoder_direction"]==mapping
            oracle=(-mapping*pairs*(16000+100)*2*math.pi/16384)%(2*math.pi)
            assert abs(circle_difference(r["electrical_zero_rad"],oracle))<0.004
            assert not r["calibration_verified"] and not r["parameters_applied"]
            reports.append(r)
        result=combine(*reports)
        assert result["pole_pairs"]==pairs and result["zero_difference_electrical_rad"]<0.004

forward=analyze(parse_line(frame()))
reverse=analyze(parse_line(frame(direction=-1,origin=(16000+round(16384/7))%16384)))
assert combine(forward,reverse)["zero_difference_electrical_rad"]<0.004
for raw in (frame(pairs=7.4),frame(pairs=20)):
    try: analyze(parse_line(raw))
    except ValueError: pass
    else: raise AssertionError("inconsistent/ambiguous pole count accepted")
base=parse_line(frame())
bad=copy.deepcopy(base);bad["points"][3]["sum_count"]=bad["points"][2]["sum_count"]
short=copy.deepcopy(base);short["points"][1]["count"]=39
fault=copy.deepcopy(base);fault["f"]=32
for record in (bad,short,fault):
    try: analyze(record)
    except ValueError: pass
    else: raise AssertionError("bad trajectory accepted")
wrong=copy.deepcopy(reverse);wrong["electrical_zero_rad"]+=0.4
for pair in ((forward,forward),(forward,wrong)):
    try: combine(*pair)
    except ValueError: pass
    else: raise AssertionError("same direction/disagreeing zero accepted")
print("PASS scan geometry: p5/7/11, both encoder/command signs, quantization/noise, origin/time wrap, ambiguity/missing/nonmonotonic/zero disagreement; no hardware")
