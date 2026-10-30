"""离线分析一个/两个完整P1A1汇总帧；不打开串口、不写Flash、不应用参数。"""
import argparse
import json
import math
from pathlib import Path
from sensor_capture import parse_line


def circle_difference(a, b):
    return (a-b+math.pi) % (2*math.pi)-math.pi


def analyze(record):
    """独立已知电压角→机械角拟合；单方向只出候选，不把商家p7当输入。"""
    if record["format"]!="P1A1" or record["r"] or record["s"]!=1 or record["f"] or not record["done"] or record["mask"]!=127:
        raise ValueError("需要无故障、七点完整、正常结束的P1A1")
    if not 0 < (record["end"]-record["start"]) & 0xFFFFFFFF <= 1420000:
        raise ValueError("扫描起止时间无效/超出1.42秒")
    positions=[]
    for point in record["points"]:
        if not 40<=point["count"]<=64 or point["maximum"]-point["minimum"]>=16:
            raise ValueError("保持点缺帧/未稳定")
        positions.append(point["sum_count"]/point["count"])
    delta=positions[-1]-positions[0]
    if not delta or any((b-a)*delta<=0 for a,b in zip(positions,positions[1:])):
        raise ValueError("无位移或各点方向不一致")
    encoder_direction=1 if record["direction"]*delta>0 else -1
    theta=[(record["origin"]+p)*2*math.pi/16384 for p in positions]
    command=[record["direction"]*k*math.pi/3 for k in range(7)]
    candidates=[]
    for pairs in range(1,33):
        pitch=16384/pairs
        if abs(abs(delta)-pitch)>0.05*pitch:
            continue
        offsets=[circle_difference(phi,encoder_direction*pairs*angle) for phi,angle in zip(command,theta)]
        zero=math.atan2(math.fsum(math.sin(v) for v in offsets),math.fsum(math.cos(v) for v in offsets))
        residual=max(abs(circle_difference(v,zero)) for v in offsets)
        if residual<=math.pi/12:
            candidates.append(dict(pole_pairs=pairs,encoder_direction=encoder_direction,
                electrical_zero_rad=zero%(2*math.pi),max_residual_electrical_rad=residual))
    if len(candidates)!=1:
        raise ValueError("极对数候选不唯一或没有通过一致性检查")
    return {**candidates[0],"command_direction":record["direction"],"mechanical_span_count":delta,
        "origin_count":record["origin"],"configuration":record["cfg"],"source":record,
        "candidate_valid":True,"calibration_verified":False,"parameters_applied":False,
        "limits":"单方向几何候选；需正反一致，未证明绝对角度/电流增益或实际磁场对齐误差"}


def combine(first, second):
    """只合并两个方向独立取得的同配置候选；不强求机械起点相同。"""
    if first["command_direction"]==second["command_direction"]:
        raise ValueError("需要正反两个方向，不能重读同方向充数")
    if any(first[k]!=second[k] for k in ("pole_pairs","encoder_direction","configuration")):
        raise ValueError("正反极对数/映射方向/配置不一致")
    difference=abs(circle_difference(first["electrical_zero_rad"],second["electrical_zero_rad"]))
    if difference>math.pi/12:
        raise ValueError("正反零位差超过15度电角")
    a,b=first["electrical_zero_rad"],second["electrical_zero_rad"]
    zero=math.atan2(math.sin(a)+math.sin(b),math.cos(a)+math.cos(b))%(2*math.pi)
    return {"pole_pairs":first["pole_pairs"],"encoder_direction":first["encoder_direction"],
        "electrical_zero_rad":zero,"zero_difference_electrical_rad":difference,
        "formula":"theta_e_rad = wrap_0_2pi(encoder_direction * pole_pairs * theta_m_rad + electrical_zero_rad)",
        "candidate_valid":True,"calibration_verified":False,"parameters_applied":False,
        "forward_reverse":[first,second],"limits":"正反一致的几何参数候选；元数据/实物身份仍需核对，不自动应用，不证明绝对精度"}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame",type=Path,nargs="+",help="仅含一个完整P1A1的摘录文件；不是完整串口捕获声明")
    args=parser.parse_args()
    if len(args.frame) not in (1,2):
        parser.error("只能给一个或正反两个汇总帧文件")
    reports=[analyze(parse_line(p.read_bytes())) for p in args.frame]
    print(json.dumps(reports[0] if len(reports)==1 else combine(*reports),ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
