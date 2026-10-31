"""固定U矢量电零位的条件候选；只读摘录文件，不连接串口或应用参数。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from commutation_report import analyze as analyze_scan, circle_difference
from sensor_capture import parse_line


def mean_angle(angles):
    return math.atan2(math.fsum(math.sin(a) for a in angles),
                      math.fsum(math.cos(a) for a in angles)) % (2*math.pi)


def analyze(record, pole_pairs, encoder_direction):
    """p/d是调用者明确提供的前提，不从同一U点推断并自证。"""
    if type(pole_pairs) is not int or not 1 <= pole_pairs <= 32 or encoder_direction not in (-1, 1):
        raise ValueError("必须显式提供p=1..32及d=+1/-1")
    if record["format"] not in ("P1A1", "P1A2") or record["r"] or record["s"] != 1 or record["f"] or not record["done"] or record["reason"] != 1:
        raise ValueError("需要无故障、正常完成的矢量结果")
    alignment = record["format"] == "P1A2"
    if record["mask"] != (3 if alignment else 127) or record["step"] != (1 if alignment else 6):
        raise ValueError("缺少完整保持点")
    reference = record["requested"] if alignment else record["start"]
    if not 0 < (record["end"]-reference) & 0xffffffff <= (420000 if alignment else 1420000):
        raise ValueError("矢量动作超时")
    positions = []
    for point in record["points"]:
        if not 40 <= point["count"] <= 64 or point["maximum"]-point["minimum"] >= 16:
            raise ValueError("保持点缺样本或未稳定")
        positions.append(record["origin"]+point["sum_count"]/point["count"])
    zero = lambda position: (-encoder_direction*pole_pairs*position*2*math.pi/16384) % (2*math.pi)
    report = dict(pole_pairs_assumed=pole_pairs, encoder_direction_assumed=encoder_direction,
                  reference_vector="U: phi=0, ticks=(60,0,0)", configuration=record["cfg"],
                  repeatability_checked=False, calibration_verified=False, parameters_applied=False,
                  source=record, limits="条件于p/d及电压矢量实际对齐；不证明绝对精度或硬件身份")
    if alignment:
        delta = positions[1]-positions[0]
        if abs(delta) <= 32 or delta*record["direction"]*encoder_direction >= 0:
            raise ValueError("预置至U无明确净位移或方向不符")
        report.update(candidate_derived=True, electrical_zero_rad=zero(positions[1]),
                      preset_side=record["direction"], net_displacement_count=delta,
                      nominal_displacement_count=-record["direction"]*encoder_direction*16384/(6*pole_pairs))
    else:
        offsets = [zero(positions[i]) for i in (0, 6)]
        difference = abs(circle_difference(*offsets))
        consistent = difference <= math.pi/12
        try:
            full = dict(status="PASS", report=analyze_scan(record))
        except ValueError as error:
            full = dict(status="FAIL", reason=str(error))
        report.update(candidate_derived=consistent, electrical_zero_rad=mean_angle(offsets) if consistent else None,
                      same_action_difference_electrical_rad=difference, independent_actions=1,
                      full_seven_point_analysis=full)
    return report


def combine(first, second):
    """相反预置的两份有效结果才检查重复性；同一次扫描首末点不满足。"""
    if not all(r["source"]["format"] == "P1A2" for r in (first, second)):
        raise ValueError("重复性需要两种预置模式的独立动作")
    if first["preset_side"] == second["preset_side"] or any(first[k] != second[k] for k in
            ("pole_pairs_assumed", "encoder_direction_assumed", "configuration")):
        raise ValueError("预置侧/配置/p/d不匹配")
    difference = abs(circle_difference(first["electrical_zero_rad"], second["electrical_zero_rad"]))
    if difference > math.pi/12:
        raise ValueError("两侧U零位圆差超过15度电角")
    return dict(candidate_derived=True, repeatability_checked=True, calibration_verified=False,
                parameters_applied=False, pole_pairs_assumed=first["pole_pairs_assumed"],
                encoder_direction_assumed=first["encoder_direction_assumed"],
                electrical_zero_rad=mean_angle([first["electrical_zero_rad"], second["electrical_zero_rad"]]),
                difference_electrical_rad=difference, actions=[first, second],
                limits="不同预置/相反净位移下的条件重复性；实际独立启动及固件/硬件身份需现场记录，不证明绝对精度")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame", type=Path, nargs="+")
    parser.add_argument("--pole-pairs", type=int, required=True)
    parser.add_argument("--encoder-direction", type=int, choices=(-1, 1), required=True)
    args = parser.parse_args()
    if len(args.frame) not in (1, 2):
        parser.error("只接受一个摘录或两种预置的两份摘录")
    reports = []
    for path in args.frame:
        raw = path.read_bytes()
        report = analyze(parse_line(raw), args.pole_pairs, args.encoder_direction)
        report.update(source_file=str(path), source_sha256=hashlib.sha256(raw).hexdigest())
        reports.append(report)
    print(json.dumps(reports[0] if len(reports) == 1 else combine(*reports), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
