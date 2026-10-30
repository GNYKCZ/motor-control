"""离线读取既有raw.bin，汇总静止DISABLED日志的ADC零偏候选；不打开串口、不写参数。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from sensor_capture import parse_line


def summarize(raw):
    """整段拒绝故障/坏行/运动/已尝试动作，不挑选安静前缀；稀疏日志不冒充512帧MCU候选。"""
    if not raw or len(raw) > 1024 * 1024:
        raise ValueError("需要非空、最多1MiB的短记录")
    samples = []
    batches = []
    for line in raw.splitlines(keepends=True):
        record = parse_line(line)
        if record["format"] == "P1Z1":
            if record["f"] or record["s"] != 1 or record["r"] or record["fl"] & 2:
                raise ValueError("完整候选查询含故障/中止，不用于零偏")
            batches.append(record)
        elif record["format"] == "P1R2":
            if record["id"] or record["f"] or record["s"] != 1 or record["r"]:
                raise ValueError("记录含动作、故障或失败查询，不用于本次零偏候选")
        elif record["format"] == "M3B3":
            if any(record[k] != v for k, v in {"s": 1, "f": 0, "ok": 1,
                    "fp": 1, "fd": 1, "fw": 1, "fo": 0}.items()):
                raise ValueError("整段必须为有效无故障DISABLED暖机后记录")
            samples.append(record)
        else:
            raise ValueError("仅接受本批M3B3/P1R2原始帧")
    if len(samples) < 32 and not batches:
        raise ValueError("至少32条周期日志；这是报告最小样本量，不是校准通过门限")
    position = low = high = 0
    for previous, current in zip(samples, samples[1:]):
        delta = (current["a"] - previous["a"] + 8192) % 16384 - 8192
        position += delta
        low, high = min(low, position), max(high, position)
        dt = (current["t"] - previous["t"]) & 0xFFFFFFFF
        dn = (current["n"] - previous["n"]) & 0xFFFFFFFF
        # 当前M3B3每秒一条，允许至多3秒的日志间隔；不是原始采样新鲜度门限。
        if not 0 < dt <= 3000000 or not 0 < dn < 0x80000000:
            raise ValueError("日志时间/序号间断、重复或倒序")
    if high - low >= 16:
        raise ValueError("角度变化达到16count，不符合本次静止候选条件")
    channels = {}
    for channel in ("u", "v"):
        values = [s[channel] for s in samples]
        if not values:
            continue
        mean = math.fsum(values) / len(values)
        channels[channel] = {"mean_count": mean, "min_count": min(values), "max_count": max(values),
            "sample_stddev_count": math.sqrt(math.fsum((v - mean) ** 2 for v in values) / (len(values) - 1)) if len(values) > 1 else None}
    report = {"kind": "sparse_log_zero_candidate", "calibration_verified": False,
        "parameters_applied": False, "records": len(samples), "channels": channels,
        "angle_span_count": high - low if len(samples) > 1 else None, "raw_sha256": hashlib.sha256(raw).hexdigest(),
        "limits": "周期日志子采样；不证明完整原始流、独立零电流、绝对增益或真实相序；不得据此使能闭环"}
    if batches:
        batch = batches[-1]
        if batch["cn"] != 512 or batch["fl"] != 0x1D:
            raise ValueError("需要显式重开后完成的512帧静止候选；不使用自动/部分批次")
        report["kind"] = "mcu_512_zero_candidate"
        report["full_mcu_batch"] = batch
        report["mean_u_count"] = batch["su"] / 512
        report["mean_v_count"] = batch["sv"] / 512
        report["limits"] = "显式完整512帧候选；静止仅软件辅助，零电流条件/真实增益/相序未独立验证，不自动应用或使能闭环"
    return report


def compare_reports(reports):
    """只比较同启动/同声明条件的独立批次；不把重读同一统计当重复性。只报告差异，不自动通过。"""
    if len(reports) < 2 or any(r["kind"] != "mcu_512_zero_candidate" for r in reports):
        raise ValueError("重复性需要至少两个完整MCU批次")
    keys = ("firmware_sha", "board", "motor", "supply")
    first = reports[0]
    for report in reports:
        if any(not first.get("capture_metadata", {}).get(k) or
               report.get("capture_metadata", {}).get(k) != first["capture_metadata"][k] for k in keys):
            raise ValueError("批次固件/板卡/电机/供电声明不一致或缺失")
        if any(report["full_mcu_batch"][k] != first["full_mcu_batch"][k] for k in ("cfg", "scale")):
            raise ValueError("批次采样配置或名义比例不同")
    for before, after in zip(reports, reports[1:]):
        a, b = before["full_mcu_batch"], after["full_mcu_batch"]
        if not 0 < (b["beg"] - a["end"]) & 0xFFFFFFFF < 0x80000000:
            raise ValueError("候选重复、重叠或设备时间重启/倒序；不能算独立批次")
        if not 512 <= (b["n"] - a["n"]) & 0xFFFFFFFF < 0x80000000:
            raise ValueError("两批间没有足够的新原始帧")
    return {"kind": "zero_repeatability_observation", "batches": reports,
        "mean_span_u_count": max(r["mean_u_count"] for r in reports) - min(r["mean_u_count"] for r in reports),
        "mean_span_v_count": max(r["mean_v_count"] for r in reports) - min(r["mean_v_count"] for r in reports),
        "calibration_verified": False, "parameters_applied": False,
        "limits": "仅报告同声明条件的批次差异；未设置自动通过门限，不证明独立零电流或真实增益"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, nargs="+", help="一个或多个既有采集目录，含raw.bin和metadata.json")
    args = parser.parse_args()
    reports = []
    for capture in args.capture:
        metadata = json.loads((capture / "metadata.json").read_text(encoding="utf-8"))
        report = summarize((capture / "raw.bin").read_bytes())
        report["capture_metadata"] = metadata  # 元数据是声明，不升级为已验证硬件身份。
        reports.append(report)
    print(json.dumps(reports[0] if len(reports) == 1 else compare_reports(reports), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
