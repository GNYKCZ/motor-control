"""实际帧/runtime/M3B3导出及独立整数codec验收；只替换外设，不操作硬件。"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from sensor_capture import parse_line


def main():
    """运行真实模块、边界与小容量负向构建；失败也保留当前输入/命令/输出证据。"""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", type=Path, required=True)
    args = parser.parse_args()
    gcc = args.gcc.resolve()
    fw = root / "firmware/stm32f103"
    output = root / "build/host_m1"
    output.mkdir(parents=True, exist_ok=True)
    evidence_file = output / "test_evidence.json"
    if evidence_file.exists() and json.loads(evidence_file.read_text()).get("exit_code") != 0:
        (output / ("failed_attempt_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")).write_bytes(evidence_file.read_bytes())
    environment = os.environ.copy()
    environment["PATH"] = str(gcc.parent) + os.pathsep + environment.get("PATH", "")
    sources = [fw / p for p in ("board/sample_frame.c", "app/motor_runtime.c", "app/telemetry.c")]
    sources += [root / "tests/m1/runtime_gate_stub.c"]
    sources += [root / "control_core/sensor_input.c", root / "control_core/feedback.c", root / "tests/m1/test_sampling.c"]
    unit = root / "tests/m1/test_telemetry_compact.c"
    inputs = sources + [unit, root / "tools/sensor_capture.py"]
    inputs += list((fw / "app").glob("*.h")) + list((fw / "board").glob("*.h"))
    inputs += [root / "control_core/sensor_input.h", root / "control_core/feedback.h"]
    evidence = {"head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                "worktree_status": subprocess.check_output(["git", "status", "--porcelain=v1"], cwd=root, text=True),
                "source_sha256": {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
                "test_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "steps": [], "exit_code": 1}
    common = ["-std=c11", "-D__USE_MINGW_ANSI_STDIO=1", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-pedantic"]
    includes = ["-I", str(fw / "board"), "-I", str(fw / "app"), "-I", str(root / "control_core")]
    transcript = []

    def run(command):
        """不通过shell拼编译参数；每个结果先保存，失败非零退出。"""
        command = list(map(str, command))
        result = subprocess.run(command, cwd=root, env=environment, text=True, encoding="utf-8", errors="replace",
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        evidence["steps"].append({"command": command, "returncode": result.returncode, "output": result.stdout})
        transcript.append(result.stdout)
        if result.stdout.strip(): print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
        if result.returncode: raise RuntimeError("command failed; see test_evidence.json")
        return result.stdout

    try:
        evidence["compiler_version"] = run([gcc, "--version"])
        command = [gcc, *common, *includes, *sources, "-lm", "-o", output / "test_sampling.exe"]
        evidence["command"] = list(map(str, command))
        run(command)
        text = run([output / "test_sampling.exe"])
        records = [parse_line((line + "\n").encode("ascii")) for line in text.splitlines() if line.startswith("M3B3,")]
        if len(records) != 3 or [p["n"] for p in records] != [17, 27, 97] or [p["a"] for p in records] != [16380, 6, 76]:
            raise RuntimeError("runtime -> compact telemetry -> capture input mismatch")
        if [p["fd"] for p in records] != [0, 1, 1] or [p["fw"] for p in records] != [0, 0, 1]:
            raise RuntimeError("compact estimator progress mismatch")
        run([gcc, *common, *includes, fw / "app/telemetry.c", unit, "-o", output / "test_compact.exe"])
        text = run([output / "test_compact.exe"])
        extreme = [parse_line((line + "\n").encode("ascii")) for line in text.splitlines() if line.startswith("M3B3,")]
        if len(extreme) != 1 or extreme[0]["n"] != 2**32 - 1 or extreme[0]["log_id"] != 16:
            raise RuntimeError("integer boundary record mismatch")
        # 小容量变体仅用于负向构建，不更改正式头文件；必须拒绝半行提交。
        short = output / "small_config"
        short.mkdir(exist_ok=True)
        config = (fw / "board/board_config.h").read_text(encoding="utf-8")
        config, count = re.subn(r"(#define BOARD_SERIAL_BUFFER_BYTES )\d+u", r"\g<1>8u", config)
        if count != 1: raise RuntimeError("buffer capacity configuration missing")
        (short / "board_config.h").write_bytes(config.encode("utf-8"))
        run([gcc, *common, "-DTEST_SMALL_BUFFER", "-I", short, *includes, fw / "app/telemetry.c", unit,
             "-o", output / "test_compact_small.exe"])
        run([output / "test_compact_small.exe"])
        # 只约束本次telemetry热点依赖，不能据此禁止整个控制核使用数学/名义转换模块。
        obj = output / "telemetry_compact.o"
        run([gcc, *common, *includes, "-c", fw / "app/telemetry.c", "-o", obj])
        symbols = run([gcc.parent / "nm.exe", "--undefined-only", obj])
        if re.search(r"printf|sensor_input_convert|motor_runtime_read_status", symbols):
            raise RuntimeError("telemetry retained heavy formatting/conversion/full-snapshot dependency")
        names = "s f n t u v a ok fp fd fw fo fb_us mb_us log_us log_id".split()
        widths = [1, 10, 10, 10, 5, 5, 5, 1, 1, 1, 1, 10, 10, 10, 10, 2]
        maximum = 5 + sum(len(k) + 1 + n for k, n in zip(names, widths)) + 15 + 2
        conservative = 5 + sum(len(k) + 1 + 10 for k in names) + 15 + 2
        capacity = int(re.search(r"#define BOARD_SERIAL_BUFFER_BYTES (\d+)u", (fw / "board/board_config.h").read_text())[1])
        if maximum != 169 or conservative != 237 or conservative + 1 > capacity:
            raise RuntimeError("compact length bound/capacity mismatch")
        evidence.update({"frame_bound_bytes": maximum, "all_uint32_bound_bytes": conservative,
                         "wire_time_ms_at_115200_8N1": maximum * 10 / 115200 * 1000,
                         "conversion_budget": "one uint32 per poll, at most 10 decimal digits", "exit_code": 0})
        print("PASS M3B3: real runtime -> telemetry -> capture; codec boundaries, no heavy export dependencies; 169B / 14.67ms")
    except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired) as error:
        evidence["error"] = str(error)
        print("FAIL:", error)
    finally:
        evidence["output"] = "\n".join(transcript)
        evidence["head_after"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        evidence["worktree_after"] = subprocess.check_output(["git", "status", "--porcelain=v1"], cwd=root, text=True)
        if (evidence["worktree_after"] != evidence["worktree_status"] or evidence["head_after"] != evidence["head"]
                or any(hashlib.sha256((root / name).read_bytes()).hexdigest() != digest
                       for name, digest in evidence["source_sha256"].items())
                or hashlib.sha256(Path(__file__).read_bytes()).hexdigest() != evidence["test_script_sha256"]):
            evidence["exit_code"] = 1
            evidence["error"] = "source/head/worktree changed during tests"
        evidence_file.write_bytes((json.dumps(evidence, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    raise SystemExit(evidence["exit_code"])


if __name__ == "__main__":
    main()
