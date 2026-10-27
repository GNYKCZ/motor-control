"""M3a反馈模块主机验收：独立编译真实core和oracle，不链接/运行M1或任何硬件。

输入为显式GCC和本仓库六项源码/契约，输出只在忽略的build目录。
记录每一步实际命令/返回码/原始输出、输入哈希、完整HEAD与工作区状态；保留失败证据。
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys


def require(condition, message):
    """检查验收条件；不用可能被Python -O删除的assert，失败以明确原因停止。"""
    if not condition:
        raise RuntimeError(message)


def main():
    """编译、检查依赖/符号、执行独立oracle并记录证据；错误仍写日志后非零退出。"""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=root / "build/host_feedback")
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    require(output.is_relative_to(root / "build"), "output must stay inside this repository's build directory")
    output.mkdir(parents=True, exist_ok=True)
    gcc = arguments.gcc.resolve()
    environment = os.environ.copy()
    environment["PATH"] = str(gcc.parent) + os.pathsep + environment.get("PATH", "")
    input_names = ["control_core/feedback.c", "control_core/feedback.h",
                   "tests/control_core/test_feedback.c", "tools/test_feedback.py",
                   "docs/m3a_feedback_contract.md", "README.md"]
    inputs = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in input_names}
    evidence = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "input_sha256": inputs,
                "test_data": "deterministic integer trajectories; no random seed",
                "python_version": sys.version, "steps": [], "result": "FAIL"}
    evidence_file = output / "test_evidence.json"
    if evidence_file.exists():
        previous = json.loads(evidence_file.read_text(encoding="utf-8"))
        if previous.get("result") != "PASS":
            destination = output / ("failed_attempt_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
            destination.write_bytes(evidence_file.read_bytes())

    def run(command, allow_failure=False):
        """运行参数数组；每一步落盘，即使编译失败也不丢原始输出；仅特殊负向编译允许非零。"""
        command = [item.as_posix() if isinstance(item, Path) else str(item) for item in command]
        completed = subprocess.run(command, cwd=root, env=environment, text=True, encoding="utf-8",
                                   errors="replace", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        evidence["steps"].append({"command": command, "exit_code": completed.returncode,
                                  "expected_nonzero": allow_failure, "output": completed.stdout})
        evidence_file.write_bytes((json.dumps(evidence, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
        if completed.stdout.strip():
            print("\n".join(line for line in completed.stdout.splitlines() if not line.startswith("SYNTHETIC_CSV,")))
        if not allow_failure:
            require(completed.returncode == 0, "command failed; see test_evidence.json")
        return completed

    try:
        evidence["head"] = run(["git", "rev-parse", "HEAD"]).stdout.strip()
        evidence["worktree_before"] = run(["git", "status", "--porcelain=v1"]).stdout
        evidence["firmware_tree_sha"] = run(["git", "rev-parse", "HEAD:firmware/stm32f103"]).stdout.strip()
        evidence["compiler_version"] = run([gcc, "--version"]).stdout
        for name in input_names:
            content = (root / name).read_bytes().decode("utf-8", errors="strict")
            require(content.endswith("\n") and "\r" not in content, "authored source must be UTF-8/LF: " + name)
            require(all(line == line.rstrip() for line in content.splitlines()), "trailing whitespace: " + name)
        core = root / "control_core/feedback.c"
        header = root / "control_core/feedback.h"
        test = root / "tests/control_core/test_feedback.c"
        core_object = output / "feedback.o"
        dependency_file = output / "feedback.d"
        common = ["-std=c11", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-pedantic", "-fno-fast-math"]
        run([gcc, *common, "-I", header.parent, "-MMD", "-MF", dependency_file,
             "-c", core, "-o", core_object])
        dependencies = dependency_file.read_text(encoding="utf-8").replace("\\\n", " ").split(": ", 1)[1]
        dependency_paths = {(root / token).resolve() for token in shlex.split(dependencies)}
        require(dependency_paths == {core, header}, "core acquired nonstandard/HAL/board source dependencies")
        evidence["core_project_dependencies"] = sorted(path.relative_to(root).as_posix() for path in dependency_paths)
        symbols = run([gcc.parent / "nm.exe", "--defined-only", core_object]).stdout
        sections = run([gcc.parent / "objdump.exe", "-h", core_object]).stdout
        # PE/COFF为零长度.data/.bss也输出section符号，不能把它们当作真实全局变量。
        # 先检查可写section实际大小，再在nm中拒绝所有具名可写数据，不能只看字母b/d。
        empty_sections = set()
        for line in sections.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[0].isdigit() and parts[1].startswith((".data", ".bss", ".sdata", ".sbss")):
                require(int(parts[2], 16) == 0, "core contains writable data-section bytes")
                empty_sections.add(parts[1])
        for line in symbols.splitlines():
            parts = line.split()
            require(len(parts) < 3 or parts[-2] not in "bBdDgGsScC" or parts[-1] in empty_sections,
                    "core defines writable global/static storage")
        print("PASS core isolation: only core header/source project dependencies; no writable static/global symbols")
        executable = output / "test_feedback.exe"
        run([gcc, *common, "-I", header.parent, core_object, test, "-lm", "-o", executable])
        tested = run([executable])
        rows = [line.removeprefix("SYNTHETIC_CSV,") for line in tested.stdout.splitlines()
                if line.startswith("SYNTHETIC_CSV,")]
        require(len(rows) == 387, "synthetic evidence row count differs")
        (output / "synthetic.csv").write_text(
            "trajectory,index,unwrapped_raw_count,midpoint_us,direct_rad_s,window_rad_s,truth_rad_s,direct_valid,window_valid\n"
            + "\n".join(rows) + "\n", encoding="utf-8")
        metrics = []
        for line in tested.stdout.splitlines():
            if line.startswith("METRIC,"):
                fields = line.split(",")
                metrics.append({"trajectory": int(fields[1]), "direct_rms_rad_s": float(fields[2]),
                                "window_rms_rad_s": float(fields[3]), "evaluated_samples": int(fields[4]),
                                "direct_plateau_index": int(fields[5]), "window_plateau_index": int(fields[6])})
        require(len(metrics) == 3, "synthetic metrics missing")
        (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
        evidence["synthetic_artifact_sha256"] = {
            name: hashlib.sha256((output / name).read_bytes()).hexdigest()
            for name in ["synthetic.csv", "metrics.json"]}
        # 负向编译不是运行期断言替代：明确验证调用者不能用fast-math删掉NaN/Inf检查。
        negative = run([gcc, "-std=c11", "-O2", "-ffast-math", "-I", header.parent,
                        "-c", core, "-o", output / "must_not_compile.o"], allow_failure=True)
        require(negative.returncode != 0 and "feedback requires finite-input checks" in negative.stdout,
                "fast-math contract guard did not reject the unsafe compilation")
        print("PASS build contract: fast-math intentionally rejected")
        evidence["head_after"] = run(["git", "rev-parse", "HEAD"]).stdout.strip()
        evidence["worktree_after"] = run(["git", "status", "--porcelain=v1"]).stdout
        require(evidence["head_after"] == evidence["head"], "HEAD changed during test")
        require(all(hashlib.sha256((root / name).read_bytes()).hexdigest() == digest
                    for name, digest in inputs.items()), "input changed during test")
        require(evidence["worktree_after"] == evidence["worktree_before"], "worktree changed during test")
        evidence["executable_sha256"] = hashlib.sha256(executable.read_bytes()).hexdigest()
        evidence["result"] = "PASS"
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        evidence["error"] = str(error)
        print("FAIL:", error)
    finally:
        evidence["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        evidence_file.write_bytes((json.dumps(evidence, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    if evidence["result"] != "PASS":
        raise SystemExit(1)
    print("PASS M3a evidence saved; synthetic host math only, no firmware/hardware execution")


if __name__ == "__main__":
    main()
