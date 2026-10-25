"""M2b电流PI主机验收：独立编译真实core和oracle，不链接/运行M1或任何硬件。

输入为显式GCC和本仓库九项源码/契约，输出只在忽略的build目录。
记录每一步实际命令/返回码/原始输出、输入哈希、完整HEAD与工作区状态；保留失败证据。
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
from urllib.parse import unquote


def require(condition, message):
    """检查验收条件；不用可能被Python -O删除的assert，失败以明确原因停止。"""
    if not condition:
        raise RuntimeError(message)


def main():
    """编译、检查依赖/符号、执行独立oracle并记录证据；错误仍写日志后非零退出。"""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=root / "build/host_current_control")
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    require(output.is_relative_to(root / "build"), "output must stay inside this repository's build directory")
    output.mkdir(parents=True, exist_ok=True)
    gcc = arguments.gcc.resolve()
    environment = os.environ.copy()
    environment["PATH"] = str(gcc.parent) + os.pathsep + environment.get("PATH", "")
    input_names = ["control_core/current_control.c", "control_core/current_control.h",
                   "control_core/foc_math.c", "control_core/foc_math.h",
                   "tests/control_core/test_current_control.c", "tools/test_current_control.py",
                   "docs/m2b_current_control_contract.md", "docs/m2a_foc_contract.md", "README.md"]
    inputs = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in input_names}
    evidence = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "input_sha256": inputs,
                "test_data": "deterministic grids; no random seed", "steps": [], "result": "FAIL"}
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
            print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n")
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
            if name.endswith(".md"):
                require(content.count("```") % 2 == 0, "unclosed code fence: " + name)
                for link in re.findall(r"\]\(([^)]+)\)", content):
                    if "://" in link or link.startswith("#"):
                        continue
                    target = ((root / name).parent / unquote(link.split("#")[0])).resolve()
                    require(target.is_relative_to(root) and target.exists(), "bad relative link: " + name + " -> " + link)
        core = root / "control_core/current_control.c"
        header = root / "control_core/current_control.h"
        test = root / "tests/control_core/test_current_control.c"
        core_object = output / "current_control.o"
        dependency_file = output / "current_control.d"
        common = ["-std=c11", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-pedantic", "-fno-fast-math"]
        run([gcc, *common, "-I", header.parent, "-MMD", "-MF", dependency_file,
             "-c", core, "-o", core_object])
        dependencies = dependency_file.read_text(encoding="utf-8").replace("\\\n", " ").split(": ", 1)[1]
        dependency_paths = {(root / token).resolve() for token in shlex.split(dependencies)}
        require(dependency_paths == {core, header, root / "control_core/foc_math.h"}, "core acquired nonstandard/HAL/board source dependencies")
        evidence["core_project_dependencies"] = sorted(path.relative_to(root).as_posix() for path in dependency_paths)
        dependencies_by_module = {"current_control": evidence["core_project_dependencies"]}
        foc_object = output / "foc_math.o"
        foc_source = root / "control_core/foc_math.c"
        foc_header = root / "control_core/foc_math.h"
        foc_dependencies = output / "foc_math.d"
        run([gcc, *common, "-I", header.parent, "-MMD", "-MF", foc_dependencies,
             "-c", foc_source, "-o", foc_object])
        foc_paths = {(root / token).resolve() for token in shlex.split(
            foc_dependencies.read_text(encoding="utf-8").replace("\\\n", " ").split(": ", 1)[1])}
        require(foc_paths == {foc_source, foc_header}, "M2a acquired unexpected project dependencies")
        dependencies_by_module["foc_math"] = sorted(path.relative_to(root).as_posix() for path in foc_paths)
        evidence["dependencies_by_module"] = dependencies_by_module
        undefined = run([gcc.parent / "nm.exe", "--undefined-only", core_object]).stdout
        require(all(name in undefined for name in ("foc_transform_current", "foc_modulate_voltage")),
                "current controller does not actually call both canonical M2a functions")
        for object_file in (core_object, foc_object):
            symbols = run([gcc.parent / "nm.exe", "--defined-only", object_file]).stdout
            sections = run([gcc.parent / "objdump.exe", "-h", object_file]).stdout
            # PE/COFF允许零长度可写section符号，禁止实际可写字节/具名静态变量。
            empty_sections = set()
            for line in sections.splitlines():
                parts = line.split()
                if len(parts) >= 3 and parts[0].isdigit() and parts[1].startswith((".data", ".bss", ".sdata", ".sbss")):
                    require(int(parts[2],16) == 0, "core contains writable data-section bytes")
                    empty_sections.add(parts[1])
            for line in symbols.splitlines():
                parts = line.split()
                require(len(parts)<3 or parts[-2] not in "bBdDgGsScC" or parts[-1] in empty_sections,
                        "core defines writable global/static storage")
        print("PASS isolation: canonical M2a calls, exact project dependency closure, no writable static/global data")
        executable = output / "test_current_control.exe"
        run([gcc, *common, "-I", header.parent, core_object, foc_object, test, "-lm", "-o", executable])
        run([executable])
        # 负向编译不是运行期断言替代：明确验证调用者不能用fast-math删掉NaN/Inf检查。
        negative = run([gcc, "-std=c11", "-O2", "-ffast-math", "-I", header.parent,
                        "-c", core, "-o", output / "must_not_compile.o"], allow_failure=True)
        require(negative.returncode != 0 and "current_control requires finite-input checks" in negative.stdout,
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
    print("PASS M2b evidence saved; synthetic host control math only, no firmware/hardware execution")


if __name__ == "__main__":
    main()
