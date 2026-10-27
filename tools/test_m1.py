"""用主机 GCC 运行实际帧/运行管理/导出模块；仅替换 board 外设，不接触硬件。"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
import re
import subprocess
from sensor_capture import parse_line
from pathlib import Path


def main():
    """编译真实模块与独立 oracle 测试，记录命令、SHA 和当前工作区状态。"""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", type=Path, required=True)
    arguments = parser.parse_args()
    output = root / "build/host_m1"
    output.mkdir(parents=True, exist_ok=True)
    previous_file = output / "test_evidence.json"
    if previous_file.exists() and json.loads(previous_file.read_text(encoding="utf-8")).get("exit_code") != 0:
        (output / ("failed_attempt_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")).write_bytes(previous_file.read_bytes())
    firmware = root / "firmware/stm32f103"
    source_names = ["board/sample_frame.c", "app/motor_runtime.c", "app/telemetry.c"]
    sources = [firmware / name for name in source_names] + [root / "control_core/sensor_input.c", root / "control_core/feedback.c", root / "tests/m1/test_sampling.c"]
    # MinGW默认旧MSVCRT格式器不支持C99 %lld，使用其标准C格式器以匹配目标微库。
    command = [str(arguments.gcc.resolve()), "-std=c11", "-D__USE_MINGW_ANSI_STDIO=1", "-O2", "-g", "-Wall", "-Wextra", "-Werror",
               "-pedantic", "-I", str(firmware / "board"), "-I", str(firmware / "app"),
               "-I", str(root / "control_core"), *map(str, sources), "-lm", "-o", str(output / "test_sampling.exe")]
    version = subprocess.check_output([str(arguments.gcc), "--version"], text=True)
    environment = os.environ.copy()
    environment["PATH"] = str(arguments.gcc.resolve().parent) + os.pathsep + environment.get("PATH", "")
    result = subprocess.run(command, text=True, env=environment, timeout=60,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    transcript = result.stdout
    if result.returncode == 0:
        result = subprocess.run([str(output / "test_sampling.exe")], text=True, env=environment, timeout=60,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        transcript += result.stdout
        if result.returncode == 0:
            # 同一真实runtime/telemetry文本直接交真实capture；三行含首帧、直接速度、满窗。
            parsed = [parse_line((line + "\n").encode("ascii")) for line in result.stdout.splitlines()
                      if line.startswith("M3B2,")]
            if len(parsed) != 3 or [p["fc"] for p in parsed] != [0, 10, 80] or not parsed[-1]["fw"]:
                raise RuntimeError("runtime -> telemetry -> capture integration mismatch")
            transcript += "PASS M3b real runtime -> telemetry -> capture: first/direct/window records\n"
    print(transcript, end="")
    # 独立上界：将实际C格式串的每个转换替换为其整数类型最长十进制表示。
    # 包含符号与CRLF，预留结尾NUL；不靠本次正常小数值行长度推断最坏容量。
    source = (firmware / "app/telemetry.c").read_text(encoding="utf-8")
    format_parts = re.findall(r'return APPEND\("([^"\n]*)"', source)
    if len(format_parts) != 13 or any(len(re.findall(r"%llu|%lld|%lu|%ld|%u", part)) > 7 for part in format_parts):
        raise RuntimeError("telemetry format parts exceed fixed conversion budget")
    format_text = "".join(json.loads('"' + part + '"') for part in format_parts)
    widths = {"%llu": 20, "%lld": 20, "%lu": 10, "%ld": 11, "%u": 10}
    longest = re.sub(r"%llu|%lld|%lu|%ld|%u", lambda match: "9" * widths[match[0]], format_text)
    capacity = int(re.search(r"#define BOARD_SERIAL_BUFFER_BYTES (\d+)u",
                            (firmware / "board/board_config.h").read_text(encoding="utf-8"))[1])
    if "%" in longest or len(longest) + 1 > capacity:
        raise RuntimeError("telemetry worst-case integer length exceeds static buffer")
    transcript += f"PASS M3b format bound: {len(longest)} bytes + NUL <= {capacity}\n"
    print(transcript.splitlines()[-1])
    inputs = sources + list((firmware / "board").glob("*.h")) + list((firmware / "app").glob("*.h"))
    inputs += [root / "control_core/sensor_input.h", root / "control_core/feedback.h"]
    inputs += [root / "tools/sensor_capture.py"]
    evidence = {"head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                "worktree_status": subprocess.check_output(["git", "status", "--porcelain=v1"], cwd=root, text=True),
                "compiler_version": version, "command": command, "exit_code": result.returncode,
                "output": transcript, "source_sha256": {p.relative_to(root).as_posix():
                    hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
                "test_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (output / "test_evidence.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
                                              encoding="utf-8")
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
