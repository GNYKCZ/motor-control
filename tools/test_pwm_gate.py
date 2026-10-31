"""生产board_pwm/runtime/command的主机寄存器交错测试；没有串口/下载/硬件操作。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from sensor_capture import parse_line
from commutation_report import analyze, combine


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    fw = root / "firmware/stm32f103"
    output = root / "build/host_pwm_gate"
    output.mkdir(parents=True, exist_ok=True)
    gcc = args.gcc.resolve()
    environment = os.environ.copy()
    environment["PATH"] = str(gcc.parent) + os.pathsep + environment.get("PATH", "")
    model = root / "tests/m1/pwm_model"
    c_sources = [fw / p for p in ("app/motor_runtime.c", "app/command.c", "board/sample_frame.c")]
    c_sources += [root / "control_core" / (n + ".c") for n in ("feedback", "sensor_input")]
    inputs = c_sources + [fw / "board/board_pwm.c", root / "tests/m1/test_pwm_gate.cpp", Path(__file__).resolve()]
    inputs += list(model.glob("*.h")) + list((fw / "app").glob("*.h")) + list((fw / "board").glob("*.h"))
    inputs += [root / "control_core" / (n + ".h") for n in ("feedback", "sensor_input")]
    inputs += [root / "tools" / n for n in ("sensor_capture.py","commutation_report.py")]
    hashes = {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    evidence = {"head": head, "input_sha256": hashes, "result": "FAIL", "steps": [], "hardware": "NOT RUN"}

    def run(cmd):
        command = list(map(str, cmd))
        result = subprocess.run(command, cwd=root, env=environment, capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=60)
        text = result.stdout + result.stderr
        evidence["steps"].append({"command": command, "returncode": result.returncode, "output": text})
        if text.strip():
            print(text, end="" if text.endswith("\n") else "\n")
        if result.returncode:
            raise RuntimeError("gate test command failed")
        return text

    try:
        run([gcc, "--version"])
        includes = [flag for directory in (model, fw / "board", fw / "app", root / "control_core") for flag in ("-I", directory)]
        flags = ["-O2", "-g", "-Wall", "-Wextra", "-Werror", "-pedantic", *includes]
        scan_reports=[]
        for tag in ("production", "scan_forward", "scan_reverse", "align_positive", "align_negative", "synthetic"):
            synthetic = tag=="synthetic"
            define = (["-DMOTOR_GATE_TEST_PROFILES"] if synthetic else
                      ["-DMOTOR_LOCAL_PULSE_BASELINE"] if tag=="production" else
                      ["-DBOARD_SCAN_DIRECTION=-1"] if tag=="scan_reverse" else
                      ["-DBOARD_ALIGNMENT_MODE="+str(1 if tag=="align_positive" else -1)] if tag.startswith("align_") else [])
            objects = []
            for source in c_sources:
                obj = output / (tag + "_" + source.stem + ".o")
                run([gcc, "-std=c11", *flags, *define, "-c", source, "-o", obj])
                objects.append(obj)
            cpp = [gcc.parent / "g++.exe", "-std=c++11", *flags, *define]
            board = output / (tag + "_board_pwm.o")
            run([*cpp, "-include", model / "linkage.h", "-x", "c++", "-c", fw / "board/board_pwm.c", "-o", board])
            exe = output / (tag + ".exe")
            run([*cpp, root / "tests/m1/test_pwm_gate.cpp", board, *objects, "-lm", "-o", exe])
            target_text=run([exe])
            if tag.startswith("scan_"):
                frame=next((line+"\r\n").encode() for line in target_text.splitlines() if line.startswith("P1A1,"))
                scan=parse_line(frame)
                if scan["done"]!=1 or scan["mask"]!=127 or scan["direction"]!=(-1 if tag.endswith("reverse") else 1):
                    raise RuntimeError("production scan snapshot does not match completed model")
                scan_reports.append(analyze(scan))
            if tag.startswith("align_"):
                frame=next((line+"\r\n").encode() for line in target_text.splitlines() if line.startswith("P1A2,"))
                record=parse_line(frame)
                if not record["done"] or record["mask"]!=3 or record["direction"]!=(1 if tag.endswith("positive") else -1):
                    raise RuntimeError("alignment mode/frame mismatch")
                evidence[tag]=record
            if tag=="scan_forward":
                old_window=output/"scan_window_old.c"
                old_window.write_bytes(subprocess.check_output(["git","show",
                    "2371c5b85d5fb538c63c091ffbcf5e78cec513a9:firmware/stm32f103/app/motor_runtime.c"],cwd=root))
                old_window_obj=output/"scan_window_old.o"
                run([gcc,"-std=c11",*flags,"-c",old_window,"-o",old_window_obj])
                old_window_exe=output/"scan_window_old.exe"
                run([*cpp,root/"tests/m1/test_pwm_gate.cpp",board,old_window_obj,*objects[1:],"-lm","-o",old_window_exe])
                boundary=subprocess.run([str(old_window_exe)],cwd=root,env=environment,capture_output=True,text=True,timeout=30)
                if boundary.returncode==0 or "scan.point[0].count==0u" not in boundary.stderr:
                    raise RuntimeError("scan window RV-001 did not reject ADC-completion-based admission")
                evidence["scan_window_RV001"]={"returncode":boundary.returncode,"output":boundary.stdout+boundary.stderr,
                    "runtime_sha256":hashlib.sha256(old_window.read_bytes()).hexdigest()}
            if tag=="production":
                # 带流RV-001：旧实现只看ADC完成时间，同一跨开门边界夹具必须失败。
                old_runtime = output / "current_boundary_old.c"
                old_source = subprocess.check_output(["git", "show",
                    "7c41d93459dc0ac42a9fdbffeac51591fb794d60:firmware/stm32f103/app/motor_runtime.c"], cwd=root)
                # 仅对齐本版V矢量；不改旧采集判据，确保仍命中跨开门边界反例。
                old_runtime.write_bytes(old_source.replace(b"{{60u,0u,0u,434u}", b"{{0u,60u,0u,434u}"))
                old_runtime_obj = output / "current_boundary_old.o"
                run([gcc, "-std=c11", *flags, "-DMOTOR_LOCAL_PULSE_BASELINE", "-c", old_runtime, "-o", old_runtime_obj])
                old_capture_exe = output / "current_boundary_old.exe"
                run([*cpp, root / "tests/m1/test_pwm_gate.cpp", board, old_runtime_obj, *objects[1:], "-lm", "-o", old_capture_exe])
                bad_capture = subprocess.run([str(old_capture_exe)], cwd=root, env=environment,
                                             capture_output=True, text=True, timeout=30)
                if bad_capture.returncode == 0 or "current.samples.accepted_count==0u" not in bad_capture.stderr:
                    raise RuntimeError("current boundary regression did not reject completion-only capture")
                evidence["current_boundary_RV001"] = {"returncode": bad_capture.returncode,
                    "output": bad_capture.stdout + bad_capture.stderr,
                    "runtime_sha256": hashlib.sha256(old_runtime.read_bytes()).hexdigest()}
                # 原真实board实现+新推进时钟回归：旧比较点不能通过同一压力场景。
                baseline = output / "window_old_board.cpp"
                baseline.write_bytes(subprocess.check_output(["git", "show",
                    "35fc2209b24edb0abe11e4df55ad4e9e1ffe3cba:firmware/stm32f103/board/board_pwm.c"], cwd=root))
                old_board = output / "window_old_board.o"
                run([*cpp, "-include", model / "linkage.h", "-c", baseline, "-o", old_board])
                old_exe = output / "window_old.exe"
                run([*cpp, root / "tests/m1/test_pwm_gate.cpp", old_board, *objects, "-lm", "-o", old_exe])
                old_result = subprocess.run([str(old_exe)], cwd=root, env=environment, capture_output=True,
                                            text=True, timeout=30)
                if old_result.returncode == 0 or "state==MOTOR_RUNTIME_ACTIVE && enables==1u" not in old_result.stderr:
                    raise RuntimeError("advancing-clock regression did not reject old compare scheduling")
                evidence["window_old_negative"] = {"returncode": old_result.returncode,
                    "output": old_result.stdout + old_result.stderr,
                    "board_sha256": hashlib.sha256(baseline.read_bytes()).hexdigest()}
        geometry=combine(*scan_reports)
        if geometry["pole_pairs"]!=7 or geometry["encoder_direction"]!=1:
            raise RuntimeError("production seven-point geometry mapping failed")
        evidence["scan_geometry"]=geometry
        # RV-001反例：在Break处理后故意留下MOE，旧测试补救STOP会洗掉，新测试必须当场失败。
        # 只改变测试夹具的模拟寄存器，不改生产board/runtime/command，也不宣称真实固件有此缺陷。
        old = subprocess.check_output(["git", "show", "77e27db1c4f9aeb5109a610962707943de9fe94b:tests/m1/test_pwm_gate.cpp"], cwd=root).decode("utf-8")
        # 旧夹具保留其补救STOP行为，仅把原始失联场景对齐本次3ms契约；用ARM避免1ms合成脉冲先到期。
        old = old.replace("reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();\n    timer_tick(300u);CHECK(status().state==MOTOR_RUNTIME_ACTIVE);",
                          "reset();CHECK(motor_runtime_arm(1u));\n    timer_tick(3000u);CHECK(status().state==MOTOR_RUNTIME_ARMED);")
        old = old.replace("reset(0xffffd800u);CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();",
                          "reset(0xffffd800u);CHECK(motor_runtime_arm(1u));")
        old = old.replace("timer_tick(301u);", "timer_tick(3001u);").replace("now_us+=301u;", "now_us+=3001u;")
        current = (root / "tests/m1/test_pwm_gate.cpp").read_text(encoding="utf-8")
        anchor = "if (kind==2u) { TIM1_BRK_IRQHandler(); }"
        oracle = {}
        for label, source in (("old", old), ("fixed", current)):
            if source.count(anchor) != 1:
                raise RuntimeError("RV-001 negative fixture anchor mismatch")
            fixture = output / ("rv001_" + label + ".cpp")
            fixture.write_text(source.replace(anchor, anchor + " if (kind==2u) { pwm_tim1.BDTR.value |= TIM_BDTR_MOE; }"), encoding="utf-8")
            exe = output / ("rv001_" + label + ".exe")
            run([*cpp, fixture, board, *objects, "-lm", "-o", exe])
            result = subprocess.run([str(exe)], cwd=root, env=environment, capture_output=True, text=True, timeout=30)
            oracle[label] = {"returncode": result.returncode, "output": result.stdout + result.stderr,
                             "fixture_sha256": hashlib.sha256(fixture.read_bytes()).hexdigest()}
        if oracle["old"]["returncode"] != 0 or oracle["fixed"]["returncode"] == 0 or "BDTR.value & TIM_BDTR_MOE" not in oracle["fixed"]["output"]:
            raise RuntimeError("RV-001 negative control did not distinguish old and fixed assertions")
        evidence["RV-001"] = oracle
        negative = subprocess.run([str(gcc), "-std=c11", *map(str, flags), "-DMOTOR_GATE_TEST_PROFILES",
                                   "-D__arm__", "-fsyntax-only", str(fw / "app/motor_runtime.c")],
                                  cwd=root, env=environment, capture_output=True, text=True, timeout=30)
        if negative.returncode == 0 or "Synthetic output profiles are forbidden" not in negative.stderr:
            raise RuntimeError("target synthetic-profile compile guard failed")
        legacy_negative=subprocess.run([str(gcc),"-std=c11",*map(str,flags),"-DMOTOR_LOCAL_PULSE_BASELINE",
                                       "-D__arm__","-fsyntax-only",str(fw/"app/motor_runtime.c")],
                                      cwd=root,env=environment,capture_output=True,text=True,timeout=30)
        if legacy_negative.returncode==0 or "Synthetic output profiles are forbidden" not in legacy_negative.stderr:
            raise RuntimeError("target legacy-profile compile guard failed")
        evidence["target_profile_guard"] = "PASS: target architecture rejects synthetic profiles at compile time"
        evidence["result"] = "PASS"
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        evidence["error"] = str(error)
    finally:
        if any(hashlib.sha256((root / p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
            evidence.update(result="FAIL", error="inputs changed during tests")
        path = output / "test_evidence.json"
        if path.exists():
            old = json.loads(path.read_text(encoding="utf-8"))
            if old.get("result") != "PASS":
                number = len(list(output.glob("failed_*.json"))) + 1
                (output / f"failed_{number}.json").write_bytes(path.read_bytes())
        path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    raise SystemExit(0 if evidence["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
