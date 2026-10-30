"""P1a-1真实命令/运行状态/UART寄存器适配集成；仅主机软件，不打开串口。"""
import argparse
import binascii
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
from command_link import make_command
from sensor_capture import parse_line, save_chunks


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=root / "build/host_command")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    gcc = args.gcc.resolve()
    fw = root / "firmware/stm32f103"
    sources = [fw / n for n in ("board/sample_frame.c", "board/board_uart.c", "app/motor_runtime.c",
                                "app/command.c", "app/telemetry.c")]
    sources += [root / "tests/m1/runtime_gate_stub.c"]
    sources += [root / n for n in ("control_core/sensor_input.c", "control_core/feedback.c", "tests/m1/test_command.c")]
    inputs = sources + list((fw / "app").glob("*.h")) + list((fw / "board").glob("*.h"))
    inputs += [root / n for n in ("tests/m1/uart_stub/usart.h", "tools/test_command.py", "tools/command_link.py", "tools/sensor_capture.py", "control_core/feedback.h", "control_core/sensor_input.h")]
    inputs += [root / n for n in ("tests/m1/test_uart_tc_race.cpp", "tests/m1/uart_model/usart.h", "tests/m1/uart_model/linkage.h")]
    hashes = {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True)
    evidence = {"head": head, "worktree_before": status, "input_sha256": hashes, "result": "FAIL", "steps": []}
    environment = os.environ.copy()
    environment["PATH"] = str(gcc.parent) + os.pathsep + environment.get("PATH", "")

    def run(cmd):
        result = subprocess.run(list(map(str, cmd)), cwd=root, env=environment, text=True, encoding="utf-8",
                                errors="replace", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        evidence["steps"].append({"command": list(map(str, cmd)), "returncode": result.returncode, "output": result.stdout})
        if result.stdout.strip():
            print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
        if result.returncode:
            raise RuntimeError("command test failed")
        return result.stdout

    try:
        # 发布的黄金向量与库自检；生成测试向量不使用生产make_command或C CRC。
        if binascii.crc_hqx(b"123456789", 0xFFFF) != 0x29B1:
            raise RuntimeError("CRC oracle self-check")
        if make_command(1, "status") != b"P1C1,00000001,01,A1E6\r\n" or make_command(2, "stop") != b"P1C1,00000002,02,1A67\r\n":
            raise RuntimeError("host tool golden command mismatch")
        sequences = [0, 1, 2, 3, 0xFFFFFFFE, 0xFFFFFFFF, 0x80000002]
        rows = []
        for seq in sequences:
            for code in [1, 2, 3, 4, 5, 6, 7, 0x55]:
                prefix = f"P1C1,{seq:08X},{code:02X},".encode("ascii")
                frame = prefix + f"{binascii.crc_hqx(prefix, 0xFFFF):04X}\r\n".encode("ascii")
                rows.append(f"{{{seq}u,{code}u,{json.dumps(frame.decode('ascii'))}}}")
        header = "#include <stdint.h>\nstatic const struct {uint32_t seq; unsigned code; const char *frame;} vectors[]={\n" + ",\n".join(rows) + "\n};\n"
        (output / "command_vectors.h").write_text(header, encoding="ascii", newline="\n")
        run([gcc, "--version"])
        command = [gcc, "-std=c11", "-D__USE_MINGW_ANSI_STDIO=1", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-pedantic"]
        for directory in [root / "tests/m1/uart_stub", output, fw / "board", fw / "app", root / "control_core"]:
            command += ["-I", directory]
        run(command + sources + ["-lm", "-o", output / "test_command.exe"])
        text = run([output / "test_command.exe"])
        # RV-001：同一生产ISR用C++寄存器语义模型编译，注入SR读写间硬件到达；不改生产源码。
        model = root / "tests/m1/uart_model"
        model_objects = []
        for source in sources:
            if source.name in {"board_uart.c", "test_command.c"}:
                continue
            obj = output / ("race_" + source.stem + ".o")
            run(command + ["-c", source, "-o", obj])
            model_objects.append(obj)
        cpp = [gcc.parent / "g++.exe", "-std=c++11", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-pedantic"]
        run([gcc.parent / "g++.exe", "--version"])
        for directory in [model, fw / "board", fw / "app", root / "control_core"]:
            cpp += ["-I", directory]
        irq_object = output / "race_board_uart.o"
        run(cpp + ["-include", model / "linkage.h", "-x", "c++", "-c", fw / "board/board_uart.c", "-o", irq_object])
        run(cpp + [root / "tests/m1/test_uart_tc_race.cpp", irq_object, *model_objects, "-lm", "-o", output / "test_uart_tc_race.exe"])
        run([output / "test_uart_tc_race.exe"])
        stream = b"".join((line + "\r\n").encode("ascii") for line in text.splitlines() if line.startswith(("P1R2,", "P1Z1,", "P1V1,", "M3B3,")))
        records = [parse_line(line + b"\n") for line in stream.split(b"\n") if line]
        replies = [r for r in records if r["format"] == "P1R2"]
        if not replies or any(replies[0][key] != value for key, value in {"q": 1, "c": 1, "r": 0, "s": 1, "f": 0, "n": 25, "t": 2500}.items()):
            raise RuntimeError("real runtime status snapshot mismatch")
        if not any(r["c"] == 2 and r["r"] == 128 for r in replies) or not any(r["c"] == 3 and r["r"] == 4 for r in replies):
            raise RuntimeError("STOP/denied reply evidence missing")
        capture_dir = output / ("mixed_capture_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
        info = save_chunks([stream[:19], stream[19:61], stream[61:]], capture_dir, {"source_kind": "synthetic", "head": head})
        if info["invalid_lines"] or info["valid_lines"] != len(records):
            raise RuntimeError("mixed capture failed")
        reply = next(line + b"\n" for line in stream.split(b"\n") if line.startswith(b"P1R2,"))
        # 所有位翻转都必须因布局/CRC拒绝，包括CRC字段和终止符；保留原行作为独立参照。
        rejected = 0
        for pos in range(len(reply)):
            for bit in range(8):
                bad = bytearray(reply)
                bad[pos] ^= 1 << bit
                try:
                    parse_line(bytes(bad))
                except (ValueError, UnicodeError):
                    rejected += 1
                else:
                    raise RuntimeError("single-bit reply corruption accepted")
        golden = b"P1R2,00000001,01,00,01,00000000,00000019,000009C4,00000000,00000000,00000000,00000000,00,6B79\r\n"
        if replies[0] != parse_line(golden) or len(reply) != 95 or rejected != 760:
            raise RuntimeError("P1R2 independent golden/length/single-bit oracle mismatch")
        zero_golden = b"P1Z1,00000001,05,00,01,00000000,00000001,00000064,0001,000007EB,000007EC,07EB,07EB,07EC,07EC,08,000C4B20,00000064,00000064,00020001,1DDD\r\n"
        zero_actual = next(line + b"\n" for line in stream.split(b"\n") if line.startswith(b"P1Z1,"))
        if zero_actual != zero_golden or parse_line(zero_actual)["cn"] != 1:
            raise RuntimeError("P1Z1 independent 138B golden mismatch")
        for i in range(138):
            for bit in range(8):
                corrupted = bytearray(zero_actual); corrupted[i] ^= 1 << bit
                try:
                    parse_line(bytes(corrupted))
                except ValueError:
                    pass
                else:
                    raise RuntimeError("P1Z1 accepted one-bit corruption")
        current_prefix = b"P1V1,00000001,07,00,01,00000000,00000001,00000064,0001,000007EB,000007EC,07EB,07EB,07EC,07EC,08,000C4B20,00000064,00000064,00020001,0000,00000000,00000000,0000,0000,0000,0000,00000000,00000000,0000,0000,00,00000000,"
        current_golden = current_prefix + f"{binascii.crc_hqx(current_prefix,0xFFFF):04X}\r\n".encode()
        current_actual = next(line+b"\n" for line in stream.split(b"\n") if line.startswith(b"P1V1,"))
        if current_actual != current_golden or parse_line(current_actual)["ic"] != 0:
            raise RuntimeError("P1V1 independent 221B empty capture mismatch")
        for i in range(221):
            for bit in range(8):
                corrupted = bytearray(current_actual); corrupted[i] ^= 1 << bit
                try:
                    parse_line(bytes(corrupted))
                except ValueError:
                    pass
                else:
                    raise RuntimeError("P1V1 accepted one-bit corruption")
        zero_prefix = b"P1R2,00000002,01,00,01,00000000,00000001,00000000,00000001,FFFFFF00,00000000,00000000,C4,"
        zero_frame = zero_prefix + f"{binascii.crc_hqx(zero_prefix,0xFFFF):04X}\r\n".encode()
        zero = parse_line(zero_frame)
        if zero["start"] != 0 or zero["why"] != 0xC4:
            raise RuntimeError("wrapped zero timestamp lost validity")
        evidence.update(result="PASS", mixed_records=len(records), rejected_single_bits=rejected,
                        request_bytes=23, reply_bytes=95, oracle="explicit golden strings; CRC-CCITT check 123456789=29B1; Python crc_hqx")
        evidence["RV-001"] = "PASS: actual ISR with W0C/RMW-arrival model, 23 STOP-byte overlaps, no SR write, TC retention/next-TX clear"
        print("PASS P1a-1 real runtime -> UART -> command -> reply -> capture, errors/priority/backpressure; no hardware")
    except (RuntimeError, OSError, ValueError, subprocess.TimeoutExpired) as error:
        evidence["error"] = str(error)
        print("FAIL", error)
    finally:
        evidence["head_after"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        evidence["worktree_after"] = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True)
        if evidence["head_after"] != head or evidence["worktree_after"] != status or any(hashlib.sha256((root / name).read_bytes()).hexdigest() != value for name, value in hashes.items()):
            evidence.update(result="FAIL", error="inputs changed during test")
        path = output / "test_evidence.json"
        if path.exists() and json.loads(path.read_text(encoding="utf-8")).get("result") != "PASS":
            (output / ("failed_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")).write_bytes(path.read_bytes())
        path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    raise SystemExit(0 if evidence["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
