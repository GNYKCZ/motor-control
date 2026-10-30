"""构建唯一 M1 固件。只编译/链接，不烧录、不打开调试器，不操作硬件。

编译器通过 --compiler-bin 指定，工程源码完全来自仓库。输出只写 build 或 --output；
日志记录真实工具版本、起始 Git SHA、工作区状态和所有构建输入哈希，供复核。
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def main():
    """按显式源清单编译，使用同一 scatter 文件链接并保留真实 map/警告。"""
    root = Path(__file__).resolve().parents[1]
    firmware = root / "firmware/stm32f103"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler-bin", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=root / "build/m1")
    parser.add_argument("--scan-direction", type=int, choices=(-1,1), default=1,
                        help="编译期扫描方向；无运行期串口选择，不授权实际运动")
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    compiler = arguments.compiler_bin.resolve()
    project = json.loads((firmware / "project_sources.json").read_text(encoding="utf-8"))
    sources = [firmware / name for name in project["sources"]]
    includes = [firmware / name for name in project["includes"]]
    commands = []
    log = []

    def run(command):
        """执行结构化命令数组，完整保留输出；失败立即停止，绝不把失败写成成功。"""
        command = [str(part) for part in command]
        commands.append(command)
        result = subprocess.run(command, cwd=firmware, text=True, encoding="utf-8", errors="replace", timeout=60,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        log.append(subprocess.list2cmdline(command) + "\n" + result.stdout)
        (output / "build.log").write_text("\n".join(log), encoding="utf-8")
        if result.stdout.strip():
            print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
        if result.returncode:
            raise SystemExit(result.returncode)
        return result.stdout

    version = run([compiler / "armclang.exe", "--version"])
    if "ARM Compiler " + project["compiler_version"] not in version:
        raise SystemExit("编译器版本与冻结配置不一致，停止构建。")
    objects = []
    for source in sources:
        object_file = output / (source.stem + ".o")
        command = [compiler / "armclang.exe", "--target=arm-arm-none-eabi", "-mcpu=cortex-m3",
                   "-mfloat-abi=soft", "-std=c11", "-O2", "-g", "-ffunction-sections", "-fdata-sections",
                   "-Wall", "-Wextra", "-DSTM32F103xB", "-DUSE_HAL_DRIVER", "-D__MICROLIB",
                   "-DBOARD_SCAN_DIRECTION="+str(arguments.scan_direction)]
        # 厂家文件原样保留；其警告仍完整记录，自编源码则将全部警告作为错误。
        if "Drivers" not in source.parts:
            command += ["-Werror"]
        for include in includes:
            command += ["-I", include]
        run(command + ["-MMD", "-MF", output / (source.stem + ".d"), "-c", source, "-o", object_file])
        objects.append(object_file)
    startup = firmware / project["startup"]
    startup_object = output / "startup_stm32f103xb.o"
    run([compiler / "armasm.exe", "--cpu=Cortex-M3", "--debug", "--pd", "__MICROLIB SETA 1",
         startup, "-o", startup_object])
    objects.append(startup_object)
    image = output / "m1_sampling.axf"
    run([compiler / "armlink.exe", "--cpu=Cortex-M3", "--strict", "--library_type=microlib",
         "--scatter", firmware / project["scatter"], "--entry=Reset_Handler", "--callgraph",
         "--map", "--symbols", "--info=sizes,totals,unused,veneers", "--list", output / "m1_sampling.map",
         "--output", image, *objects])
    run([compiler / "fromelf.exe", "--i32combined", "--output", output / "m1_sampling.hex", image])
    size_report = run([compiler / "fromelf.exe", "--text", "-z", image])
    inputs = sorted(path for path in firmware.rglob("*") if path.is_file()
                    and "build" not in path.relative_to(firmware).parts
                    and "DebugConfig" not in path.relative_to(firmware).parts
                    and path.suffix not in {".uvoptx", ".uvguix", ".bak"})
    # 共享控制核依赖：正式固件直接编译canonical测量及反馈模块，禁止复制第二份源码。
    inputs += [root / "control_core" / (name + suffix)
               for name in ("sensor_input", "feedback") for suffix in (".c", ".h")]
    inputs += [Path(__file__).resolve()]
    top_level = subprocess.run(["git", "rev-parse", "--show-toplevel"], cwd=root, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    is_checkout = top_level.returncode == 0 and Path(top_level.stdout.strip()).resolve() == root
    git_head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip() if is_checkout else None
    git_status = subprocess.check_output(["git", "status", "--porcelain=v1"], cwd=root, text=True) if is_checkout else "SOURCE COPY: bind by input hashes"
    evidence = {"head": git_head, "worktree_status": git_status, "compiler_version": version,
                "scan_direction": arguments.scan_direction,
                "optimization": "O2", "target": "STM32F103C8", "size_report": size_report,
                "binary_sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
                "inputs": {str(path.relative_to(root)).replace("\\", "/"):
                           hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs},
                "commands": commands}
    (output / "build_evidence.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
                                               encoding="utf-8")
    print(f"目标构建完成：{image}")


if __name__ == "__main__":
    main()
