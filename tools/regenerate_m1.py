"""用 CubeMX 6.12.0 实际生成初始化并核对正式固件；不构建、不下载、不操作硬件。

默认只检查。--write 只更新清单内生成文件，保留正式 main.c/tim.c 的 USER CODE 内容。
每次在 build 下新建干净目录，完整日志及哈希保留在那里；不覆盖 board/app/IRQ。
"""
import argparse
import datetime
import hashlib
import json
import re
import subprocess
from pathlib import Path

GENERATED = [*(f"Inc/{name}.h" for name in ("adc", "dma", "gpio", "main", "spi", "tim", "usart", "stm32f1xx_hal_conf")),
             *(f"Src/{name}.c" for name in ("adc", "dma", "gpio", "main", "spi", "tim", "usart", "stm32f1xx_hal_msp"))]
USER = re.compile(r"(/\* USER CODE BEGIN (.*?) \*/)(.*?)(/\* USER CODE END \2 \*/)", re.S)


def preserve_user_code(generated, current):
    """仅保留同名 USER CODE 区；Header 保留本次真实 ST 生成文本，不隐去版权差异。"""
    blocks = {m[2]: m[3] for m in USER.finditer(current) if m[2] != "Header"}
    generated_names = {m[2] for m in USER.finditer(generated)}
    if not blocks.keys() <= generated_names:
        raise SystemExit("生成模板删除了已有 USER CODE 区；拒绝丢弃业务代码，请先人工复核。")
    return USER.sub(lambda m: m[1] + blocks.get(m[2], m[3]) + m[4], generated)


def main():
    """固定工具/包与目标配置，在全新输出目录生成并逐文件精确比较，失败非零退出。"""
    root = Path(__file__).resolve().parents[1]
    fw = root / "firmware/stm32f103"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cubemx", type=Path, required=True)
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    cube, package = args.cubemx.resolve(), args.package.resolve()
    if "FW.F1.1.8.6" not in (package / "package.xml").read_text():
        raise SystemExit("需要 FW_F1 1.8.6 包。")
    # CubeMX 的安装记录是版本来源；IOC 字段本身不证明运行的工具版本。
    installation = (cube.parent / ".installationinformation").read_text(errors="replace")
    if "6.12.0" not in installation:
        raise SystemExit("需要 CubeMX 6.12.0。")
    output = root / "build" / ("cubemx_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=False)
    raw = output / "generated"
    # -Dfile.encoding 防止 Windows 缺省编码破坏中文安装路径；不依赖登录/联网。
    script = output / "generate.txt"
    script.write_bytes((f'config load "{(fw / "m1_sampling.ioc").as_posix()}"\n'
                        f'project setCustomFWPath "{package.as_posix()}"\n'
                        'project couplefilesbyip 1\n'
                        f'generate code "{raw.as_posix()}"\nexit\n').encode("utf-8"))
    command = [str(cube.parent / "jre/bin/java.exe"), "-Dfile.encoding=UTF-8", "-jar", str(cube), "-q", str(script)]
    with (output / "generation.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, timeout=180)
    log_text = (output / "generation.log").read_text(encoding="utf-8", errors="replace")
    if result.returncode or "Code succesfully generated" not in log_text or re.search(r"^KO\s*$", log_text, re.M):
        raise SystemExit(f"CubeMX 生成失败，查看 {output / 'generation.log'}")
    records, differences = [], []
    for name in GENERATED:
        source, dest = raw / name, fw / "Core" / name
        generated = source.read_text(encoding="utf-8")
        current = dest.read_text(encoding="utf-8") if dest.exists() else ""
        integrated = preserve_user_code(generated, current) if name in {"Src/main.c", "Src/tim.c"} else generated
        content = integrated.encode("utf-8")  # 仅统一 CRLF→LF，不修剪厂家空白。
        if not dest.exists() or dest.read_bytes() != content:
            differences.append(name)
            if args.write:
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(content)
        records.append({"path": "Core/" + name, "raw_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                        "integrated_sha256": hashlib.sha256(content).hexdigest()})
    evidence = {"head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                "status": subprocess.check_output(["git", "status", "--porcelain=v1"], cwd=root, text=True),
                "command": command, "cubemx": "6.12.0", "package": "FW_F1 1.8.6",
                "cubemx_sha256": hashlib.sha256(cube.read_bytes()).hexdigest(),
                "ioc_sha256": hashlib.sha256((fw / "m1_sampling.ioc").read_bytes()).hexdigest(),
                "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "package_xml_sha256": hashlib.sha256((package / "package.xml").read_bytes()).hexdigest(),
                "files": records, "differences_before_write": differences, "write": args.write}
    (output / "regeneration_evidence.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"CubeMX 6.12.0 / FW_F1 1.8.6: {len(records)} files; differences={differences}; evidence={output}")
    if differences and not args.write:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
