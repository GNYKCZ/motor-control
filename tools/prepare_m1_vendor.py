"""一次性来源工具：仅复制本批明确列出的厂家文件并记录哈希；不复制作者 USER CODE。

输入为只读参考根目录，输出为本仓库厂家依赖和来源清单。只在文件不存在时写入，
再次运行会核对完全相同的内容，不覆写已修改文件；不读取凭据、不联网。
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def main():
    """解析来源位置，逐项限制文件范围，生成可复核的相对路径/哈希清单。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    arguments = parser.parse_args()
    reference = arguments.reference.resolve()
    root = Path(__file__).resolve().parents[1]
    target = root / "firmware/stm32f103"
    manifest_path = target / "vendor_sources.json"
    if manifest_path.exists() and any(item.get("origin") for item in json.loads(
            manifest_path.read_text(encoding="utf-8"))["files"]):
        raise SystemExit("当前清单已含 CubeMX 修正所需的固件包依赖；本工具仅适用于最初39项来源导入，拒绝覆盖混合来源清单。")
    git_command = ["git", "-c", f"safe.directory={reference.as_posix()}", "-C", str(reference)]
    expected_commit = "672e32f1005d9019140d1a9c754fb8ca2e8eedde"
    actual_commit = subprocess.check_output(git_command + ["rev-parse", "HEAD"], text=True).strip()
    if actual_commit != expected_commit:
        raise SystemExit("参考 SHA 与冻结来源不一致；先复核设计/来源再更改清单。")
    files = []
    hal_headers = ["hal", "hal_def", "hal_cortex", "hal_gpio", "hal_gpio_ex",
                   "hal_rcc", "hal_rcc_ex", "hal_dma", "hal_dma_ex", "hal_flash",
                   "hal_flash_ex", "hal_adc", "hal_adc_ex", "hal_spi", "hal_uart"]
    for name in hal_headers:
        path = f"Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_{name}.h"
        files.append((path, path, "BSD-3-Clause (component fallback)"))
    for name in ["hal", "hal_cortex", "hal_gpio", "hal_rcc",
                 "hal_rcc_ex", "hal_dma", "hal_adc", "hal_adc_ex", "hal_spi", "hal_uart"]:
        path = f"Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_{name}.c"
        files.append((path, path, "BSD-3-Clause (component fallback)"))
    for path, license_name in [
        ("Drivers/STM32F1xx_HAL_Driver/LICENSE.txt", "BSD-3-Clause (component fallback)"),
        ("Drivers/STM32F1xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h", "BSD-3-Clause (component fallback)"),
        ("Drivers/CMSIS/LICENSE.txt", "Apache-2.0"),
        ("Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt", "Apache-2.0 (component fallback)"),
    ]:
        files.append((path, path, license_name))
    for name in ["core_cm3.h", "cmsis_compiler.h", "cmsis_version.h", "cmsis_armclang.h", "mpu_armv7.h"]:
        path = "Drivers/CMSIS/Include/" + name
        files.append((path, path, "Apache-2.0"))
    for name in ["stm32f103xb.h", "stm32f1xx.h", "system_stm32f1xx.h"]:
        path = "Drivers/CMSIS/Device/ST/STM32F1xx/Include/" + name
        files.append((path, path, "Apache-2.0 (component fallback)"))
    # 这两个是带 ST 署名的启动/系统模板，不是例程作者的电机算法或 USER CODE。
    files += [
        ("Core/Src/system_stm32f1xx.c", "Core/Src/system_stm32f1xx.c", "Apache-2.0 (device component fallback)"),
        ("MDK-ARM/startup_stm32f103xb.s", "MDK-ARM/startup_stm32f103xb.s", "Apache-2.0 (device component fallback)"),
    ]
    source_changes = subprocess.check_output(
        git_command + ["diff", "--name-only", "HEAD", "--", *(item[0] for item in files)], text=True).strip()
    if source_changes:
        raise SystemExit("待引入的厂家文件含本地修改，停止：" + source_changes)
    records = []
    for source_name, destination_name, license_name in files:
        source = reference / source_name
        destination = target / destination_name
        content = source.read_bytes()
        if destination.exists() and destination.read_bytes() != content:
            raise RuntimeError(f"拒绝覆盖已修改文件：{destination_name}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.exists():
            destination.write_bytes(content)
        records.append({"source": source_name, "destination": destination_name,
                        "sha256": hashlib.sha256(content).hexdigest(), "license": license_name})
    provenance = {
        "reference_url": "https://gitee.com/best_pureer/stm32_foc_2",
        "reference_commit": "672e32f1005d9019140d1a9c754fb8ca2e8eedde",
        "reference_workspace_note": "本地 MDK 工程有既有修改；未引入该文件。",
        "custom_motor_code": "NOT IMPORTED: no root license specified",
        "files": records,
    }
    (target / "vendor_sources.json").write_bytes(
        (json.dumps(provenance, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    print(f"厂家文件 {len(records)} 项，逐项 SHA256 已记录；未复制作者电机算法。")


if __name__ == "__main__":
    main()
