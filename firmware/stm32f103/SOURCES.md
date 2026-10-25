# 来源与许可边界

本工程以作者黄铮（best_pureer）的进阶工程为板卡/配置参考：
[stm32_foc_2](https://gitee.com/best_pureer/stm32_foc_2)，
本地 HEAD `672e32f1005d9019140d1a9c754fb8ca2e8eedde`。
基础工程 [stm32_foc](https://gitee.com/best_pureer/stm32_foc) 仅用于对照；本地副本没有 Git 元数据。
不将基础/进阶共同来源互相一致当作独立正确性证明。

## 实际引入

[vendor_sources.json](vendor_sources.json) 是厂家文件准确来源路径、目的路径与 SHA256 清单。
所选文件保留原始字节、原作者与版权声明；来源工具检查指定 HEAD 及所选文件未修改。
本地参考工程仅 MDK 项目有既有改动，未引入该文件。

| 组件 | 实际来源/版本 | 许可依据 |
|---|---|---|
| STM32F1 HAL | 上述参考快照；源码版本宏 1.1.10 | [组件原 LICENSE](Drivers/STM32F1xx_HAL_Driver/LICENSE.txt) 明确包外接收/未附包许可时适用 BSD-3-Clause；附 [ST 条款全文](LICENSES/ST-BSD-3-Clause.txt) |
| CMSIS Core-M | 上述参考快照；版本宏 5.1，具体头有自己的版本号 | [Apache-2.0 全文](Drivers/CMSIS/LICENSE.txt)，各文件署名保留 |
| STM32F1 device、system、startup | STM32F103xB 设备头，ST MCD 系统/启动模板 | [设备组件原 LICENSE](Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt) 明确 fallback Apache-2.0；全文同上 |

BSD 条款附本来自 [ST 官方 HAL 仓库](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/LICENSE.md)。
这不把官方仓库当前 HEAD 当作本工程的源版本；实际使用文件以本地冻结快照和哈希为准。
原 `.ioc` 声明 CubeF1 1.8.6，但实际 HAL 宏为 1.1.10，不能把包号直接当作每个文件的版本。

## 没有引入

作者自定义 `Drivers/motor/`、`algorithm/`、Core USER CODE、原 `.ioc`、原 `.uvprojx`、
DSP 库、资料 PDF/docx 和安装工具均没有复制。
作者公开工程页面提示未指定根级许可证；本地 README/说明也未给自定义代码明确的公开再发布条款。
HAL/CMSIS 许可只覆盖各自组件，不覆盖作者算法。M2 开始移植作者算法前需补齐依据。
不因为当前可浏览/可下载就标记作者自定义源码“已获许可”。

## 本项目新增内容与差异

`board/`、`app/`、`Core/Inc/main.h`、`Core/Inc/stm32f1xx_hal_conf.h`、
`Core/Src/main.c`、`Core/Src/stm32f1xx_it.c`、源清单、scatter 和新 MDK 元数据是本批自编适配。
不新增 FOC、PI 或运动控制算法，也不为仓库其他内容擅自增加统一许可证。

- 保留 Core/Drivers/MDK-ARM 布局和 ARM 6.13.1 工具链，新增工程从同一显式源清单生成。
- 不复制原 Cube 元数据；目前没有经过 CubeMX 保存/再生成验证的 `.ioc`。重新生成前需先建立
  与本工程匹配的配置并复核安全门控，不能拿参考 `.ioc` 覆盖这里的自编入口。
- 去除自动对齐、默认 0.3 A、全局控制输入指针和 TIM3 环路写入路径；这些源码没有被复制。
- 只保留必要的 HAL/CMSIS 源/头；未用的 DSP、RTOS、控制代码和库不属于构建依赖。
- SPI/DMA 不复用原示例的收发同一缓冲和无完成检查路径；本项目独立按协议构造验证/发布。

协议依据为 [MT6701 Rev1.8 第 7.8 节](https://www.magntek.com.cn/upload/pdf/202312/MT6701_Rev.1.8_%E4%B8%AD%E6%96%87%E7%89%88.pdf)，
定时器/ADC 依据 [ST RM0008](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。
板卡 U4 标注 MS8313/DRV8313，DRV 型号的真值表参见 [TI DRV8313 第 7.3 节](https://www.ti.com/lit/ds/symlink/drv8313.pdf)。
实际装配型号、SPI 极性/时间和功率电气效果仍需板端确认。
