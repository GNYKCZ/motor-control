# 来源与许可边界

板卡/配置参考黄铮（best_pureer）的进阶工程 [stm32_foc_2](https://gitee.com/best_pureer/stm32_foc_2)，
冻结快照 `672e32f1005d9019140d1a9c754fb8ca2e8eedde`。
基础工程 [stm32_foc](https://gitee.com/best_pureer/stm32_foc) 只用于对照；本地副本无 Git 元数据。
两份同源例程不能代替独立 oracle。本次按 [Issue #7 v1.4](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6393651536)
实际运行 CubeMX 6.12.0 + FW_F1 1.8.6 建立初始化，没有复制作者电机源码。

## 例程 → 配置 → 生成代码 → 运行适配

| 参考入口 | 本项目位置 | 来源分类与差异 |
|---|---|---|
| `stm32_foc_test.ioc` | [m1_sampling.ioc](m1_sampling.ioc) | 参考芯片/引脚/时钟事实后新建；原文件未整体复制。移除 PWM、TIM3、DSP 插件和原启动常量 |
| `Core/Src/main.c` | [main.c](Core/Src/main.c) | CubeMX 生成结构、时钟和外设调用；本项目 USER CODE 建立 BOOT、只采样、轮询和错误门控；没有作者 USER CODE |
| GPIO/全局 MSP | [gpio.c](Core/Src/gpio.c)、[MSP](Core/Src/stm32f1xx_hal_msp.c) | CubeMX 生成：PA8/9/10 GPIO 低，PA4 CS 高后配置输出，PB12 输入上拉，保留 SWD |
| `Core/Src/adc.c` | [adc.c](Core/Src/adc.c) | CubeMX 生成 ADC1 PA0/ADC2 PA1、单次双注入同步、7.5周期；board 校准和双 JEOC 发布，不迁入原 FOC 回调 |
| `Core/Src/spi.c`、DMA | [spi.c](Core/Src/spi.c)、[dma.c](Core/Src/dma.c) | CubeMX 生成 SPI1 1.125 MHz、CPHA2、RX ch2/TX ch3、normal/byte/MINC；board 拥有3字节事务、缓冲、TC/TE与完整检查 |
| `Core/Src/tim.c` | [tim.c](Core/Src/tim.c) | CubeMX 生成 TIM1 CMS3/ARR1799/RCR3/update TRGO、无PWM；TIM2 ITR0 reset、PSC71、比较50 us；TIM4 PSC71。原78 us及约10.2857 MHz计时不沿用 |
| `Core/Src/usart.c` | [usart.c](Core/Src/usart.c) | CubeMX 生成115200/8N1 TX；F1模式要求 PA3 RX 引脚同时选中，但硬件接收关闭；board TXE/TC发送替代原阻塞 fputc |
| 外设 IRQ/ADC 回调中的控制 | [board_io.c](board/board_io.c)、[sample_frame.c](board/sample_frame.c) | 本项目独立适配；保留双JEOC、DMA完成/错误、CRC、时间和序号。生成 IRQ 不导入，避免重复标志所有权 |
| 状态/导出 | [motor_runtime.c](app/motor_runtime.c)、[telemetry.c](app/telemetry.c) | 本项目自编，BOOT/DISABLED/FAULT、故障锁存和一致快照；没有使能、对齐或默认电流 |
| HAL/CMSIS/system/startup | [vendor_sources.json](vendor_sources.json) | 原39项来自参考快照；本次新增6项必要HAL文件与2项包许可，共47项原始文件 |
| 原 `.uvprojx` | [源清单](project_sources.json)、[MDK工程](MDK-ARM/m1_sampling.uvprojx)、[scatter](MDK-ARM/m1_sampling.sct) | 原工程未复制；本项目生成器维护ARM6.13.1/O2、64KiB/20KiB唯一构建；CubeMX不拥有正式构建元数据 |
| `Drivers/motor/foc.[ch]`、参数/查表、`algorithm/` | 未引入作者源码 | 公开再发布许可尚未明确，HAL/CMSIS条款不覆盖作者算法。M2a独立数学核心见仓库 `docs/m2a_foc_contract.md`；本批不改动或接入它 |

## 厂家原始文件与许可

[vendor_sources.json](vendor_sources.json) 区分参考快照与新增固件包来源，记录每项来源、目的路径和SHA256。
所选文件保留原始字节及署名；原参考工程只有MDK项目有既有修改，未引入。
`prepare_m1_vendor.py` 仅适用于最初39项导入，当前混合来源清单会拒绝被它覆盖。

| 组件 | 来源/版本 | 许可依据 |
|---|---|---|
| 原STM32F1 HAL | 参考快照，源码宏1.1.10 | [组件原许可](Drivers/STM32F1xx_HAL_Driver/LICENSE.txt) 的包外fallback BSD-3-Clause；[条款全文](LICENSES/ST-BSD-3-Clause.txt) |
| 新HAL TIM/TIMEx源及TIM/TIMEx/PWR/EXTI头 | STM32Cube FW_F1 1.8.6，原字节 | [组件表](LICENSES/STM32CubeF1-components.md)、[包许可](LICENSES/STM32CubeF1-package.md)明确HAL适用BSD-3-Clause；全文同上 |
| CMSIS Core-M | 参考快照，版本宏5.1，各头有自己的版本 | [Apache-2.0](Drivers/CMSIS/LICENSE.txt) 与文件署名 |
| STM32F1 device/system/startup | 参考快照ST模板 | [设备许可](Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt) fallback Apache-2.0 |
| CubeMX生成区 | 本机CubeMX 6.12.0的16个选定输出 | 保留每个ST Header原条款：组件无LICENSE时AS-IS；不擅自标为BSD或项目统一许可证 |

工具安装目录、包全集、资料PDF/docx、原.ioc、原.uvprojx、DSP库和作者USER CODE均未加入仓库。
原.ioc包号不代表其每个文件版本；使用文件以逐项哈希为准。

## 再生成边界

[regenerate_m1.py](../../tools/regenerate_m1.py) 的 `GENERATED` 是16个正式生成文件的导入清单。
每次用 `config load`、`project setCustomFWPath`、`project couplefilesbyip 1`、`generate code`
在新忽略目录生成，将输出Inc/Src映射到正式Core/Inc、Core/Src，仅统一LF。
默认精确比较；`--write` 才更新。main保留正式同名USER CODE，Header仍用本次真实生成文本。
其余15个文件无自编补丁；业务修改仅在main USER CODE或独立board/app。

生成IRQ、临时system和工程片段不导入、不参与编译。保留自编Cortex路由与board IRQ唯一所有权，
正式system/startup仍取来源清单内原始模板。运行命令见 [README](README.md)，
实际证据见 [本次验证](../../docs/cubemx_validation.md)。

没有CubeMX无法表达的定时器参数覆盖。board只做运行操作：校准ADC、SPI SPE、清初始化UG状态、
启动计数/事件、每帧DMA地址/长度/中断、时间扩展。DMA基础格式与生成配置一致，寄存器事务避免
HAL IRQ清掉board独占标志。初始化前及故障门控将PA8/9/10置普通低电平并清MOE/CCER，
与生成配置相同；低侧制动不等于功率高阻关断。

协议/寄存器依据仍为 [MT6701 Rev1.8](https://www.magntek.com.cn/upload/pdf/202312/MT6701_Rev.1.8_%E4%B8%AD%E6%96%87%E7%89%88.pdf)、
[ST RM0008](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
与 [TI DRV8313](https://www.ti.com/lit/ds/symlink/drv8313.pdf)。
U4实际装配、SPI波形/实时预算、复位瞬态和功率电气效果需板端确认；本批未烧录、上电或启动电机。
