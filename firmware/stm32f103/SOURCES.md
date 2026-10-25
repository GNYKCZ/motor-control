# 来源与许可边界

本工程以作者黄铮（best_pureer）的进阶工程为板卡/配置参考：
[stm32_foc_2](https://gitee.com/best_pureer/stm32_foc_2)，
本地 HEAD `672e32f1005d9019140d1a9c754fb8ca2e8eedde`。
基础工程 [stm32_foc](https://gitee.com/best_pureer/stm32_foc) 仅用于对照；本地副本没有 Git 元数据。
不将基础/进阶共同来源互相一致当作独立正确性证明。

## 逐文件对照：直接复用、参考改写、暂未引入

本表落实 [设计补充 v1.2 / A13](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6393305178)。
源路径均相对上述进阶例程快照；目标路径均相对本目录。**参考改写**在这里指参考板卡配置、
信号流和官方外设/协议规则后自编适配，不表示逐字复用作者自定义源码，也不能用于规避许可核查。
直接原样复用的准确数量仅为 `vendor_sources.json` 记录的 **39 个厂家文件**。

| 来源文件/组件 | 目标位置或状态 | 复用分类 | 保留事实、行为变化与原因 | M2 或后续条件 |
|---|---|---|---|---|
| `stm32_foc_test.ioc` | [board_config.h](board/board_config.h)、[board_io.c](board/board_io.c)；无目标 `.ioc` | 参考配置；源文件未引入 | 保留 STM32F103、板卡引脚与外设链路知识；未把参考 Cube 工程当作当前工程再生成入口 | 如需 CubeMX 再生成，先建立匹配本工程的配置并验证不会覆盖门控；目前未验证 |
| `Core/Src/main.c` | [main.c](Core/Src/main.c)、[motor_runtime.c](app/motor_runtime.c) | 参考改写，自编入口/生命周期 | 原 HAL 启动→PWM→强拖对齐→默认电流路径改为状态先建立→初始化→只采样；不复制作者 USER CODE | 后续显式使能/校准需要单独状态、边界和用户安排，不能恢复默认通电启动 |
| `Core/Src/adc.c` | [board_io.c](board/board_io.c)、[sample_frame.c](board/sample_frame.c) | 参考数据流，自编发布契约 | 延续 ADC1/2 两路注入同步采样配置；双 JEOC 才发布原始值，电流/角度各保留时间；旧回调内的 Clarke/Park/PI 未迁入 | 核对通道、符号、零偏/增益与电角度后，再通过明确输入送控制核；配置推导不代替板端实测 |
| `Core/Src/spi.c` | [board_io.c](board/board_io.c)、[sample_frame.c](board/sample_frame.c) | 参考配置/时序思路，自编 DMA 交接 | 沿用 SPI1/MT6701 读取链；收发缓冲分开，只有 TC/长度/错误检查后才解析并发布，增加 CRC、序号、读取区间/年龄 | 保持完整发布接口，板端确认波形、器件状态与时间预算后才用于换相 |
| `Core/Src/tim.c` | [board_config.h](board/board_config.h)、[board_io.c](board/board_io.c) | 参考配置后改写 | 保留 TIM1 更新触发、TIM2 延时、TIM4 计时；SSI 延时78→50 us，TIM4 整数分频改为1 MHz；不再由 TIM3 并发写控制状态 | 在接入控制器前冻结实际周期、最迟输出提交时点和参数换算，不照搬频率宏/增益 |
| `Core/Src/usart.c` | [telemetry.c](app/telemetry.c)、[board_io.c](board/board_io.c) | 参考引脚/波特率后改写 | 延续 USART2 115200/8N1；不复制阻塞 `fputc`，使用前台格式化、TXE逐字节发送、TC后释放缓冲 | M3 如增加命令/高速记录，仍不得让通信改控制状态或阻塞高频路径 |
| `Drivers/motor/foc.c`、`foc.h` | 暂未引入；M2 候选 `control_core/foc.[ch]`、`pi.[ch]`、`motor_control.[ch]`，目前不存在 | 暂未引入 | M1 无 FOC/PI 调用者；作者自定义源码许可未明确；现有速度模式也并非都经过电流内环 | 先核实许可和实际 `foc_forward`/电流环调用链，再提取最小必需算法并保留署名/函数映射 |
| `Drivers/motor/motor_runtime_param.[ch]`、`conf.h`、`sin_table.h` | 暂未引入；后续显式配置/状态与必要查表候选 | 暂未引入 | 不复制全局可变输入指针和默认电机参数，不因例程有查表就预建依赖 | 同上取得许可；逐项说明 A/V/rad 单位、极对数/偏置来源、状态所有权和 reset；查表须有实际调用者 |
| `algorithm/filter.[ch]` | 暂未引入 | 暂未引入 | 当前没有速度估算/滤波需求 | 到对应阶段再核对许可、噪声/延迟代价及同一数据集比较 |
| `Drivers/STM32F1xx_HAL_Driver/`、`Drivers/CMSIS/` | [Drivers/](Drivers/)，只选本批必要文件 | 直接原样复用 | 保留组件版本、原字节/署名和分别适用的许可；不复制整个 SDK | 后续只按实际依赖增加，并更新准确来源/哈希清单 |
| ST `Core/Src/system_stm32f1xx.c`、`MDK-ARM/startup_stm32f103xb.s` | [system](Core/Src/system_stm32f1xx.c)、[startup](MDK-ARM/startup_stm32f103xb.s) | 直接原样复用 | ST 系统/启动模板，与作者电机 USER CODE 区分；计入39项厂家文件 | 目标变化才调整；变更需说明与重建验证 |
| `MDK-ARM/stm32_foc_test.uvprojx` | [m1_sampling.uvprojx](MDK-ARM/m1_sampling.uvprojx)、[project_sources.json](project_sources.json)、[scatter](MDK-ARM/m1_sampling.sct) | 保留形态/工具链，项目元数据自编 | 原项目未复制；当前显式源清单与M1范围一致，ARM6.13.1/O2、C8标称容量已真实构建 | 新增实际算法模块时同步源清单和构建；不把参考工程编译成功当作本项目新版本验证 |

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

## M2 优先复用路线与进入条件

本轮只补说明与方案，不新增控制接口源码、PWM 使能或硬件操作。M2 仍需另行固定实施范围。

1. **先看已有调用链**：从原 `Core/Src/adc.c` 的 Clarke/Park、电流环，到 `foc.[ch]` 的
   `foc_forward`/SVPWM 和 `set_motor_pid`；区分直接调制电压的模式与经过电流内环的模式。
   配合 `conf.h`、`sin_table.h` 和 `motor_runtime_param.[ch]` 列出最少依赖，不整包搬入。
2. **先补许可依据再提取**：作者自定义代码仍未见明确的公开再发布条款。若取得依据，
   优先复用最小变换/调制/电流控制代码，保留作者署名、准确快照和源函数→目标函数对应；
   不能以 HAL/CMSIS 条款替代作者许可，也不因缺许可而直接另写整套控制算法。
3. **只先定接口边界**：拟由现有 `board_make_sample`/`motor_sample_t` 提供完整输入，
   runtime 组合未来控制核与板级输出；未来 `control_core/motor_types.h`、`foc.[ch]`、
   `pi.[ch]`、`motor_control.[ch]` 都还是规划位置，当前不创建空模块。
   控制核不读 HAL/DMA 内存，不自行使能硬件，不保留临时输入指针。
4. **行为改变单独验证**：电流换算/电角度→Clarke/Park→d/q PI→联合电压约束→逆变换/SVPWM→
   最终门控。明确 A/V/rad、采样→PWM生效、初始化/reset及饱和顺序；不直接照搬归一化增益。
   保留行为用同输入回归，改变行为用独立方程/手算边界核对；例程输出不是唯一正确性依据。

许可不明确时可继续接口分析、来源核查与独立 oracle 设计；不公开引入作者控制源码。
真实板端采样/时序、供电、下载、对齐和电机运行仍待用户单独安排。此路线不是已实现的 M2。

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
