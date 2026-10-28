# M1：先只采样，禁止 PWM 调制

本工程是 Issue #1 的 P0a 软件基线，初始化现依据 [架构 #7 v1.4](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6393651536)。
只实现完整原始采样、设备时间、禁用生命周期和最小串口状态；没有电流闭环、速度/位置控制、
通电对齐、使能命令或柔顺模式。P0已按用户接受的基础功能口径完成；连续性能与P1输出尚未验收。

**输出边界：三路 PWM 永久禁止；功率桥高阻关断仍是未满足项。** 用户已接受先交软件基线。
本地 V2.0 原理图的 EN1–3、nSLEEP、nRESET 接固定电源，软件不能关闭它们。
PA8/9/10 固定为普通 GPIO 低电平，不配置成定时器复用；TIM1 的 MOE 和 CCER 保持 0。
对 DRV8313 来说，这会保持低侧导通制动，而非让电机三相高阻悬空；实际 U4 型号还需看板。
旧M2c固件已有有限烧录/板端观察，详见[记录](../../docs/m3b_feedback_integration.md#已有板端证据与本批限制)。
M3B1/M3B2已有用户自烧录积压故障观察；本批M3B3仅软件优化，不自动烧录，不能据构建宣称电气保护有效。

M2c 已在同一固件增加 [名义测量与零偏候选](../../docs/m2c_sensor_contract.md)。
runtime从DISABLED连续完整帧累计512次整数统计；异常冻结，不补数，不自动校准通过。
历史M2C1在前台快照后换算名义A/机械单圈rad；当前M3B3退出这些展示换算，内部统计仍保持。
格式、误差/来源与保存命令见该契约；[现场完整验收步骤](../../docs/m2c_sampling_check.md)尚未完成。

M3b将同一feedback模块接入前台，输出版本为[M3B1](../../docs/m3b_feedback_integration.md#m3b1日志)。
每10个完整有效原始帧产生一个输入，真实序号不重编，单槽邮箱满/原始跳号锁故障8；
板端执行时间和真实速度仍NOT RUN，旧M2c板端观察不证明本批性能。

历史[M3B2](../../docs/m3b_timing_check.md)已实测多个日志步骤超过约1ms，板端检查未通过。
当前输出为[M3B3](../../docs/m3b3_compact_log.md)：只取紧凑快照并发送16个必要字段，
不再做名义量/均值/64位展示换算，每poll只做一个有界uint32转换。新版本尚未板测。

## 从哪里开始阅读

按下面顺序读，自编函数和字段均有中文说明；厂家代码维持原署名与命名，避免改名后难以查手册。

1. [范围与验收](../../docs/m1_sampling_contract.md)：先知道本批做什么，以及不做什么。
2. [main.c](Core/Src/main.c)：只有初始化、失联检查和前台导出，是整条链的入口。
3. [board_config.h](board/board_config.h)：本批所有时间上限、计数范围与容量。
4. [sample_frame.h](board/sample_frame.h) 和 [sample_frame.c](board/sample_frame.c)：原始数值含义、完整帧、CRC 和时间规则。
5. [motor_runtime.h](app/motor_runtime.h) 和 [motor_runtime.c](app/motor_runtime.c)：谁拥有状态，什么时候进入 FAULT。
6. [board_io.h](board/board_io.h) 和 [board_io.c](board/board_io.c)：原理图引脚→外设→完成事件；这里才访问硬件。
7. [telemetry.c](app/telemetry.c) 与 [sensor_input](../../control_core/sensor_input.h)：前台名义测量/导出、串口忙时跳过。
8. [测试](../../tests/m1/test_sampling.c)、[来源清单](SOURCES.md) 和 [验证记录](../../docs/m1_validation.md)。

## 从现有例程找到本项目对应代码

**最初直接复用了 39 个厂家文件；本次增加6个固件包依赖及2个原始许可文件，共47项。
外设初始化由 CubeMX 6.12.0 实际生成；应用层和采样适配是参考例程配置/数据流后自编的，
作者的 FOC、PI 包装和模式代码尚未迁入。** “参考已有例程”不等于“所有文件原样复制”。
下面左侧路径以进阶例程 `stm32_foc_2` 为根，右侧链接以本工程为根；基础例程用于对照学习。
来源分类延续 [Issue #7 v1.2](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6393305178)，
本次按 v1.4 恢复可再生成的初始化，M1 的只采样行为不变。

| 原例程入口 | 本项目对应入口 | 处理方式与学习重点 |
|---|---|---|
| `stm32_foc_test.ioc` | [m1_sampling.ioc](m1_sampling.ioc) | **参考配置，新建目标**：保留芯片/引脚/时钟事实，移除 PWM、TIM3、DSP 扩展和原启动假设 |
| `Core/Src/main.c` | [main.c](Core/Src/main.c)、[motor_runtime.c](app/motor_runtime.c) | **CubeMX 生成 + 自编 USER CODE**：生成时钟/外设调用，USER CODE 建立状态、启动采样并轮询导出 |
| `Core/Src/adc.c` | [adc.c](Core/Src/adc.c)、[board_io.c](board/board_io.c) | **CubeMX 生成配置 + 自编完成事件**：双 ADC 注入同步；双 JEOC 才发布原始值 |
| `Core/Src/spi.c`、DMA | [spi.c](Core/Src/spi.c)、[dma.c](Core/Src/dma.c)、[board_io.c](board/board_io.c) | **CubeMX 生成基础配置 + 自编 DMA 事务**：配置 SPI1/双 DMA；board 管理缓冲、CRC/长度/时间和完整发布 |
| `Core/Src/tim.c` | [tim.c](Core/Src/tim.c)、[board_io.c](board/board_io.c) | **CubeMX 生成**：TIM1 仅计数/TRGO，TIM2 50 us，TIM4 1 MHz；board 只清状态、起动计数/事件和扩展时间 |
| `Core/Src/usart.c` | [usart.c](Core/Src/usart.c)、[telemetry.c](app/telemetry.c) | **CubeMX 生成 + 自编有界收发**：115200/8N1 TX_RX；RX/TXE/TC由board_uart独占，命令语义见P1a1契约 |
| `Drivers/motor/foc.c`、`foc.h` 及控制参数/查表 | **M1 暂未引入**；未来控制核位置见 [SOURCES.md](SOURCES.md) | 当前无控制环调用者，作者自定义源码公开再发布许可尚未明确；不能把可运行参考工程冒称本仓库已移植 FOC |
| HAL/CMSIS、ST 系统和启动模板 | [Drivers/](Drivers/)、[system_stm32f1xx.c](Core/Src/system_stm32f1xx.c)、[startup](MDK-ARM/startup_stm32f103xb.s) | **直接原样复用**：39 个必要厂家文件逐项保留原字节、许可与署名，见 [准确来源/哈希清单](vendor_sources.json) |
| `MDK-ARM/stm32_foc_test.uvprojx` | [m1_sampling.uvprojx](MDK-ARM/m1_sampling.uvprojx)、[project_sources.json](project_sources.json) | **保留工程形态，重建源清单**：沿用 ARM Compiler/O2 约束，原项目文件未直接复制 |

逐项的保留行为、改动原因、许可边界和 **M2 优先复用的前置条件**见 [来源说明](SOURCES.md)。
后续会优先核查例程已有的 `foc_forward`、Clarke/Park、SVPWM 与电流控制路径，
取得明确许可后才按最小调用链提取；不从空白重造整套算法，也不先建立无人调用的控制目录。
这块板本身可以输出 PWM；当前 M1 不输出 PWM 是本批软件选择，不能误读为板卡不能驱动电机。

## 实际调用链与状态所有权

```text
Reset_Handler → SystemInit → C 运行库 → main
  HAL_Init
  motor_runtime_init → board_disable_output → 建立 BOOT
  SystemClock_Config + MX_GPIO/DMA/ADC1/ADC2/SPI1/TIM1/TIM2/TIM4/USART2_Init
  board_initialize_devices → 校准 ADC、启用 SPI、清初始化状态并起动 TIM4
  motor_runtime_finish_boot → 成功进入 DISABLED，失败进入 FAULT
  board_start_sampling → 只启动计数/采样事件，不启动 PWM 通道
  while: motor_runtime_check_deadline + motor_runtime_poll_feedback
         + command_poll（回执优先）或telemetry_poll_status

TIM1 更新 TRGO ─┬→ ADC1/2 同步注入转换
                └→ TIM2 计数归零，50 us 后比较 IRQ 开始 SSI/DMA
DMA1 Channel2 TC → 检查完整传输 → CRC/状态校验 → 一次替换 completed_encoder
ADC1_2 IRQ → 确认双 JEOC → 取完成帧副本 → board_make_sample
           → motor_runtime_handle_sample → 一次发布状态
前台 → motor_runtime_read_status（一致复制）→ snprintf（仅前台）
     → UART RX/TXE/TC IRQ（最多一个RX和一个TX事件；TC后释放缓冲）
```

CubeMX 拥有外设参数、HAL 句柄、MSP/NVIC；board 拥有 DMA 事务、完成编码器帧和设备计数；runtime 拥有生命周期/故障/状态快照；
telemetry 拥有导出行缓冲。没有可被前台逐字段改写的控制上下文指针，也没有并发环路状态。

- BOOT：尚未准备好采样，不允许把输入当作正常工作包。
- DISABLED：允许采样，PWM 永久禁止，等待有效首帧最多 1 ms。
- FAULT：初始化、帧校验、序号、超期、DMA 或 nFAULT 异常锁存；后续好帧也不自动恢复。

本批没有 FAULT 复位/使能串口命令；这两个动作不能因枚举名称或规划存在就视为已支持。
超时检查使用设备微秒计数；前台先取时间、ADC 后发布新包的交错会被识别，避免误判巨大模差为超时。

## 一个最小数据例子

以下M1例子说明保留字段的意义；当前完整输出前缀已为M3B1，新增统计/名义量字段见M2c契约。

假设双 ADC 完成时设备计数为 1000 us，采样序号为 123：

```text
原始 ADC：U=1234，V=2345（12 bit count，不是 A）
完成编码器：序号123，读取开始950 us，完成972 us，原始角度123 count，Mg=0
CRC 合格；传输22 us <=40 us；从开始到 ADC 完成50 us <=200 us
→ 有效完整包 → runtime 保持 DISABLED
→ M1,s=1,f=0,n=123,t=1000,e=123,es=950,ec=972,u=1234,v=2345,a=123,mg=0,ok=1,bad=0,skip=0,pwm=blocked,power=unverified
```

示例使用主机人工数据，**不是板端采集结果**。真实链路从未完成的 DMA 缓冲读取不到半帧；
若双 ADC 的任何 JEOC 缺失，本 IRQ 只锁存故障而不构造“完整包”。DMA 完成前保留上个完成包；
旧包若重复/过期会被 runtime 拒绝，不能把“最近一次读取”冒充新角度。

`t` 是 ADC 完成 IRQ 的设备时间；`es/ec` 是 CS 开始/接收完成事件的设备时间，
它们构成传输区间，不是传感器内部精确采样时刻。考虑手册 <5 us 内部延迟、SPI 传输和 IRQ
延迟后，物理角度年龄还需实测/进一步界定。本批不使用角度做实时换相。

串口是115200、8N1、TX_RX；周期日志每100ms尝试开始一行。当前前缀`M3B3`，固定16字段的
含义、单位、范围和已删字段理由见[M3B3契约](../../docs/m3b3_compact_log.md)。
M1/M2C1/M3B1/M3B2是历史格式，解析器仍兼容；不把历史完整字段表当作当前周期输出。
UART忙时不重写在途缓冲，有界放弃本行，不再维护未输出的skip计数；周期日志没有主机ACK；P1a1请求有独立P1R1回执，
发送端状态不代表电脑已收到数据。功率桥高阻仍未验证，删掉固定power标签不表示验证通过。

## 时间：哪些是配置推导，哪些还没有测量

| 项目 | 本批值/依据 | 边界 |
|---|---|---|
| 主频 | 8 MHz HSE ×9 =72 MHz | 标称配置，尚无板端测量 |
| APB1/APB2 定时器 | APB1=36 MHz、定时器×2；APB2=72 MHz | 两者定时器时钟均72 MHz |
| TIM1 | PSC=0、ARR=1799、中心对齐、RCR=3 | 完整上/下计数周期2×1799 tick，约20.011 kHz；本批只计数无 PWM 输出 |
| ADC TRGO | 每4个边界更新一次，4×1799/72 MHz≈99.944 us | 约10.0056 kHz，不能照宏名直接写40 kHz PWM |
| ADC 时钟/转换 | 12 MHz；采样7.5 +转换12.5=20周期 | 配置转换约1.667 us，回调延迟另算 |
| 编码器读取 | TIM2 1 MHz，更新后50 us；SPI 72 MHz/64=1.125 MHz | 24 bit≈21.333 us；完成 IRQ 额外延迟未测 |
| 设备计时 | TIM4 PSC=71，1 MHz，16 bit计数扩展为32 bit | 65.536 ms溢出 IRQ；不得屏蔽 IRQ 超过一次溢出 |
| 首帧/年龄/失联 | 1 ms /200 us /300 us | M1 保守软件阈值，不能直接作为闭环采样预算 |

相对参考：TIM1 ARR/RCR 和 ADC 触发关系保留；SSI 开始由更新后78 us 调整为50 us，
避免24 bit传输几乎挤到下一次 ADC 边界。TIM4 从参考整数分频的约10.2857 MHz
改为精确配置1 MHz，以避免把7分频误当0.1 us。没有迁移环路增益或改变控制算法，因为 M1 没有控制器。

预期第 k 次 ADC 使用本周期前已完成的一帧；首包没有合法编码器时标无效。
DMA 迟到/BSY/错误会保守拒绝并可能锁 FAULT，不在中断里等待 SPI 完成。
CS 在 ADC IRQ 拉高、下一次 TIM2 比较时再拉低；尾保持时间和 ADC/DMA 的实际先后关系待板测。

## 构建与软件测试

目标 STM32F103C8：按标称 **64 KiB Flash、20 KiB RAM** 链接；Cortex-M3，无 FPU，软件浮点 ABI。
依赖均在仓库内；只需外部编译器和 Python。无需参考目录、DSP 二进制库或安装资料目录。
Python 使用标准库；实际测试工具版本见验证记录。命令从仓库根目录运行：

```powershell
python tools/build_m1.py --compiler-bin 'D:\Keil_v5\ARM\ARMCLANG\bin'
python tools/test_m1.py --gcc 'D:\mingw64\bin\gcc.exe'
python tools/generate_m1_mdk.py --check
python tools/verify_m1.py
```

若 `python` 不在 PATH，可用本机 Python 完整路径；不要把 Python 安装目录加入工程。
目标构建冻结为 ARM Compiler 6.13.1、C11、O2、MicroLIB。自编代码 `-Wall -Wextra -Werror`；
厂家代码警告保留在日志。本机 HAL ADCEx 有一条多余括号警告，不为清零警告改第三方源码。

输出在 `build/m1/`：axf、hex、map、静态调用图和 `build_evidence.json`；
主机测试证据在 `build/host_m1/test_evidence.json`。记录会明确当前完整 HEAD 与未提交修改，
并记录每个构建输入哈希；未提交工作区的 PASS 不能冒充干净 commit 的验证。

MDK 用户可打开 [m1_sampling.uvprojx](MDK-ARM/m1_sampling.uvprojx)，选择 `m1_sampling` 后只 Build：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -r firmware/stm32f103/MDK-ARM/m1_sampling.uvprojx -t m1_sampling -j0 -o "$PWD\build\m1\mdk_build.log"
```

MDK 在本机返回后需核对日志里的 `0 Error(s)` 与实际产物，GUI 程序启动成功不等于构建成功。
应等待本次日志出现Build Time Elapsed，并检查文件更新时间与当前源；可能异步启动时不要读旧日志当作通过。
MDK 项目由显式源清单生成；CubeMX 负责初始化源码，两者交付同一套固件，见 [来源说明](SOURCES.md)。
此处所有命令只构建/运行主机测试，不执行下载和调试。

## CubeMX 再生成与所有权

使用 **CubeMX 6.12.0、FW_F1 1.8.6**。从仓库根运行，路径按本机安装位置填写：

```powershell
python tools/regenerate_m1.py --cubemx 'D:\FOC_Tools\STM32CubeMX-6.12.0\STM32CubeMX.exe' --package "$env:USERPROFILE\STM32Cube\Repository\STM32Cube_FW_F1_V1.8.6"
```

默认在全新 `build/cubemx_<时间>/generated` 目录实际生成并只读比较；16 个生成文件完全相同才通过。
输出保留实际命令、完整日志、工具/配置/输入及结果哈希。检查通过后若确需更新生成内容，在同一命令末尾加
`--write`。仅导入清单内文件、统一 LF，保留正式 `main.c` 的同名 USER CODE 区；ST Header 不豁免对比。
生成代码其余部分不手改。先改 `.ioc`，重新生成，再构建和回归；更新源码清单后运行 MDK 生成器。
工具版本/年份改变造成 Header 差异也需明确复核。逐项证据见 [本次验证](../../docs/cubemx_validation.md)。

外设 `.c/.h` 和全局 MSP 原样生成，无手工寄存器参数覆盖。PA3 因 CubeMX F1 的 Asynchronous 模式要求
TX/RX 引脚成对选择而保留为输入；P1a1将 `UartMode` 改为 `UART_MODE_TX_RX`，
由[board_uart.c](board/board_uart.c)拥有有界RX/TX，应用调用[command.c](app/command.c)处理命令。
`board_initialize_devices` 执行校准、SPI SPE运行使能、清UG状态、起动TIM4，再启用UART接收IRQ；采样起动另由
`board_start_sampling` 负责。`board_disable_output` 是初始化前及故障时可调用的独立门控，始终把
PA8/9/10 设为 GPIO 低并清 MOE/CCER；它与生成 GPIO 低电平配置一致，不覆盖外设参数。
复位到 `motor_runtime_init` 前引脚仍遵循芯片复位状态，软件不能保证上电全过程的电气状态。

CubeMX 还会输出 IRQ 路由和临时 system/工程片段，它们不导入、不参与正式构建。
本项目保留自编 [Cortex 异常路由](Core/Src/stm32f1xx_it.c) 和 board 外设 IRQ，
避免生成 HAL IRQ 与完整帧发布路径争用标志；ST system/startup 仍取来源清单内的原始模板。
不要直接在正式目录点击“生成工程”覆盖这些文件，应通过上述受限导入入口更新。

## 中文术语表与命名

| 英文/缩写 | 中文 | 本项目用途 |
|---|---|---|
| board / runtime / telemetry | 板级 / 运行管理 / 状态导出 | 三个状态所有者，职责不交叉 |
| raw / measured / target / limited | 原始 / 测量 / 目标 / 受限后 | M1 只用 raw，不冒称已校准 measured current |
| sequence / is_valid | 序号 / 是否有效 | 判断重复、完整性与时间规则 |
| ADC (Analog-to-Digital Converter) | 模数转换器 | PA0/1 两路原始电流电压采样 |
| PWM (Pulse-Width Modulation) | 脉宽调制 | 本批永久禁止输出，计数触发和输出使能分开 |
| DMA (Direct Memory Access) | 直接内存访问 | SSI 三字节接收；只有完整后才能发布 |
| SPI / SSI | 串行外设接口 / 同步串行接口 | STM32 SPI 用作 MT6701 SSI 时钟与接收 |
| CRC | 循环冗余校验 | 用18 bit数据和6 bit校验判断帧损坏 |
| JEOC / TC / TXE | 注入转换完成 / 传输完成 / 发送寄存器空 | 通过硬件事件判断阶段完成 |
| PRIMASK / critical section | 中断屏蔽状态 / 临界区 | 保存旧值、屏蔽、复制完整对象、恢复旧值 |
| MOE / CCER / TRGO | 主输出使能 / 通道使能寄存器 / 触发输出 | 采样触发可工作，但 MOE/通道不开放 |
| FOC / PI / d/q | 磁场定向控制 / 比例积分 / 旋转坐标轴 | 后续 M2，M1 没有实现 |

物理单位后缀保留标准大小写，例如 `_us`、`_rad_s`、`_A`；本批原始计数用 `_count`，
不能仅改名加 `_A` 就当作已标定电流。厂家函数不重命名；新增函数采用模块+动作+对象，
公共接口不用 cfg/ctx/fdb/tmp。注释讲明作用、调用前提、状态与错误，避免只逐行翻译语法。

P1a1的23B请求、56B回执、CRC、序号、STOP优先级与现场工具详见
[无输出命令往返契约](../../docs/p1a1_command_link.md)。ARM/PULSE拒绝，板端实际RX仍NOT RUN。
