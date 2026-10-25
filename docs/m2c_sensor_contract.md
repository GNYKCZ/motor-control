# M2c：原始采样、零偏候选与名义测量

Refs #1/#7；[设计v1.6](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6395820234)。
基线main `9b8696228e6c8273fca909df1a13030210ba8ec9`，已含CubeMX、M2a、M2b。
本批只采样；不接PI、不算有效电角度、不改生成初始化，不烧录/上电/运行电机。

## 来源与实际调用

参考黄铮（best_pureer）的[进阶例程](https://gitee.com/best_pureer/stm32_foc_2)，快照
`672e32f1005d9019140d1a9c754fb8ca2e8eedde`。作者应用源码再发布许可未明确，未复制其USER CODE。

| 来源 | 本批关系 | 行为与变化 |
|---|---|---|
| 例程adc.c的adc_offset_calibration | 参考512次无电流采集，独立重写 | 原普通ADC轮询/静态float累加，改为既有完整注入帧的O(1)整数统计；统计前提未经现场确认，故仅候选 |
| 例程adc.c电流换算 | 参考单位和板级信号链，独立重写 | 原callback用Vref/2居中；本项目显式传入每相候选count，按差值与A/count换算，不复制原回调或原全局变量 |
| conf.h与V2原理图U2/U11、R5/R6 | 参数来源，非实测 | ADC参考3.3V，采样电阻20mΩ，INA199A1名义50V/V；不导入7极对/12V默认换相或母线测量假设 |
| sample_frame/runtime/telemetry | 已有模块直接复用并扩展 | 继续由完整双JEOC/DMA完成帧与单一runtime拥有者发布；前台快照后做浮点/格式化 |
| HAL/CMSIS/CubeMX16输出 | 厂家直接复用，保持原内容 | 不新增驱动、PWM通道或生成补丁，来源与许可仍见固件SOURCES |
| M2a/M2b | 保持原算法和主机测试，不接入固件 | 当前测量没有电角度/方向/独立电流标定，不能送电流PI冒称实物闭环 |

INA199 x1的50V/V见[TI原厂数据手册](https://www.ti.com/lit/ds/symlink/ina199.pdf)。
实际装配、Vref误差、电阻容差、放大器增益/零偏/温漂、接线方向和无电流前提都未实测。

## 数学和统计契约

[sensor_input.h](../control_core/sensor_input.h)和[实现](../control_core/sensor_input.c)为唯一测量实现，
固件与主机直接编译同一文件。无HAL/串口/可变内部状态；配置按值、输出独占不重叠。

```text
nominal_A_per_count = 3.3 / (4096 * 0.020 * 50) ≈ 0.0008056640625 A/count
nominal_current_u_A = (raw_u - estimated_offset_u_count) * nominal_A_per_count
nominal_current_v_A = (raw_v - estimated_offset_v_count) * nominal_A_per_count
mechanical_angle_rad = raw_angle * 2*pi/16384
```

ADC为0..4095，LSB分母4096；角度为0..16383，输出[0,2*pi)，16383→0表示机械单圈回绕，
不做圈数展开、极对数换算或电零位估计。offset为有限[0,4095]的float，允许平均后的半count；
A/count有限且>0。中间double、输出float；非有限/越界配置、无效帧、结果超float范围均false，
非空输出清零无效。is_valid只代表这组显式参数的数学转换有效，绝不代表校准通过。

sensor_offset_init清零调用者上下文；add仅接收调用者已核对连续性/时序/状态的完整帧，
每帧固定次数整数操作，计入前先验证。达到512后add拒绝且不修改；无效帧/raw越界/坏状态失败不提交。
最大累加值512*4095=2096640，不溢出uint32。调用者负责异常冻结，不能跳过坏帧后补数。

runtime仅在DISABLED、候选未中止且不足512时累计。任何无效/重复/时序错误均中止批次，
ADC和编码器完成序号都须逐一连续（允许uint32回绕）；M1允许的“更新但跳号”包也中止候选，
而不改变原M1对该raw包的有效性和故障语义。
即使M1首包宽限仍保持DISABLED，候选也不恢复；任何FAULT中止未满批次。已有min/max/sum保留，
后续好帧不补数；只有重新初始化才能开新批，本批无在线重开命令。收齐也仍是DISABLED，
calibration_verified恒false。故障后仍记录raw证据，但名义转换输出无效。

ADC IRQ只做整数统计；前台在runtime_read_status释放临界区后计算均值、名义电流与机械角。
新快照包含候选字段并统一复制，保持原PRIMASK恢复约束。无需动态分配/等待串口。
临界区是固定大小复制/固定次数统计，实际耗时、WCET/栈水位仍NOT RUN。

## M2C1文本格式与保存

新前缀M2C1显式区别旧M1；原s/f/n/t/e/es/ec/u/v/a/mg/ok/bad/skip/pwm/power字段意义保持。
前台100ms尝试一行，缓冲768B，UART忙跳过并增加skip，TC完成前不重写。

| 新字段 | 意义/单位 |
|---|---|
| cn/cb/ca/cd/cv | 累计有效帧数0..512/候选被首个无效帧中止次数0..1/中止/满512/校准已验证；cv恒0 |
| ou_mc/ov_mc | 当前候选均值，0.001 count；未收齐也可观察，cn=0时值无物理意义 |
| umin/umax/vmin/vmax | 候选批原始count范围；cn=0无物理意义 |
| mv | 转换有效：DISABLED、完整有效新包、候选满批且未中止；不是可信标定或使能位 |
| iu_mA/iv_mA | 名义电流mA，向0截断，量化误差<1mA；mv=0时为0且不得使用 |
| ma_mrad | 未校正机械单圈角mrad，向0截断误差<1mrad；mv=0时为0 |
| scale_nA | 名义nA/count，向0截断；实际计算用上式float系数 |

均值日志截断误差<0.001count；实际转换使用sum/512的float均值，不使用日志截断值。
UART文本无额外传输CRC；ok只描述原采样包验证，不证明串口字节未受干扰。

[保存工具](../tools/sensor_capture.py)保留raw.bin和每行raw_hex/raw_text/字段/错误/设备原32位时间，
metadata.json包含现场填的完整固件SHA、板卡/电机/供电限流、source_kind、工具版本/哈希、UTC起止。
不删坏行、半行、未知版本或超长数据；不把时间回绕擅自改成主机绝对时间；旧M1也可解析。
超长物理行可拆为有界片段，但所有片段共享physical_line编号并保持错误，真实换行后才恢复解析；
is_line_fragment标记这些片段。valid_lines只统计完整有效物理行，invalid_lines统计错误记录/片段，
元数据invalid_count_unit显式说明该计数单位，不将坏行尾部当作新采样。
仅新建输出目录，Ctrl+C保留已有数据。文件解析不需要串口库，现场--port需pyserial3.5且只读不发命令。
串口收集本批NOT RUN。所有示例和测试文件明确为synthetic，不是实测记录。

## 实施前固定验收与运行

| 验收 | 独立依据/工况 | 入口/证据 |
|---|---|---|
| 数值 | 0.25A/count二进制精确手算覆盖4096ADC；16384角度比例/端点回绕；半count；NaN/Inf/越界/溢出 | test_sensor_input.py，纯core真实编译/依赖/符号 |
| 统计 | 0..511整数级数，和130816/均值255.5/min0/max511，两实例；满批/失败不提交 | 同上 |
| runtime | 真实512连续帧均值/min/max、无效/重复/时间/故障冻结、初包无效后不补数，cv恒false | test_m1.py调用真实runtime/telemetry |
| 采集 | 合成行、坏行、半行、字节分块/原字节一致、回绕、元数据/中止/拒绝覆盖 | tests/m1/test_sensor_capture.py，经test_sensor_input.py统一记录 |
| 集成/回归 | ARM/MDK真实链接sensor模块；M1/M2a/M2b回归；CubeMX16输出不变且干净再生成 | build_m1.py、原回归入口、regenerate_m1.py、verify_m1.py |

电流容差`8*FLT_EPSILON*max(1,|expected A|)`，精确二进制用例直接相等；角度绝对预算
`4*FLT_EPSILON*2*pi`。整数统计/冻结逐字段或对象字节相等，不能放宽以迁就实现。

```powershell
python tools/test_sensor_input.py --gcc 'D:\mingw64\bin\gcc.exe'
python tools/test_m1.py --gcc 'D:\mingw64\bin\gcc.exe'
python tools/test_foc_math.py --gcc 'D:\mingw64\bin\gcc.exe'
python tools/test_current_control.py --gcc 'D:\mingw64\bin\gcc.exe'
```

正式构建和CubeMX命令仍见固件README。源清单仅额外允许canonical sensor_input源/头，
不放开其他控制核到固件；共享输入哈希进入目标构建证据。提交后各证据绑定完整head/输入/命令，
PR记录真实资源/警告与独立Review；首轮未提交PASS不能冒称main已验证。
只采样现场步骤见 [独立验收计划](m2c_sampling_check.md)。本批不宣称实物校准、波形/高阻关断或闭环有效。

## Work验证记录

首次未提交软件检查已覆盖上述全部计数、统计、runtime冻结、名义导出与合成文件保存/解析。
M1原协议/状态/快照/背压回归保留并通过；M2a/M2b原测试未改且回归通过。
目标命令行ARM6.13.1/C11/O2实际链接sensor_input_convert和sensor_offset_add；
当前map资源为Code16020B、RO572B、RW12B、ZI2676B，ROM16604B/64KiB，RAM2688B/20KiB。
fromelf按对齐填充分组，显示RO576/RW16/ZI2672，总占用一致。厂家ADCEx原一条括号警告保留，
自编/生成Core无新警告。提交后重新执行目标与MDK、全部主机入口和干净再生成，以PR绑定最终head。
串口实际接收、现场元数据、实机观察、耗时/栈水位均NOT RUN；测试保存文件仅合成数据。
