# M2a：纯 FOC 数学契约、来源与验收

主路线图 #1；[设计 #7 v1.3 / M2a](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6393462846)。
起始 main 为 `7d37e6c26f6407130b10b689bd23ce601f9b5c2e`，承接已合并 M1 PR #8。
本页在实施前固定公式、范围和误差预算；实际命令、完整被测 SHA 与结果在对应 PR/运行证据中记录。

## 本批范围与最小文件

- `control_core/foc_math.h`、`.c`：两相电流→d/q，以及d/q电压→限幅后电压和三相占空比。
- `tests/control_core/test_foc_math.c`：独立三相投影/极坐标与线电压 oracle、边界和交错调用。
- `tools/test_foc_math.py`：独立主机编译/执行、实际依赖与符号核对、SHA/配置/原始结果记录。
- 本页、根 README：学习入口、例程对应、行为差异和复现说明。

不修改 `firmware/stm32f103/`、M1 输出禁用或构建配置，不接 ADC/runtime、不写 PWM；
不实现电流 PI、模式/三环/柔顺、标定/对齐，不创建尚无调用者的其他控制模块。
当前调用者只有主机测试。M2a 结束后停止，不自动进入 M2b。

## 来源与逐函数对应

主要参考：作者黄铮（best_pureer）的 [进阶例程](https://gitee.com/best_pureer/stm32_foc_2)，
准确快照 `672e32f1005d9019140d1a9c754fb8ca2e8eedde`；基础例程仅用于对照。
作者自定义控制源码公开再发布许可未明确。**本批不复制原 `foc.c`、USER CODE、查表或 DSP 库**，
也不为它们添加新许可证；读取例程用于确定职责、符号和行为差异。
数学实现依据本设计的方程独立编写，标准 C 数学库来自使用者的工具链，没有新增第三方源文件。

| 例程实际入口 | 本项目对应 | 保留 / 改变 / 未引入 |
|---|---|---|
| `Core/Src/adc.c` 内 `arm_clarke_f32`、`arm_park_f32` 调用 | [foc_transform_current](../control_core/foc_math.c) | 保留幅值不变变换和q轴符号；输入为显式已校准A/已知电角度rad，不复制ADC换算或回调，也不接板级 |
| `Drivers/motor/foc.c` 的 `svpwm` 内 `arm_inv_park_f32` | [foc_modulate_voltage](../control_core/foc_math.c) 的逆Park | 保留坐标约定；用rad和标准数学库，不引入整数角度查表或归一化电压接口 |
| `svpwm` 的逐轴±0.9限制 | 同一函数中的联合矢量限幅 | **改变**为显式 V、母线V、调用者调制限额；等比缩放，保持方向，不暗设0.9 |
| `svpwm` 的扇区/有效矢量时间组合 | 同一函数中的相电压中点注入 | 使用连续等价SVPWM形式，线性区对照线电压；不复制扇区表或作者源码 |
| `foc_forward`→`set_pwm_duty` | 纯函数返回 `foc_modulation_t` | **不引入硬件写入**，不依赖全局指针，结果不含使能位；错误不产生可用输出 |
| `torque_d_loop`、`torque_q_loop`、`set_motor_pid`、各 `motor_mode_*` | 本批没有对应实现 | 不迁移 PI/增益、模式切换、默认电流或启动对齐，后续另行设计/许可核查 |
| `conf.h`、`motor_runtime_param.[ch]`、`sin_table.h` | 本批没有对应依赖 | 不复制默认参数、可变状态或查表，避免不明许可/板卡常量进入纯数学层 |

官方公式/坐标参考：[Arm Clarke](https://arm-software.github.io/CMSIS-DSP/main/group__clarke.html)、
[Arm Park](https://arm-software.github.io/CMSIS-DSP/main/group__park.html)、
[Arm inverse Park](https://arm-software.github.io/CMSIS-DSP/main/group__inv__park.html)。
这几个链接是数学约定依据，不表示本项目已引入当前版本 CMSIS-DSP 或从其源码复制实现。
原例程的归一化量与逐轴限制行为不同，不能把本模块叫作无差异搬迁。

## 公式、单位和输入前提

测试输入为人工合成的**已校准两相电流**和**已知电角度**。M1 只有原始 ADC/机械角，
尚未确认零偏/增益、方向、电零位，不能直接把它们接入本模块并声称已可换相。

相序为 U/V/W，alpha 与 U 同向，beta 为其正交正向；电角度theta从alpha向beta为正。
所有角度为电角度 rad，电流 A、电压 V，占空比无量纲。假设三相电流和为零，
`i_w = -i_u-i_v`，采用幅值不变 Clarke 与 Park：

```text
i_alpha = i_u
i_beta = (i_u + 2*i_v)/sqrt(3)
i_d = i_alpha*cos(theta) + i_beta*sin(theta)
i_q = -i_alpha*sin(theta) + i_beta*cos(theta)
```

电压先整体等比限制，`m=modulation_limit`、`U=bus_voltage_V`：

```text
R_max = m*U/sqrt(3),  0<m<=1, U>0
r = hypot(target_voltage_d_V, target_voltage_q_V)
若 r>R_max，则 (v_d,v_q)=(target_d,target_q)*(R_max/r)；否则保持
v_alpha = v_d*cos(theta) - v_q*sin(theta)
v_beta  = v_d*sin(theta) + v_q*cos(theta)
p_u = v_alpha
p_v = -v_alpha/2 + sqrt(3)*v_beta/2
p_w = -v_alpha/2 - sqrt(3)*v_beta/2
offset = (max(p_u,p_v,p_w)+min(p_u,p_v,p_w))/2
duty_x = 0.5 + (p_x-offset)/U
```

减中点只加入公共分量，不改变相间线电压；`max(p)-min(p)<=sqrt(3)*r<=m*U`，
所以线性区占空比在[0,1]，最大与最小占空比之和为1；零矢量三路均为0.5。
`limited_voltage_d_V/q_V` 是本数学模型的限幅结果，不是实测端电压；占空比转为float会有量化误差。
没有力矩常数、物理负载或板端测量结论。

## 接口、状态与错误行为

接口见 [foc_math.h](../control_core/foc_math.h)。只有 `foc_current_t`、`foc_modulation_t` 两个实际使用类型。
参数按值传入；调用者独占输出对象，函数不保留输入/输出指针，不拥有设备或可变全局状态，
不同对象可以交错调用。只依赖标准 C 头与数学库，禁止 fast-math 绕过非有限检查。

- 任一 NaN/Inf、母线非正、调制限额不在(0,1]或空输出指针返回false。
- 非空输出在入口清零，失败保持全部数值0、标志false、`is_valid=false`；NULL时无法清零，只返回false。
- 有限电流变换后若d/q超过float可表示范围，同样失败，不返回Inf。
- 有效电角度接受任意有限float；按所传数值求三角函数。大角度的输入量化不能还原已丢失的相位精度。
- 内部用double避免有限float输入的平方/加法溢出；公开输出仍为float。MCU成本未测，不声称已适合高频中断。
- 连续公式不做逐相饱和；仅将距0/1不超过 `8*DBL_EPSILON` 的中间舍入误差修回端点。
  超出这个纯数值边界则失败，不通过大范围占空比裁剪掩盖算法问题。
- `is_valid=true` 与函数返回true才表示数学结果可用；失败时的全零数值**不等于硬件关断命令**。

## 实施前固定的验收矩阵与误差预算

数值预算基于float发布精度预先固定，不能看到输出后放宽：电流绝对误差
`8*FLT_EPSILON*max(1,|i_u|+|i_v|+|i_w|)`；占空比 `4*FLT_EPSILON`；
由占空比反算的线电压/dq为 `16*FLT_EPSILON*U`，范数上界/方向按同一量化预算；
公开float电压分量额外允许一个最小非零float台阶，覆盖次正规数舍入。
平方幅值不变量按 `32*FLT_EPSILON*max(1,i_u^2+i_v^2+i_w^2)` 预算验证。
超小/极大输入另查有限性、范围和错误确定性，不借固定1 V绝对容差证明这些极端工况精度。

| 验收/契约 | 独立 oracle/不变量 | 工况与实际方式 |
|---|---|---|
| A01 无设备/共享状态 | 主机单独编译，实际依赖仅core与标准库；目标符号无可写静态数据 | 两对象交错调用，重复结果一致；依赖文件/nm扫描；不链接HAL或M1对象 |
| A02 坐标与幅值 | 独立double三相余弦/正弦投影；手算零角/pi/2；平方幅值和逆向重建 | 正负角、U/V/W平衡正弦、相序、零电流、有限范围边界；不调用待测函数生成expected |
| A03 调制/限制 | 独立极坐标矢量→三相线电压；占空比反算去公共量；圆形上界与缩放方向 | 零矢量、六扇区边界两侧、0/90度已知值、母线/限额变化、超限、NaN/Inf/非法参数、极端有限值 |
| A13 来源与差异 | 上述准确例程快照、逐函数对应、公式、作者/厂家分别的许可边界 | 人工核对调用链/来源，不复制作者源码；原例程只作结构/未限幅已知向量对照，不是唯一oracle |
| 文档/范围 | 链接、UTF-8/行尾、diff和M1文件树不变 | 真实主机命令与完整SHA记录到PR；板端/闭环/硬件项不纳入软件通过结论 |

未执行旧例程作为主机程序：原入口带硬件写入且许可仍缺，本批不为运行它复制控制实现。
在theta=0、q=0、目标d电压为 `U/(2*sqrt(3))` 时，独立公式给出占空比约
`(0.71650635,0.28349365,0.28349365)`，可与设计已推导的未裁剪参考向量对应；
轴向裁剪与联合矢量限制在超限区的不同是有意行为改变，不能要求输出全域相同。

## 运行与证据

从仓库根目录运行（Python≥3.9、主机GCC可用）：

```powershell
python tools/test_foc_math.py --gcc 'D:\mingw64\bin\gcc.exe'
```

上述入口现已建立；执行后保留 `build/host_foc_math/` 中的编译/执行原始结果，
记录工具版本、seed、输入哈希、完整HEAD及未提交状态。首次未提交验证不能冒充干净提交结果，
提交后重新绑定证据，再交review1独立审查。失败记录保留，不能改期望值迁就实现。

六项输入包含两份core源码/头、独立C测试、测试入口、本契约和根README，均记录原字节哈希。
`test_evidence.json` 保存每步命令/返回码/输出以及运行前后HEAD和工作区状态。
`-ffast-math` 编译是预期拒绝的负向检查，其非零返回不是生产构建失败。
早期依赖诊断曾把PE/COFF的零长度 `.data/.bss` section标记误判为可写变量，已修正为同时检查
实际section大小和具名数据符号；该失败记录保留，没有放宽数学oracle或任何数值容差。

本批固件目标构建、Flash/RAM和板端运行均 NOT APPLICABLE/NOT RUN：没有改或链接M1固件，
新数学模块仅在主机验证；并不声明目标MCU实时性、电流精度、控制闭环或功率关断已通过。
