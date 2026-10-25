# M2b：双轴电流 PI 软件契约

Refs #1/#7；[设计v1.5](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6395677890)。
起始main `896ac935cf6e73c3fa94e8feb7a6a3e896d2711d` 已含M2a。开始实施时PR #10审查通过但未合并，
本批不包含其固件/CubeMX修改。当前仅合成输入和主机测试，不接M1 raw ADC、runtime或PWM。
实施期间用户授权合并PR #10；交审基线随后同步到main
`61b3fd9bd2c268cc8306210bd4b792c55359c6e2`，M2b对该基线的固件树无修改。

## 来源与实际复用

参考黄铮（best_pureer）[进阶例程](https://gitee.com/best_pureer/stm32_foc_2)，快照
`672e32f1005d9019140d1a9c754fb8ca2e8eedde`；不复制许可未明的作者控制源码。

| 真实参考入口 | 本批对应与分类 | 保留/改变/未引入 |
|---|---|---|
| `Core/Src/adc.c`：Clarke/Park→双torque_loop→foc_forward | `current_control_step`参考组合顺序 | 输入为显式已校准A、电角度rad、dt秒；不复制ADC USER CODE、全局控制指针或板端换算 |
| `foc.c`：torque_d_loop/torque_q_loop | 新模块中两个独立积分量，参考双轴与reset需求 | 原增量PID输出逐轴裁剪±1并修改state[2]；改为有单位的位置式PI和最终联合电压限幅反馈 |
| `set_motor_pid`/motor_mode_torque | 显式参数按值传入、init/reset | 不导入默认增益/电流、不用模式全局变量；reset仅清积分，未设计无冲击模式切换 |
| `arm_pid_f32`/init/reset | 核对公式和状态，不新增依赖 | 参考包CMSIS-DSP为Apache-2.0；不复制或修改内部state来接最终限幅，标准C独立按本契约公式实现 |
| 本仓库M2a `foc_transform_current` | **直接调用现有已审函数** | 复用相序和q轴符号，不重写坐标变换 |
| M2a `foc_modulate_voltage` | **直接调用现有已审函数** | 复用联合圆限幅/逆Park/SVPWM；它返回的最终受限模型电压用于积分跟踪 |
| 例程conf.h、查表、对齐/硬件写入 | 未引入 | 无实际校准或板端证据，不迁移默认参数，不把占空比当作硬件使能 |

[Arm PID说明](https://arm-software.github.io/CMSIS-DSP/main/group__PID.html)定义增量递推
`y[n]=y[n-1]+A0*x[n]+A1*x[n-1]+A2*x[n-2]`及独立状态；它没有本项目的最终矢量限幅反馈参数。
未运行或复制原硬件回调作oracle。M2a的原公式/许可边界见 [M2a契约](m2a_foc_contract.md)。

## 接口、单位与数值规则（实施前固定）

新增 [current_control.h](../control_core/current_control.h)、[实现](../control_core/current_control.c)、
[独立测试](../tests/control_core/test_current_control.c) 和 [主机入口](../tools/test_current_control.py)。
状态由调用者持有，只有d/q两个积分电压float；init/reset均清零，NULL返回false。
配置、每拍输入按值传入，不保留指针；状态和输出须为独占、不重叠的有效对象，同一状态不并发写。
参数不带电机默认值，两个轴分别提供有限非负kp(V/A)、ki(V/(A·s))、tracking_gain(1/s)，允许0。
共同的reference_limit_A有限且>0；分别对d/q目标裁剪到[-limit,+limit]，**不是电流矢量圆限幅**。
modulation_limit在(0,1]；minimum_dt_s、maximum_dt_s有限，0<minimum<=maximum，dt在闭区间内。
不附加隐藏增益/积分夹限，也不把合法配置等同于已证明控制稳定；参数整定不属于本批。

输入为已校准U/V相电流A，假设三相和为0；电角度rad、d/q目标A、正母线V、dt秒。
测得d/q由M2a返回float，参考裁剪也以float表示。两轴均按下式先计算本拍电压再更新下拍积分：

```text
e = reference_limited - measured_current
requested = float(kp*e + integral)
limited/duty = foc_modulate_voltage(requested_d, requested_q, angle, bus, modulation_limit)
next_integral = float(integral + ki*dt*e + tracking_gain*dt*(limited-requested))
```

除明确float发布点外，PI中间运算用double。tracking使用**实际传入M2a的float请求**与其返回的
float最终受限电压之差，两轴同一次联合限幅，无逐轴电压预裁剪。
requested或next_integral非有限/超出float范围即失败；次正规数按工具链正常float舍入。
所有配置/输入/现有积分检查、变换、调制和两个next_integral都成功后，才一起提交两轴状态和输出。
任一步失败：返回false，非空输出所有数值0、全部标志false，原状态逐字节不变；NULL输出也不更新状态。
无隐式恢复/使能；调用者须锁存故障、处理原因并显式reset。失败值0不是硬件关断命令。

## 验收与独立oracle

| 项 | 独立依据与工况 | 验证方式 |
|---|---|---|
| A01 | 两实例交错与单独轨迹一致；无可变全局/设备依赖；真实调用M2a | 编译依赖、nm/section、未定义符号必须引用两M2a入口、交错测试 |
| A02/A03 | 三相旋转投影的double电流oracle；极坐标圆界、相间线电压与占空比重建 | 多角/母线/限额网格，d/q符号与限幅标志；不调用待测函数生成expected |
| A04递推 | 手算零误差/积分一拍延迟、限流、双轴联合限幅；独立double离散序列 | 进入/保持/退出饱和、反向误差、不同dt、reset、两轴不同增益及非零初始状态 |
| A04失败 | 每项配置/输入/状态NaN/Inf与边界，变换超范围、请求溢出、仅第二轴积分溢出 | 确定全零无效输出、状态memcmp不变；reset后恢复有效数值，没有硬件使能 |
| A13/回归 | 上述逐函数来源与变化，M2a源码/头/测试不改，固件树不改 | M2a主机回归、差异/链接/来源人工核对、提交后再绑定完整SHA |

预先固定预算：电流`8*FLT_EPSILON*max(1,|iu|+|iv|+|iw|)`；由电流舍入传播到PI的误差按
对应kp/ki/dt显式传播。每拍独立oracle从当前存储的float积分出发，比较后再进入下一拍，
避免将无限长轨迹误差藏在固定绝对容差内。请求/积分另允许`8*FLT_EPSILON*max(1,各加项绝对值和)`；
积分检查以独立方程计算并按接口舍入的float电压差为输入。占空比`8*FLT_EPSILON`；线电压重建预算`32*FLT_EPSILON*bus`。
另有零角手算序列直接检查整数/二进制分数，及长期饱和稳态/退出不变量；不靠待测输出反算expected。

从仓库根运行：

```powershell
python tools/test_current_control.py --gcc 'D:\mingw64\bin\gcc.exe'
python tools/test_foc_math.py --gcc 'D:\mingw64\bin\gcc.exe'
```

证据只写忽略build目录，记录完整head、运行前后状态、所有输入哈希、工具版本、命令与原始输出，
保存失败记录。fast-math预期编译拒绝；数值测试使用真实core，既有M2a模块直接链接、不mock。
本批固件构建/Flash/RAM NOT APPLICABLE（模块未链接固件）；MCU实时预算与硬件 NOT RUN。
不声明电流闭环实物可用、真实电流精度、功率关断或整个Issue #1完成。本批交独立Review后停止。

## Work软件验证记录

首轮在未提交工作区验证，随后基线同步至上述新main，重新执行同一入口。
GCC8.1.0/x86_64-posix-sjlj、C11/O2、-Wall/-Wextra/-Werror/-pedantic、禁用fast-math：
7,312个独立step工况、75个非法输入/回滚工况通过，另含NULL接口、reset和1000轮双实例交错。
先分别跑完整轨迹再交错重放，逐拍状态一致；极大合法有限配置和请求也有成功用例，未一律拒绝。
两个真实core编译对象无可写静态/全局存储，current_control对象真实引用两个M2a入口。

M2a回归：7,210个独立电流投影、415,317个极坐标/线电压工况，以及边界、非法/极值和隔离检查通过。
两套正常编译无警告；各自fast-math负向编译按预期失败，不当作正常构建失败。
提交后再次运行两套入口；完整head、干净状态和九项M2b/六项M2a输入哈希分别保存在
`build/host_current_control/test_evidence.json` 与 `build/host_foc_math/test_evidence.json`，
最终提交和独立审查状态以PR记录为准，旧PR9/#10的PASS不自动继承到本批。
