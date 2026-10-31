# 受限d/q电流闭环工作包

来源：用户在Design对话明确要求设计并推进d/q电流闭环；Design于2026-10-31一次性交Work。
本批从6264f86690b6426704986668a9166b3357fb42fa派生，未把PR25/旧依赖当作main。
远端检查先后遇Windows TLS凭据错误和连接重置，最新PR状态尚未核实；不阻断本地软件实现，不自动合并。
用户Keil下载/调试设置保留，只追加三个真实调用源；正式工程不要求选择另一个HEX。

## 阶段成果与停止点

真实完整原始帧→名义U/V电流→Clarke/Park→双轴PI/联合限幅→逆变换/SVPWM→同一自然UEV三相更新。
软件检查及独立Review后交付零指令版本和小参考候选计划。未安排下载/12V/PA12，不操作硬件。
本包不增加速度/位置环、Flash参数存储、串口运动命令或对齐自动串联；小电流跟踪板验前不冻结P1。

| 验收项 | 契约/独立依据 | 具体不变量与验证 |
|---|---|---|
| 数学与PI | 已审M2a/M2b几何、PI方程 | 复用canonical核心，原核心回归；新四角度真实路径与独立RL对象 |
| 参数 | 商家R5.5Ω、Ld0.72/Lq0.9mH及精确分段指数积分 | R±30%、L0.504..1.17mH、母线±15%、测量增益±20%、正负参考/延迟740..810us共648组 |
| 采样窗口 | 生成TIM1/TIM2/ADC配置、PWM1计数几何 | duty只在848..952/1800，ADC孔径和转换均落共同低窗；越界拒绝不裁tick |
| 同步提交 | CCR预装载/自然UEV及受审生命周期 | 代次/seq/最新帧、迟到/STOP/Break/截止/邮箱积压测试，实际硬件边沿NOT RUN |
| 故障与积分 | runtime唯一故障/资格所有者 | STOP/故障/正常结束撤销pending与PI；仅硬件提交确认才采纳临时积分 |
| 实时与目标 | 实际Keil目标、有限前台事务 | 数学计算放IRQ外；准备阶段32次核心计时及每次实际控制计算200us墙钟门槛；MCU观测尚NOT RUN |
| 数据 | 固定250ms实验/完整瞬态 | 256×28=7168B RAM，停机后CRC导出；丢帧/错CRC/缺段拒绝，不只看末值 |

## 候选参数与物理量

电零位来源为用户31-17-27 SINGLE_U成功帧：保持位置7453count，p=7、方向+1条件下
z=5.125413307522898rad；theta_e=wrap(7*raw*2pi/16384+z)。旧337°/276°/七点FAIL保留，不能混用。
该参数仅在本实验RAM控制状态使用，来源采样cfg00030001。闭环协议cfg00040001，外设配置不变。
没有再次自动对齐或Flash写入。单次保持不证明p/d/完整相序，电流跟踪也不能独立证明电零位正确：
例如15°电角偏置时，估计dq=(0,40mA)仍可跟踪，而真实磁链轴含约10.35mA的d分量。
本包任何结果仍calibration_verified=false；闭环异常停止，不能以控制器自洽宣称绝对标定。

每启动保持12V/静止条件采512帧零偏，资格沿用已有静止/噪声/连续性检查。
名义A=(raw-sum/512)*3.3/(4096*0.02*50)，U/V符号均+1由此前独立U/V响应支持；
旧2kHz实验不能证明新波形模拟建立或绝对增益。没有母线ADC，12V只是名义值。
三相和为0推算W；Clarke/Park沿canonical M2a符号，旧stride10速度反馈不作为电流环输入。
测角SSI比ADC完成早约52us，不外推未验证速度；运动包络及已有原始有效性仍约束输入。

## 调制与采样窗口

保留PSC0/ARR1800/CMS3/RCR39，fPWM=72MHz/(2*1800)=20kHz；UEV周期=1800*40/72MHz=1ms。
TIM2 PSC71、SSI175us、ADC225us。ADC 12MHz、采样7.5cycle=0.625us、总20cycle≈1.667us。
Vdq圆界0.4V，M=0.4*sqrt(3)/12≈0.057735；连续SVPWM围绕0.5，最大偏移sqrt(3)*0.4/(2*12)。
round后的CCR只在848..952，因此最迟下降沿952/72≈13.222us，上升沿50-13.222us。
以峰部25us、相位读取不确定性±2us计，最早孔径23us前至少9.778us相线保持低电平；
最晚转换结束28.667us仍早于36.778us上升沿。新调制绝不沿用任意占空比都有效的假设。

[TI INA199资料](https://www.ti.com/lit/ds/symlink/ina199.pdf)给x1带宽典型80kHz；
若仅作一阶近似，1%建立约ln100/(2pi*80kHz)=9.16us。
这是典型小信号估计，不能作为开关共模跳变、外部RC、器件负载/温度的最大建立时间保证。
此处只证明几何余量，板端模拟建立、死区/真实峰值/外部电流精度仍未测。窗口不足立即拒绝，无自动扩大调制。

## 离散PI与有效提交

1kHz电流环，Kp=0.5V/A，Ki=150V/(A*s)，tracking=200/s；每轴参考上限40mA，圆界0.4V。
R/L电气时间常数约0.13..0.16ms，因此不套高带宽连续公式；以分段精确RL指数积分建模。
从ADC测量到下一UEV约773us：本拍先沿用上一电压，下一UEV后才施加新电压，显式零阶保持。
独立模型扫描648组，在180..225ms平台最大测量跟踪误差小于6mA，实际电流峰值小于65mA。
模型不是锁转/自由转子的力学保证，也不覆盖开关尖峰；母线/增益误差意味着实际安培值并未校准。
模型700ms零参考恢复只作算法分析，正式250ms动作会停PWM，不把模型延长段冒充板端观测。

前台取单槽完整原始帧，首次dt显式1ms，后续dt必须990..1010us且seq连续；处理开始年龄≤250us，
计算墙钟≤200us，提交年龄≤450us；旧计算不得覆盖更新输入。任一超时/积压锁存停止，不放宽新鲜度。
浮点/三角函数均在前台、IRQ开放；32角度/正常与饱和核心预运行只做无输出计时，超过200us不READY。
这不是WCET证明，实际ARM时间和IRQ延迟要由新固件板上记录；执行时仍逐次检查，慢则停止。

board独占CCR/MOE。按键请求先装有效0，再首个UEV开启零占空比、预装三路900，第二UEV施加50%零电压矢量。
活动期间只在UEV后225..500us内接纳提交：读配置和当前帧/代次后，PRIMASK下三个连续APB写，
留给下一UEV至少500us；无等待/三角/格式化/UG/UDIS/逐相HAL_Start。写后≤20us且无新UIF，否则清MOE并锁存。
此约束依赖已验证的1ms硬件周期及短寄存器事务，不覆盖无限NMI/总线停顿；目标时序仍须观测。
硬件下一UEV统一转移预装值，board确认包代次/seq/时标，PI才采纳临时新积分。
不会在board再次裁tick或窗口饱和：越界/迟到拒绝并reset，而非按未施加电压继续积分；占空比量化误差约1tick。

## 固定刺激与停止

默认Keil为DQ_ZERO，id_ref=iq_ref=0；Reset只准备、PA12新按下释放一次才闭环，不能从旧对齐结果自动进入。
可编译的DQ_POS40/NEG40是候选实验，不是运行授权：id=0，iq在25ms上升至±40mA，保持至225ms，再下降至250ms。
最后一拍可能仍有电流，250ms后清MOE/CCER并接管GPIO低，不能说闭环已在零指令下完全恢复。
正式独立期限270ms包含启动与观察，正常250ms由原始样本检查结束；丢采样由独立TIM3新鲜度期限停止。
原始U/V/推算W任一绝对偏差>128count（名义约103mA）或端点立即停；测得|id|/|iq|>80mA停。
100ms后|iq_ref-iq|>40mA拒绝（发现反号/明显失跟），并非最终跟踪验收门限；离线平台判据更严格。
累计运动256count约5.625°即撤销，离散检测不保证机械绝不超出该角。普通12V适配器无可设限流。
自由转子的小iq可能先触发运动/反电势限制，不能为了跑满250ms扩大包络/电压；可能只能取得暂态证据。
0A指令会保持PWM，停止必须board_disable_output；MS8313固定高EN的高阻缺口仍未解决。

## 停机后数据与最少板验计划（尚未执行）

P1D0固定14个8位hex字段：cfg/ref_mA/count/max_compute_us/max_latency_us/bench_count/bench_failed/start_us/
sum_u/sum_v/zero_count/pole_pairs/fault/action_reason，随后CRC4/CRLF；负值按int32补码。
P1D1固定12字段：index/seq/ADC_complete_us/iq_ref_mA/id_mA/iq_mA/vd_mV/vq_mV/compute_us/latency_us/limited/raw_angle，CRC同CCITT-FALSE。
CRC覆盖末CRC之前含最后逗号；每个包≤160B。仅停止后导出，无IRQ打印、不扩5秒M3B3。
`dq_report.py <VOFA导出文件>`必须完整块/CRC/seq/dt/时间包络/配置/停止原因通过，才给平台误差和峰值。
头中的最大时间是墙钟观测，主机夹具为0不能当MCU时间；传感名义mA不代表独立电流参考。

软件Review通过后由用户明确安排：12V断开、原Keil Rebuild/DAP下载，接12V并Reset；
确认FW新版本DQ_ZERO、P1D0基准32/failed0、READY后，只按PA12一次。预期不持续转动，约250ms结束，
P1R2无故障/正常结束、ACTION_DONE，自动导出约250个P1D1；保留文件交工具分析，不必手动发送查询。
先完成零参考后，再展示并确认具体的单方向小参考幅度/时长/运动范围；不自动烧录正负两种或连跑。
有异常、FAILED或明显运动即停止/断12V，保留失败块，不反复触发。无示波器/独立电流基准的限制继续保留。


## 交付栈空间修复（Work自查）

首次软件Review后，实际ARM链接callgraph发现main→current_loop→PI→sin调用链最大736B；
ADC IRQ完整故障路径272B，加Cortex-M3异常入栈32B即1040B，原厂startup预留1KiB不足以覆盖这一保守组合。
因此不按原1KiB布局交付。保持厂商startup原始字节/47项hash不变，在scatter明确把原STACK置于
0x20004C00..0x20005000，连续下方0x20004800..0x20004C00额外保留1KiB EMPTY；
实际栈顶仍是20KiB SRAM末端__initial_sp=0x20005000，总向下保留2KiB，普通RW/ZI不得进入保留区。
当前正式工程固定microlib，不使用非microlib的__user_initial_stackheap/Stack_Mem下界；改库类型时必须复核布局。

静态栈分析同时列出USART(priority3)128B、TIM2(priority2)88B、DMA(priority1)128B、
ADC(priority0)272B；加主链736B和四个32B异常帧为1480B，另计SysTick/对齐/停机异常余量，仍小于2KiB。
这是对启用路径的保守组合，不宣称静态工具消除了全部间接调用/异常风险，也不冒充实测栈峰值。
通过实际armlink布局/符号/向量首字检查无堆/普通ZI与栈重叠，目标构建和独立增量复审后交付。


## 与本地例程的电流环频率对照

已实际读取D:/FOC_Projects/stm32_foc与stm32_foc_2的conf.h、tim.c、adc.c和foc.c。
基础例程PSC0/ARR1800/CMS3/RCR4：载波20kHz、ADC/电流控制8kHz；进阶ARR1799/RCR3：
载波约20.011kHz，ADC/电流控制约10.006kHz，宏motor_adc_freq按名义10kHz定义。
进阶ADC完成IRQ中直接进行Clarke/Park、两个CMSIS PID及逆变换/SVPWM，使用float与11bit三角表。
本项目1kHz是已建立采样基线下的低带宽验证阶段，不宣称电机的最佳最终频率；
其Ki显式乘dt且PI输出单位V，例程归一化PID参数不能直接复制。提高频率需同时重排SSI/ADC时刻、
CPU预算、提交延迟与PI离散模型，不只修改RCR数字；本批不自动扩大该范围。


前台收尾修复：1kHz电流包在较低频feedback_step之前处理，避免其曾观测约132us计算占用PWM提交余量。
主机直接编译main.c的USER3实际调用块，注入低优先任务300us耗时，新顺序先提交成功；
旧ca5bf7a实际main块在同夹具因开始年龄>250us拒绝，作为调度回归负对照。不放宽任何年龄/计算/相位门限。


## 21:45无输出实测与计算优化

用户FW31-19-28准备阶段P1D0 CRC有效：max_compute=0x14D=333us、bench_count0、failed1，
因此未READY/未启动输出。P1Z1 cn512/fl1D说明零偏批本身完整，r04来自组合资格拒绝；
原FAILED_ZERO_INVALID误导原因，改为FAILED_CONTROL_TIMING独立提示。
原始两帧/解析留在build/dq_current/user_timing_failure_20261031_2145，不删除此次失败。
333us是含IRQ墙钟观测，不把它全部归为某一个数学函数，也不是物理电机故障证据。

有界实验改用current_control_step_bounded：相同Clarke/Park、PI、圆限幅、逆变换/SVPWM和抗饱和方程，
采用float、共用一次sin/cos、未饱和时比较模平方避免求根、三相除法共用母线倒数。
旧通用double入口保持原行为，作为独立oracle；不启用编译器fast-math，不放宽200us/采样/提交门槛。
新的入口范围写在current_control.h，范围外和下一积分超±2V直接拒绝，输出无效且两轴状态不部分提交。
目标配置和启动正常/饱和基准均在支持范围；PI仍只在真实UEV提交确认后采纳临时状态。
主机289200组对照的电流3uA、duty2e-6、电压5uV、积分10uV误差门槛，
另有真实板级0/±40mA、角度/故障/迟到/调度回归。单精度数值误差不是实际电流测量精度。
优化后实际MCU耗时仍须重新无输出基准验证；不给未经实测的提速数字，不直接按333us提高门槛。


## 22:00第一次零参考动作与首因记录修复

用户摘录未含FW1行（预期31-21-52）；P1D0/P1R2 CRC有效，bench32/failed0、max_compute170us、
sample-to-apply最大773us，4个控制包已施加，动作开始后4241us以f0x20提前撤销；没有达到250ms验收。
f0x20共用于原始U/V/W、ADC端点/序号、运动以及dq诊断，现有摘录不能确定具体子原因，不能推断硬件坏或改变极性。

重复头帧是确认的软件问题：每个FAULT ADC包再次latch_fault→stop→current_loop_abort，
无条件header_pending=true导致每1ms重新请求头，UART每约12ms只能发送P1D0，四点记录永远饥饿。
改为只有真实动作代次/待包首次撤销才通知；后续故障采样保留锁存/禁PWM，不反复抢占导出。
真实1kHz故障后采样回归确认P1D0一次、P1E1一次、四个P1D1均发出；原dc52c70的abort函数
在同一新夹具负对照因重复P1D0失败，保留原始函数来源/哈希和失败断言。

增加仅首次保护后的P1E1单帧，不改P1D0/P1D1布局或正常日志频率：7个8位hex字段
reason/seq/ADC_complete_us/raw_u/raw_v/raw_angle/detail，末CRC4/CRLF；按已有CCITT-FALSE覆盖末CRC前逗号。
reason1U/2V/3W偏差，detail为count×512（W取负U-V）；4ADC端点、5序号异常，detail0；
6累计位移，detail机械count；7id/8iq/9q跟踪，detail名义mA。带符号detail用int32补码。
先board_disable_output再冻结首因，下一步仍由runtime锁存原故障，后续数据不能覆盖首因。
输出顺序头→首因→有限样本；新增数据只用于判定已有保护哪项触发，不增加运动或电流命令。
所有128count/80mA/256count/250ms/200us及同步PWM门槛保持；真实板端首因仍待新版本一次受控采集。


## 22:21有效首次保护与用户确认的隔离诊断

仅使用FW1,31-22-09,DQ_ZERO后的新会话。此前重复/拼接/字段损坏文字不修补、不纳入验收。
新P1D0/P1R2/P1E1及2条P1D1均CRC通过：基准32、max171us、实际提交延迟769/772us，
参考0A，程序记录输出2240us即停止。首因3为推算W原始偏差；U/V零偏2026.046875/2025.76171875，
触发原始值2089/2091，名义U+50.719/V+52.560/W-103.279mA。W比103.125mA诊断线超0.154mA约0.15%。
这不是硬件额定/峰值安全极限。相同原始数据按当前角度换算id约-97.214mA，也超过独立80mA诊断；
所以仅提高W门限不能修复本次零参考失跟。两条观测id-24/-76mA，q+10/+24mA；三个角度均6805。
记录完整导出，重复通知修复在该新会话有效。原帧及计算保留build/dq_current/user_w_trip_20261031_2221。

复核原理图既有审计：R5/R6串在OUT1/U、OUT2/V相线上，不是PGND低侧电阻。
前文的“低侧采样”用词不准确，已改为相线采样；固定低电平区只证明无新PWM边沿的几何余量，
并不能保证INA199共模跳变后的真实建立和动态零偏。20kHz下静态GPIO低零偏与三相50%开关基线未独立比较。

用户明确同意先准备PI不参与输出、三相相同50% PWM、最长20ms的诊断版，实际运行另行安排。
ARM默认DQ_VEC0，cfg00040002（与闭环40001区分）；Reset只准备，PA12仍一次手势。
沿用两自然UEV启动，随后CCR1/2/3固定900，不进行动态CCR提交。
为复用已验证变换/计时，前台仍从零临时状态算现有核心，但电压和积分结果一律丢弃，只记录电流坐标；
积分保持0，记录vd/vq=0、latency0表示没有新控制包生效，不能解读为零延迟电流闭环。
按键释放起20ms绝对截止由既有独立TIM3处理，包含约1..3ms启动等待，实际输出短于20ms；
正常诊断提示VECTOR_DIAG_DONE，不写ACTION_DONE作为闭环通过。15..20条观测用于比较采样基线。
保留128count/80mA/256count、200us、STOP/BKIN/丢帧保护；达到任一仍停止并冻结P1E1，不屏蔽启动瞬态。
三相相同占空比理想线电压0，不代表实物输出/峰值电流已获独立证明。

若不让PI修改PWM也出现相似偏移，说明PI并非唯一原因，继续查开关测量基线/模拟建立/实际驱动输出；
若固定矢量稳定而闭环才增大，再查反馈符号、通道映射和有效PWM。不能仅据此翻转符号或提高门限。
本对照仅一次短采集，结果仍diagnostic_only/accepted=false，不替代闭环跟踪验收，不重开对齐工具支线。


安全量级口径：103.125mA是本次小电流诊断线，不是电机/驱动额定极限；超0.154mA还小于1个ADC计数约0.806mA。
但当前角度下同一数据的id约-97.214mA，较独立80mA诊断超17.214mA，并超过计划±40mA参考的量级。
没有外部峰值/有效值/温升证据，不能拿名义103mA与商家1.5A直接相减声称还有1.397A安全裕量。
此诊断针对零参考增长与测量/PWM反馈链路，不以0.15%边界越线推断硬件危险或追求绝对电流标定。


诊断期间不重采/应用带PWM零偏：沿用本启动GPIO低时取得的512帧基线，输出期间数据只记录。
即使固定相同PWM下出现偏移，也不能未经区分就把它扣成零，否则可能掩盖真实相电流或驱动不对称。


## 固定零矢量诊断的原始记录帧（新增，板端NOT RUN）

在既有 P1D0/P1E1/P1D1 之后新增一段有序原始记录块，用于区分配置、引脚、采样时序与实际输出。
新增帧仍是 `<前缀>,<8位hex>…,<CRC4>\r\n`，CRC 为 CCITT-FALSE 覆盖末 CRC 之前含最后逗号；
最长帧为 `P1T1` 的 137B，发送缓冲 168B，低于 `BOARD_SERIAL_BUFFER_BYTES=256`。

| 帧 | 字段（除前缀/CRC） | 用途 |
|---|---|---|
| `P1T0` | version, pre, body, total, complete, overflow, frozen, trip, first, freeze_reason, freeze_sequence, configs, generation | 头帧：分段条数与完成/溢出标志 |
| `P1T2` | stage, generation, read_us, cr1, cr2, ccmr1, ccmr2, ccer, bdtr | 阶段控制寄存器快照 |
| `P1T3` | stage, read_us, psc, arr, rcr, ccr1, ccr2, ccr3 | 载波与比较值（CCR是预装值） |
| `P1T4` | stage, read_us, gpioa_crh, afio_mapr, gpioa_idr, moe, ccr_is_preload | 复用/引脚/MOE |
| `P1T1` | index, sequence, completed, before, after, raw_u, raw_v, angle, tim1, tim2, pins, flags, dir, stage | 逐条ADC完成时原始值与时序快照 |
| `P1T5` | actual, total, pre, body, complete, overflow | 末帧：实际导出条数 |

顺序固定为 头帧 → 每个阶段 `P1T2,P1T3,P1T4` → `P1T1` 按 index 递增 → 末帧；缺帧、重复、错序或
CRC 失败都由 `tools/dq_report.py:trace_reports` 拒绝，不自动补齐。

`flags` 位：1=Runtime值域有效、2=边沿不确定（TIM1方向读前读后不同或读取跨度>2us）、
4=动作前样本、8=停机后恢复样本。`pins` 低3位是同一次 IDR 读数里的 PA8/9/10 电平，bit3 为 MOE；
它只表示该瞬间逻辑电平，不是 ADC 采样孔径时刻，也不证明功率端同步。阶段 `STOPPED` 的样本
在解析器里要求三路引脚低且 MOE=0。

阶段 `IDLE` 在 `board_start_sampling()` 之前记录，此时 `CR1.CEN=0` 是正确事实，解析器据此区分
“计数器未启动”与“动作中”；只有 `ZERO`/`FIFTY` 才要求 CEN=1。停机阶段由“本代次真正开过 MOE”
的软件标志触发，Break 硬件清 MOE 时同样留下 `STOPPED` 快照。

恢复窗口固定8条1kHz样本，等待上限12ms；采样已停止时按实际条数导出并置 `complete=0`，
不会无限等待，也不会因此延长输出或重开 MOE。记录本身不参与任何保护判据。
