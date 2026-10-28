# P1a-2 一次性 PWM 门控实施设计

Refs [Issue #1](https://github.com/GNYKCZ/motor-control/issues/1)。基线为 PR #20 合并后的 main
`45ebbbda89b97aaccdac2c11d6cd29d9a2c4ff73`（tree `8c3d7b5ca83934d215113ccdf2e8cc815e50c719`）。
本文细化 [P1a 总体设计](p1a_output_gate_design.md)，涉及具体提交顺序时以本文为准。
本批只交付文档，没有修改固件、生成配置、构建入口或用户 Keil 设置，没有打开串口或操作板卡。

## 目标、范围与当前结论

目标是为“显式建立资格、消耗一次资格、执行一个固定 profile、及时停止”给出可实现的状态事务、
板级寄存器顺序和反例验证计划。先完成默认没有可输出 profile 的软件准备，再满足现场条件后另行批准非零输出。
软件编译通过、门控模型通过和 P1a-2 板端验收分别记录；本文不代表其中任何硬件项目已通过。

本批不迁入 FOC、电流/速度/位置环、自动对齐、例程默认电流、任意占空比串口参数或按键触发。
不新增硬件，不修改主 Issue 的阶段次序。无可靠会话防重放方案、无已批准 profile 时，ARM/PULSE 继续拒绝。
用户尚未选定防重放策略；这不阻止完成下述板级门控和软件反例设计，但阻止开放首次非零输出。

## 来源与实际调用路径

| 来源 | 本次核对到的事实与影响 |
|---|---|
| `Core/Src/main.c` | `motor_runtime_init → command_init → 生成初始化 → board_initialize_devices → finish_boot → board_start_sampling`；前台先检查采样期限，再消费反馈，最后一步命令或日志。PWM 准备不能绕过这一启动握手 |
| `Core/Src/tim.c` / `.ioc` | TIM1 仅 Base/内部时钟、中心对齐3、PSC0/ARR1799/RCR3、TRGO UPDATE；没有 PWM 通道。TIM2 ITR0 reset、1MHz、CCR1=50；TIM4 1MHz/16bit 设备时基；TIM3 未占用 |
| `board/board_io.c` | TIM1 更新触发双 ADC；TIM2 +50us 启动24bit SSI/DMA，RX TC 后整体发布，后续 ADC 才消费完整编码器对象。PA8/9/10 当前 GPIO 推挽低，MOE/CCER 被清零 |
| `app/motor_runtime.c` | `handle_sample` 每包开头无条件禁用输出；多处故障分支依赖这次禁用，部分直接写 state/fault。反馈入队/计算/采样失联检查仅覆盖 DISABLED；增加 ACTIVE 不能只删除第一行禁用 |
| `board/board_uart.c → app/command.c` | USART2 SR→DR，逐字节 CRC；完整合法 STOP 在 RX IRQ 直接执行 `motor_runtime_stop`，先停后排回执；当前 ARM/PULSE 均 DENIED。RX错误/解析超时当前只丢帧、排回执，不停机 |
| `app/telemetry.c` / `tools/sensor_capture.py` | M3B3 与 P1R1 都只定义 state0..2；解析器明确拒绝更大 state。不能不升级协议就塞入新生命周期或冒用 DISABLED |
| 本地 `stm32_foc` | 无 Git 身份，按原文件哈希冻结参考；其 TIM1 ARR公式、RCR=4 与当前不同；有逐相 HAL 启动和400ms强拖，不作为本项目的时序或安全值 |
| 本地 `stm32_foc_2` | Git `672e32f1005d9019140d1a9c754fb8ca2e8eedde`；PWM1/三相输入、RCR3可作连接参考；逐相 HAL 启动、`(0.5,0,0)` 400ms 对齐、0.3A默认扭矩均不迁入；BreakState实际 DISABLE |

参考源码不复制入仓库；许可与作者说明沿用 [SOURCES](../firmware/stm32f103/SOURCES.md)。
本次参考文件的确切哈希另存本地验证证据，不能给没有 Git 的基础例程编造提交号。

### 硬件事实与未确认项

V2.0 原理图的 U4 同时标 MS8313/DRV8313PWPR，实际丝印未确认；PA8/9/10 对应 IN1/2/3，
EN1/2/3、nSLEEP、nRESET 固定高，无 MCU 断开这些信号的接口。PB12 接有效低 nFAULT。
上述是参考接线，不代替实际板卡一致性核实。

若实际为 DRV8313，EN高/IN低是低侧导通，EN低才高阻；依据
[TI 数据手册的半桥控制表](https://www.ti.com/lit/ds/symlink/drv8313.pdf)。
因此本文的 STOP 表示停止调制并把三个 IN 置低，功率桥高阻/断能继续列缺口。
实际若为 MS8313，须用准确型号资料核对极性、换向和保护，不能照搬 TI 保护参数。

用户已有普通12V适配器，无可见电流/可设上限；不是可编程限流电源。USB-only 曾报 `f=4`，
不能屏蔽 nFAULT 或清故障让 ARM 成功。现有单次 STATUS 成功回执支持命令链路观察，
开口破帧调查和有限回执不构成任意时刻 STOP 必达保证。普通串口 STOP 也不构成独立急停。

## 状态、资格与协议边界

### 两个不同保证

“复位不自动输出”由默认 profile 不可用、MOE 关闭、无启动激励、RAM资格/待请求清空保证软件行为；
它不证明复位时引脚电气波形。“旧命令不重放”还要求跨复位/重连识别旧请求。
P1C1 只有32bit序号/命令/CRC；复位后 `have_sequence=false`，积压的一对 ARM/PULSE 可重新成为新命令。
清 RAM、主机不主动重发、CRC正确或序号递增都不能单独解决这一问题。
UART也没有可靠的“主机关闭/重开COM9”事件；DTR/RTS及开口延迟不作为会话身份。

### 推荐与待决选项

| 选项 | 接口和使用影响 | 开发/验证代价及边界 |
|---|---|---|
| 推荐后续：版本化命令 + 持久启动 epoch + 短效单次 ticket | STATUS 握手读当前 epoch；ARM/PULSE 必须携带该 epoch 和同一 ticket，PULSE绑定固定profile。主机重连须先 STOP，取得新ticket；旧帧不能跨启动使用 | 需单独设计可靠Flash记录、掉电原子性、寿命、满/坏页拒绝输出、下载擦除后身份规则；禁止在ACTIVE擦写。现有scatter用满64KiB地址范围，必须另审预留页和下载影响。本批不实现 |
| 已有本地按钮确认 | 每次启动/实验要新“释放→按下→释放”序列，确认窗口只准一次profile；启动按住无效，串口不能单独建立输出资格 | 先确认 PA12 按钮实际存在/接线；生成输入/去抖并测试误触、复位、串口积压；仍须将确认后的命令绑定新ticket。改变板卡操作，用户未选择，不默认采用 |
| 仅 RAM nonce、UID、开机计时或更大seq | 主机看似有握手，接口改动较少 | UID恒定，启动时间可重复，未证明随机熵；跨复位唯一性/重放防护不成立，不接受其作为唯一依据 |

推荐的理由是保持串口操作方式并能定义跨启动身份；不是本批批准Flash方案。
持久 epoch 也不能检测一次静默重连：旧 ticket 在有效窗口内仍可能执行一次。
若验收要求“任意主机重连瞬间立即撤销旧资格”，必须另有可观察断连信号或每次本地确认；
仅 UART 字节协议无法作此保证。选定前默认拒绝 ARM/PULSE，这一缺口不能用模糊“会话已重置”掩盖。

### 拟定生命周期（后续开放输出前冻结）

保留 BOOT=0/DISABLED=1/FAULT=2，未来新版本追加 ARMED=3/STARTING=4/ACTIVE=5；
STARTING 指请求已消耗资格、等待装载/共同低电平窗口，此时 MOE 必须为0。
不另外建立控制器状态机或无限命令队列。一次只有一份请求，按值复制，不保留串口缓冲指针。

| 事件/状态 | 必须执行的事务 |
|---|---|
| BOOT → DISABLED | 初始化/采样启动成功后仍无输出；资格、待启动、期限全部无效，FAULT 不自动清除 |
| DISABLED + ARM | 仅新会话/新序号、profile已批准且与该固件绑定、采样与反馈当前有效、无故障、nFAULT高、BKIN已配置才建立一次短效ticket；不写非零输出 |
| ARMED + PULSE | 再核对所有条件，消费ticket后进入STARTING；固定三路tick和绝对期限从只读profile取得，不能由串口改幅值/时长；资格即使启动失败也不返还 |
| STARTING → ACTIVE | 仅在预装载已生效、共同低窗口及最终故障核对均通过时一次开MOE。等待过程中原绝对期限继续推进，不能挪到真正开门时延长 |
| 正常期限到/STOP | 清MOE、撤销ticket/待启动/期限；无锁存故障则DISABLED，否则FAULT；再次输出必须新ARM |
| 故障/非法输入 | 先禁用，再锁存。ARMED/STARTING/ACTIVE的UART错误、坏CRC/超长/解析超时、反馈/采样失效都撤销资格；ACTIVE不等待前台或ACK。DISABLED噪声沿用丢行/错误回执 |
| 重复/旧ARM或PULSE、ACTIVE再ARM/PULSE | 拒绝且不能续期、刷新ticket或覆写请求；已有资格也撤销，避免从失败交互继续输出。STATUS不续期 |
| STOP（含旧序号） | 完整格式/CRC正确即幂等停止，仍先执行再回序号错误；坏CRC/不完整STOP不能解释为有效STOP，但受保护状态按通信错误撤销输出 |
| Reset/异常 | Reset重新走BOOT；HardFault/NMI等保留最短板级禁用后停留，不在异常里格式化或等待回执；CPU已无法执行时不保证软件时限 |

采样校验继续保留 boot首帧1000us、raw间隔300us、反馈间隔3000us、stride10，不放宽门限。
ARM必须检查“现在”的年龄与估算有效性，不能只看“曾收到过有效帧”；USB-only f4 始终不符合资格。
零偏512帧是候选，`calibration_verified=false` 不是可绕过的电流校准结论；P1a-2不冒称闭环电流保护。

新协议序号沿用uint32向前半周期规则，`0 < (seq-last_seq) mod 2^32 < 0x80000000`；
每个已识别会话独立追踪，语法/CRC错误不推进，合法但被拒绝的新命令消耗序号，STOP仍例外先执行。
ARM不能刷新已有ticket；PULSE必须引用当前未消费的ticket，错误ticket/旧epoch不能创建新状态。
资格有效期必须在协议/profile批准时给出具体上限，当前未定，不填任意“安全毫秒数”。
PULSE的接受回执只证明进入STARTING，不证明MOE已开或电机已动；实际开始/结束及停止原因分别记录，
回执丢失不自动重试，不能通过重复请求延长输出。STOP后迟到的旧成功回执也不能重新建立资格。

### 版本兼容

第一实施批只做默认拒绝输出的软件准备：P1C1/P1R1、M3B3长度和含义完全保持，03/04仍DENIED。
开放输出前另行冻结 P1C2/P1R2 与新日志版本（拟M3B4）的逐字节布局、CRC黄金帧、状态/停止原因、
profile标识、epoch/ticket、请求/实际开始/结束设备时间。不能仅扩展旧解析器的state上限而声称兼容。
现有 P1R1 的 n/t 是ADC快照时间，不能当动作时刻；56B旧回执不够表达上述身份与动作证据。

新固件仍须识别P1C1 STOP并立即停止；可返回新版本回执，旧采集器会严格拒绝但保留原字节，
文档须明确升级主机工具，不能让旧工具误判成功。P1C1 ARM/PULSE始终拒绝；不得支持自动降级发旧输出命令。
新版本请求有限长、逐字节CRC与每poll一步组帧、无动态分配；格式未冻结前不预建实现。

## CubeMX 与时序改动表

以下是后续实现的配置目标，不是本PR已经生成的结果。以 CubeMX6.12.0/FW_F1 1.8.6 从`.ioc`生成，
检查真实生成差异，不手改生成区。所有未列时钟/通道/输入配置保持。

| 对象 | 精确目标与不变量 |
|---|---|
| TIM1 基本计数 | PSC0、CENTERALIGNED3、ARR1799、RCR3、ARPE=1、MMS=UPDATE 保持；运行中不写UG、不改CNT/RCR、不用UDIS暂时屏蔽UEV |
| TIM1 CH1/2/3 | PWM1、Pulse=0、高有效、OC fast关闭、OCxPE=1、OIS1/2/3=0；不启用互补CHxN，内部集成半桥不照搬六管死区；DTG=0，不以此证明实际驱动死区 |
| PA8/9/10 | 无TIM1重映射，生成TIM1_CH1/2/3 AF推挽；运行时由唯一board所有者在GPIO低与AF之间切换，三路使用一次CRH掩码写保留PA11..15。禁止主循环或HAL逐相Start |
| 空闲与Break | OSSI=1、OSSR=1、AOE=0、LOCK=OFF；CCxNE=0。MOE=0而CCxE=1时由timer驱动idle低；生成后核查CCER/OIS/OSSI组合，不用原例程的OSSI/OSSR关闭配置 |
| PB12 | 选TIM1_BKIN默认映射、输入上拉、低有效、BKE=1；保留GPIO IDR低检测。不是GPIO EXTI标签。真实电平/供电未确认前不能批准输出；F103该BKIN不假设有可编程数字滤波 |
| TIM1 更新与CC4 | 更新IRQ只用于确认预装载已生效，完成即关闭UIE；内部CH4 Output Compare no-output、timing模式、无PA11复用、无TRGO改接。CC4仅一次启动窗口判断，用完关闭CC4IE，不作常驻20kHz计算 |
| TIM3 | APB1 timer72MHz、PSC71、ARR65535、向上、1MHz自由计数；内部CH1 timing/no-output，用CC1 IRQ调度最近软件绝对期限；无ITR/slave/TRGO连接，不触碰TIM2/TIM4 |
| NVIC | 保持PRIORITYGROUP_4；BKIN/TIM3设0，与现有ADC/TIM4同级，互不抢占；TIM1_UP/CC设1，与DMA RX同级；TIM2仍2、USART2仍3。同级与PRIMASK阻塞均计入实测，不把“优先级0”当零延迟 |
| ADC/SPI/DMA | ADC1 PA0/ADC2 PA1双注入、7.5周期、12MHz、TIM1_TRGO、SPI1/64与50us延时、CS保持和完整帧发布不变；输出变化后原采样点未证明在低侧有效窗口，不作电流闭环依据 |

72MHz与ARR1799推导完整中心对齐周期为3598tick≈49.972us（约20.011kHz）；
RCR3更新为7196tick≈99.944us，TIM2在每次更新后约50us开始SSI，24bit/1.125MHz≈21.333us。
新实现须核对首次UEV相位和稳态间隔，不能从参考`40000`宏直接声称40kHz完整PWM或严格100us采样。

依据为 [ST RM0008 的先进定时器/GPIO说明](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
及仓库原字节 [HAL TIM头](../firmware/stm32f103/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h)
对OC idle、OSSI/OSSR、AOE/MOE的定义。本次官方PDF在线提取受大小限制，
生成寄存器与实际波形仍是后续必须补的证据，不能把此表当CubeMX已通过。

## 三相提交、故障抢占与独立期限

### 唯一写入者与最小接口

在既有`board_io.h`增加板级有调用者接口，实现在小型`board_pwm.c`；不要让command或control_core写寄存器。
所有CCR1..4/CCER/MOE/PA8..10模式及TIM3期限寄存器由该模块唯一写入；生成初始化仅负责启动前配置。
原`board_disable_output`移入该模块，仍可在初始化/异常调用，不依赖已初始化对象或日志设施。

只需一个只读`pwm_profile_t`（三路uint16 tick、uint32总时长us、配置标识和approved标志）与
一个runtime私有请求对象（请求序号/撤销代次、开始/截止时间、阶段）。profile数量先为0，
不存在可由内存默认零值误判为批准的profile；单测合成profile只进入host构建，禁止链接到正式映像。

拟定接口：`motor_runtime_arm`、`motor_runtime_pulse`由新协议调用；
`board_prepare_pulse`只装载/等事件、返回成功与否，绝不在任意调用点直接开MOE；
TIM1_UP/CC IRQ回调runtime核对请求代次；`board_schedule_deadline`写TIM3最近比较点；
`motor_runtime_stop`及`motor_runtime_latch_fault`统一撤销资格与待启动。
各接口中文注释必须说明单位、锁由谁持有、失败无部分输出及IRQ调用条件，不另加抽象驱动框架。

### 启动事务与首个脉冲

1. runtime在同一保存/恢复PRIMASK的短事务中复核资格/当前输入/nFAULT/BIF、profile整体验证和期限，
   消费ticket、登记撤销代次并进入STARTING；先安排TIM3，期限从接受PULSE开始，不从开MOE开始。
2. MOE保持0；GPIO三个IN保持低。写三路预装CCR，允许写入间自然UEV发生，因为外部尚无输出；
   写完后清旧UIF并启用UIE，等待其后的自然UEV，才能认定三个新值都已进入活动比较寄存器。
   不用UG或UDIS“原子更新”而打断ADC/TRGO；不要求CCR三个CPU写在同一机器周期完成。
3. 生成的`HAL_TIM_MspPostInit`会把引脚转AF，不能等它执行完才第一次关MOE。
   在生成TIM1_Init末尾USER CODE、其调用PostInit之前，board准备MOE0、CCxE三位统一1、
   OSSI/OSSR1、OIS低，保证切AF时是受控idle低；返回后初始化仍调用板级禁止并保持GPIO低。
   后续STARTING同样先准备idle低，再以一次CRH写切三路AF。Reset到软件接管前的引脚过程仍未测。
4. 装载确认后用内部CC4等共同低电平窗口：PWM1/正极性下`CNT > max(CCR1..3)`时三路OCREF都低。
   只在上数方向的批准窗口允许一次开门，迟到/窗口不足就不启用，不能用ADC IRQ到达当波形边界。
   profile须限制最大tick并留出“最长IRQ/临界区延迟+最终核对”的余量；无测量界限不批准窗口。
   不照抄“自然更新ISR立即开MOE”，否则首个PWM1高段可能从中途开始。
5. 在共同低窗口内持锁，核对撤销代次、仍STARTING、所有有效性/期限、PB12/BIF，再设置一次MOE；
   紧接读取BIF/PB12及期限，若失败立即撤销，必须在该低窗口结束前完成。
   这是为了覆盖“短Break已清MOE、软件随后又置MOE”的竞态；只在置MOE前读一次BIF不够。
   此后无任何周期性自动重置MOE代码。硬件BKE可继续异步关MOE，AOE=0不得自行恢复。
6. 只有成功完成最终事务才ACTIVE；关闭启动UIE/CC4IE。正常运行不动态修改CCR，后续新profile须先STOP。

这里“三相原子”指外部可见启用的共同门与整份profile一致，不声称三次CCR写硬件原子。
首次非零波形从共同低窗口后的自然比较沿开始；无法证明窗口余量时拒绝输出，不能退化为任意相位启动。

### 停止与故障线性化

统一停止事务先清MOE，再撤销请求代次/ticket/STARTING事件和期限，关闭启动中断；
保持CCxE/OSSI所定义的idle低直到GPIO BRR及一次CRH写把三路接管为GPIO低，最后清CCER。
这细化了旧总体设计的“清MOE/CCER再切GPIO”顺序，避免先关闭CCxE使AF暂时无驱动。
禁止清BKE/AOE配置或停TIM1计数，否则会破坏保护与采样。CCR可在MOE0后清零，不以清CCR代替停机。

runtime停止/故障的资格撤销与最终开门共用短临界区：若停止事务先完成，旧代次永不能重开；
若启动先完成，随后STOP必关门且不能返还资格。到达UART引脚的字节、完整STOP通过CRC、
runtime停止事务是三个时刻，不能把串口线传播延迟写成“收到任意字节即停”。
BKIN ISR先走最短停止再锁存设备故障，不等待ADC；读取BIF后按专用清位写法清标志，
不以陈旧SR读改写清掉新事件。低电平持续时禁止重新开门。异常路径不依赖命令回执。

必须逐一改造当前runtime的直接FAULT分支：采样、反馈入队/发布、邮箱溢出、失联、初始化失败均先停，
再锁存并清资格。把反馈入队/poll的允许状态扩展到ARMED/STARTING/ACTIVE，继续stride10/3000us，
不能停止消费反馈再把超时当正常。零偏候选仅在DISABLED且输出从未开始时收集，输出后冻结，不能混入带流样本。
`pwm_is_permanently_blocked`不能在新功能中保留恒true并冒充门控状态，须在版本升级时改为准确能力/实际状态。

### TIM3 最近期限调度

用同一1MHz设备时间的uint32绝对时刻记录：采样失联、反馈失联、ticket失效、PULSE截止。
只调度有效状态下最近一个，禁止新ARM/PULSE/STATUS更新已有pulse_deadline。
TIM3自身16bit CNT自由运行，CC1设`CNT + min(剩余us,65535)`模2^16；较远期限分段唤醒，
不是把32bit期限截断成16bit。比较值写入前后复核当前设备时间；若已到期立即停止，
不能写入已经越过的比较值后再等65.536ms回绕。调度/更新共享同一短临界区。

每个有效ADC/反馈发布只更新对应的输入期限，绝不续期PULSE。TIM3到点调用runtime有界检查，
先处理所有已到期停机，再安排最近下一次；不做double/CRC/格式化/等待，不进入循环追赶。
采样/反馈既有“超过300/3000us”保持，可在deadline+1us触发；PULSE在now>=deadline时停止，
两种边界分别测试，所有模差在半周期内有效。now与快照同事务取得，避免旧now交错误判。

无PULSE时也要服务ARMED输入/ticket期限；主循环停滞时ACTIVE仍须由TIM3处理deadline和输入失联。
CPU/IRQ被长期屏蔽、调试暂停或时钟停止时不保证此机制；不配置调试冻结来假装持续时间仍有效。
“配置的duration”是请求接受到门控撤销的目标上限，实际最迟停机还含IRQ等待与门控耗时。
后续须测出延迟界限才能批准profile；不能声明严格等于某个毫秒数或把该软件定时当硬件断能。

## Profile 与首次非零输出缺口

默认profile表为空，ARM/PULSE拒绝；没有“先给1%/0.3A/10ms试试”的默认值。
已知名义12V、Rs5.5Ω、Ld718.5uH/Lq905uH、INA199A1/20mΩ均来自参考；
不同三相tick造成相间电压，电流还受电感、转子位置、反电势和桥换向影响。
`V/R`及平均占空差只能用于有条件估计，不能由普通适配器和未标定两路ADC证明峰值电流安全。
三路等占空比/零tick也是输入模式，不是已经证实无相电流的物理工况。

未来唯一固定profile至少冻结：适用板卡/驱动/电机/供电标识、三路tick与最大相间差、
请求接受到截止us、共同低窗口与允许延迟、起动允许条件、允许电流风险与依据、停止/撤销规则。
编译期固定且有明确approved标志，ACK绑定profile标识及固件SHA；不能靠CRC赋予物理安全性。
若资料不能支持首次输出限值，仍交付默认无输出的软件准备，单列“profile未批准”阻断首次非零运行。

首次输出前需要用户核对的最少现场事实（本轮不请求操作）：

- 只确认U4准确丝印/实际板卡与参考连接有无差异；不重复要求整板照片或拆电机三相。
- 确认仍为哪一套电机/普通12V适配器、两路实际供电、能立即切断的现有方式，以及转子周围无接触/卡阻。
- 由资料及用户认可的风险范围确定唯一profile限值；没有限流读数和独立电流标定的事实必须保留。
- 选定会话防重放策略及可执行的每次确认方式。RX路径已有往返证据，不重复要求重新找串口线。

不能因用户能断电而把软件STOP、高阻或首脉冲质量写成已验证。

## 最小文件与分批验收

| 批次 | 最小变动职责 | 完成与停止条件 |
|---|---|---|
| D：本次文档 | 本文、总体设计链接、README入口 | 只核对路径/调用/约束/来源和diff，独立Review后停止，不直接实施 |
| A：默认无输出软件准备 | `.ioc`与生成tim/gpio/main及必要头；`board_pwm.c`、`board_io.c/.h`；`motor_runtime.c/.h`故障/期限事务；`command.c`受保护状态错误停机契约；生产路径寄存器语义测试 | 空profile、03/04仍拒绝、没有正常开MOE的可到达外部入口；目标构建/再生成/竞态与旧回归通过。只是可编译准备，不算P1a-2板端验收 |
| B：协议/资格闭包 | 用户选定策略后冻结帧与epoch/ticket，必要`command`/`sensor_capture`/新单次主机工具；按实际需要调整清单/存储布局 | 消除跨启动旧输出命令重放或明确批准的本地确认；新版本双向严格解析、状态和动作时刻可追溯，仍无默认非零profile |
| C：批准profile与板端 | 仅经批准的只读profile及其依据、现场单次计划、真实日志 | 新用户安排后才下载/上电/单次输出；失败立即停，不能自动调大/重试。完成板端门控不等于P1b电流或P1c换相标定通过 |

生成差异仍由`tools/regenerate_m1.py`精确核对原16项文件，新增TIM3在现有tim文件中；
自编IRQ只允许唯一实现，检查启动表中的TIM1_BRK/TIM1_UP/TIM1_CC/TIM3符号和map，不与生成it重复。
新增`board_pwm.c`时同步`project_sources.json`与canonical MDK源清单、verify链接入口；
用户未提交`.uvprojx`不得覆盖或混入PR。需要清单插入时先保存原字节，精确加条目并证明可还原；
严格canonical校验在隔离的精确提交工作树进行，不能为用户UI配置放宽规则。

### 验收矩阵：独立依据与反例

| 契约/独立oracle | 必须区分的反例与工况 | 证据/状态 |
|---|---|---|
| STM32寄存器语义：preload仅在自然UEV转移、中心对齐计数方向、MOE/OSSI/BKIN行为 | 每次CCR写之间插入UEV；旧UIF、窗口IRQ迟到/方向不对、CCR0/ARR/溢出；三路只能同一profile，首次高沿不能从脉冲中间开始 | 编译生产`board_pwm.c`的寄存器模型，独立预期时间轨迹；模型不能代替真实PA波形。NOT RUN |
| runtime线性化与撤销代次 | STOP/FAULT在检查前后、MOE置位前后、每次AF/CCR操作间插入；BKIN短脉冲先清MOE后恢复高；重复IRQ/旧代次不能重开 | 同一生产runtime/board的交错枚举，断言“撤销后无置MOE”和共同低窗覆盖最终核对。NOT RUN |
| 期限不续期/模差算术 | 主循环停滞、UART洪泛/TX忙、now/seq回绕、已越过CC1、长于16bit期限、多期限同刻、空profile、超期ticket、重复PULSE | 手工绝对时间oracle，与TIM3比较点独立核对；不以待测函数生成expected。NOT RUN |
| 既有输入与命令契约 | BOOT首帧宽限、raw300us/feedback3000us的等于/超过边界、SSI半帧/CRC、邮箱积压；所有新增状态继续消费反馈；错误接收及时停 | 原host回归+真实command/UART/runtime集成，保留旧失败样例；不能只mock一个“start成功”。NOT RUN |
| 无跨启动重放 | 捕获一对合法ARM/PULSE，在Reset、清RAM、不同epoch、下载擦除、同一/不同ticket下原样重放；重连不发命令不应产生输出 | B批策略选定后独立预置帧，不伪造随机性；当前BLOCKED（策略未选），空profile保持拒绝 |
| 生成闭包与目标资源 | 检查TIM1/TRGO/RCR、ADC、TIM2、SPI除表列外不变；复核HAL PostInit引脚交接；无重复IRQ/CCR所有者 | CubeMX再生成、canonical verify、ARM与正式MDK/map/HEX一致；实现批必须实跑，本设计NOT APPLICABLE |
| 板端观察 | 固件/HEX身份、真实PB12/寄存器、起停设备时刻/原因、profile/请求号、raw序号/时间/故障；无自动重试 | 当前全部NOT RUN，按C批单次计划另安排 |

现有入口如`python tools/test_command.py --gcc D:/mingw64/bin/gcc.exe`、
`python tools/build_m1.py --compiler-bin D:/Keil_v5/ARM/ARMCLANG/bin`继续使用；
新增门控测试入口需随实现提供，本文不编造一个尚不存在的测试命令。
实现批目标仍为ARM Compiler6.13.1/O2/soft-float和正式MDK，64KiB Flash/20KiB RAM，
记录新增ROM/RAM/栈与中断嵌套开销，不把已有约19,968B ROM/3,128B RAM当新增功能测量。
仅原厂已有警告可按原记录保留，自编代码零警告；新增 IRQ、临界区、TIM3调度、RX洪泛的预算须重新计时。

### 板端观察与停止条件

首次非零运行前冻结单次计划：实际供电/适配器能力、唯一profile、资格/触发、超时、软件停止和物理断电方法；
用户安排后才执行。不得因下载或Reset自动激励，不进入调试暂停，不靠人工“看起来正常”续期。
观察nFAULT/锁存故障、样本和反馈连续性、实际门控设备时间及预定运动范围；
出现非预期持续运动/声音/振动、驱动故障、数据失联、超出批准的电流观察或截止后仍报告使能，
立即停止且不自动重试；无法通信时用已确认的现有断电路径，不等待ACK。

无示波器时能取得的是MCU读取的配置/状态、设备时刻与有限机械观察，
不能确认外部首/末脉冲完整性、三相同步沿、功率桥高阻、死区、电流峰值或Reset瞬态。
这些逐项标“未测”，不以寄存器回读或CRC回执替代波形验收，也不把购买新仪器作为本次文档交付条件。
