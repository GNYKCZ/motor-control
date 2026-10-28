# P1a1 无输出串口命令往返

Refs [Issue #1](https://github.com/GNYKCZ/motor-control/issues/1)，依据
[P1a设计](p1a_output_gate_design.md)，基线main `b56d01aef9e878444d45d0c85733873dd6fa09fd`。
本批只实现 USART2 接收、STATUS/STOP 和回执；ARM/PULSE 明确拒绝。
PWM 仍永久禁止，原始采样、stride10、300us/3000us门限和故障锁存保持。
软件验证不证明 RX 实际导线已接通，本批没有打开 COM9、下载、上电、复位或转轴。

## 帧格式和判断依据

115200/8N1，所有数字为固定宽度大写十六进制，不接受空格、小写、浮点或可变参数。
下文CR/LF表示实际0D/0A字节，不是反斜杠文本。

请求固定23B：`P1C1,ssssssss,cc,hhhh` 后接CRLF。

| 0基字节位置 | 内容 |
|---|---|
| 0..4 | ASCII P1C1, |
| 5..12 | uint32序号，8位hex |
| 13 | 逗号 |
| 14..15 | 命令：01 STATUS、02 STOP、03 ARM、04 PULSE；其他值拒绝 |
| 16 | 逗号 |
| 17..20 | CRC16，4位hex |
| 21..22 | CR LF |

CRC为CRC-16/CCITT-FALSE：poly0x1021、init0xFFFF、非反射、xorout0；
覆盖请求字节0..16，包含CRC前的逗号，不含CRC文本和CRLF。
独立通用检查串 `123456789` 的结果为29B1。
黄金请求为 `P1C1,00000001,01,A1E6` 和 `P1C1,00000002,02,1A67`，各后接CRLF。
没有以生产C代码计算测试expected；主机另用标准库crc_hqx核对。

回执固定56B：`P1R1,seq8,cmd2,result2,state2,fault8,n8,time8,crc4` 后接CRLF。
这里seq8等表示字段宽度，实际帧不写字段名。CRC覆盖字节0..49，含最后逗号；
CRC文本在50..53，CR/LF在54/55。各字段使用hex。

| 回执字段 | 含义 |
|---|---|
| seq / cmd | 请求中解析的序号/命令；非法布局可能不可信，UART错误为0/0 |
| result低7位 | 00成功、01格式、02 CRC、03重复/旧序号、04命令禁止、05 UART错误、06超时、07快照失败 |
| result高位80 | 待回执邮箱发生覆盖/拒收；至少一条回执丢失，不是动作未执行证明 |
| state / fault | 一致小快照中的BOOT0/DISABLED1/FAULT2与故障位，STOP不清故障 |
| n / time | 同一快照最新ADC完成序号/设备us，非收命令或回执送出时刻 |

黄金回执：`P1R1,00000001,01,00,01,00000000,00000019,000009C4,0F49` 后接CRLF。
对应STATUS seq1、DISABLED、无故障、ADC序号25/时刻2500us；来自主机手工时间轨迹，不是板测。
现有M3B3日志仍为原16字段。`tools/sensor_capture.py`保留旧格式并增加P1R1严格长度/CRC解析，
日志与回执每行完整交错，不能拼在一行或在TX中途抢断。
两种格式的快照准备时刻可能不同，恢复发送的历史M3B3行可早于先发出的回执；
不能把混合流的n/t按全局单调排列，须按格式和快照时间分析。

## 序号与恢复

本次启动第一条格式/CRC正确的请求可取任意uint32序号；以后
`advance=(seq-last_seq) mod 2^32` 须满足 `0 < advance < 0x80000000`。
FFFFFFFF→00000000为向前，重复、倒退与恰好半周期拒绝；被禁止但语法/CRC正确的新命令也消耗序号。
语法、CRC、UART或超时错误不推进序号。主机每次只发一条，建议不超过5条/秒，无自动重试。

STOP是例外：只要完整帧语法/CRC正确，即使序号重复或旧也立即幂等禁用，再回03；
因为过期回执不应阻止停止。ARM/PULSE和未知命令永不执行使能；若序号过期先回03，否则04。
CRC只防传输错误，不提供权限认证、硬件保护或操作授权。

字符间隔超过25000us或整帧超过250000us时拒绝；按uint32模差处理，
前台取时后被RX抢占的旧now不被误判为超时。格式错误/超长/超时/UART错误后，
必须丢至真实LF边界；内嵌P1C1前缀或合法格式尾部不能恢复为命令。
错误行和终止符不正确的STOP不会执行；错误回执为尽力发送，不保证每条都收到。
超时由RX和前台检查，不要求额外定时器；RX中断本身没有等待循环。

初始化清除解析、序号和待回执，不保存待执行动作，不能因主机重连自动发命令。
当前只可能执行幂等STOP，重启前残留的合法STATUS/STOP仍无法使能输出；
未来增加ARM/PULSE前必须另审会话/资格与积压帧的重放防护，不能继承本批安全结论。

## IRQ和前台工作量

board_uart.c统一拥有UART寄存器和TX缓冲借用状态，真实调用者为main和USART2_IRQHandler。
每次IRQ读取一次SR，最多读一个DR并处理一个RX字节，然后最多交付一个TX字节或完成TC。
RX先于TX，防止同次RXNE+TXE时误把写入的TX读成请求。
ORE/FE/NE/PE均按SR→DR清除并丢弃当前行；无字符串扫描、分配、浮点或阻塞HAL调用。

command_receive_byte每次最多8轮CRC及一个hex位的位移/校验，固定位置走分支；
最后LF只做固定次数的长度/CRC/序号判断。RX状态由UART IRQ独占，前台超时/邮箱操作持短临界区。
CRC正确的STOP在RX IRQ内直接调用motor_runtime_stop，后者在保存/恢复PRIMASK的临界区调用
真实board_disable_output；不等待前台、回执邮箱、组帧或UART空闲，不把FAULT改回DISABLED。
当前所有采样/故障路径仍禁止调制，根本不存在ACTIVE使能分支；本批不能证明尚未实现的ACTIVE竞态正确，
未来增加ACTIVE必须把资格撤销与开门的最终核对纳入同一状态事务，再独立测试。

完整23B在115200/8N1的线传输约1.997ms，之后还有IRQ调度和门控耗时，均未板测。
CRC坏、线断、未接RX或中断无法运行时，串口STOP不能保证停止，因此它不是独立急停或断能。
未来输出仍需独立期限/故障门控及现有物理断电路径。

回执用一个待事件槽和一个前台发送槽，不无限排队；待STOP回执优先于新STATUS，
新STOP可以替换待STATUS；已经冻结/组装或借用的回执不改写，后来的事件最多再等一行。
覆盖或拒收待事件设置result的80位。停止动作不因回执丢失而撤销。
前台准备时复制约40B一致快照，后续每次poll只生成一个字符、最多8轮CRC，无printf。
一条回执需1次准备+56次字符poll；串口忙时直接返回。已提交缓冲保持到真实TC后释放，
末字节写DR不等于发送完成。回执优先于新周期日志，但不能打断已借用的日志；
请求洪泛可以使日志饥饿，不能令主循环等待通信。先检查deadline和消费反馈，再做一个通信步骤。

RV-001修正：TC分支仅关闭TCIE并释放缓冲，不再对USART_SR读改写。
RXNE可由写0清除，陈旧SR值的RXNE=0写回会覆盖读写之间新到达的字节；
保留TC而关闭TCIE不会继续触发TC中断，下一次TXE路径先读SR再写DR，由硬件清TC。
既有ST HAL头的清标志说明也记录这一SR读/DR写顺序，参考
[本项目原字节HAL头](../firmware/stm32f103/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h)。
新增test_uart_tc_race.cpp用C++寄存器语义模型编译同一生产board_uart.c，
模拟SR写0清位与读改写间新RX到达，覆盖STOP全部23个字节与TC重叠、保留TC/下次发送清除；
测试不是重写ISR，也不是板端时序复现。原普通内存替身继续验证其他UART交错，不能单独覆盖这一硬件语义。
使用相同新模型和旧head `2e839b52395261fa766b5c4f8a2ae18e9092f1f8` 的原UART源离线构建，
旧实现因STOP未执行而失败，修正后通过；失败证据保存在忽略build/p1a1/rv001_old_uart。

本节是静态工作上界，不是硬件WCET。USART2保持低于ADC/DMA优先级；完整新增路径的实际耗时、
RX洪泛影响和板端fo均NOT RUN，不能从M3B3原来18us日志数值推定本批耗时。

## 初始化与用户设置

.ioc只将USART2.UartMode从TX改为TX_RX。CubeMX6.12.0/FW_F1 1.8.6实际再生成，
16项生成文件中只有usart.c收发模式发生变化；引脚/时钟/定时器/ADC未改。
main在启用外设前初始化command，在设备时间启动后才启用RXNE/PE/EIE，启动前清旧SR/DR。
参考例程TX_RX只用于核对引脚/模式，不引入其阻塞fputc。

project_sources增加command.c和board_uart.c，正式MDK清单同步。
用户原Keil下载/UI配置不覆盖，仅精确插入这两项源条目，移除插入字节即可还原原文件哈希；
canonical工程通过generate_m1_mdk.py的--output另存、按路径提交，不混入用户配置。
严格验证仍使用精确提交的干净Git工作树，不能为本地设置放宽校验。

## 电脑工具与验收

只生成待发送帧，不开端口：

```powershell
python tools/command_link.py --command status --seq 1
python tools/command_link.py --command stop --seq 2
```

现场另有用户安排、确认现有DAP TX确实到PA3后，才使用--port COM9；先断开VOFA对同一端口的占用。
显式--port还须给--output全新目录及真实--firmware-sha/--board/--motor/--supply。
使用pyserial3.5，DTR/RTS在开口前设低，但不保证具体适配器不会引起复位。
工具只发送一次、接收默认1秒，保存raw.bin/records.jsonl/metadata.json/send_result.json；
前者元数据在发送前创建，是否完成写入以send_result为准。部分写入/异常不重发，所有原始接收字节保留。
匹配回执须CRC有效且seq和cmd对应；没有回执不等于STOP未执行，也不能当已停机证明。
仅收到M3B3不能区分本批与旧只采样固件，需要本次构建身份与P1R1往返证据。
主机时间是收到/解析记录的时间，不是MCU逐拍时序测量。

| 验收项 | 独立依据/软件工况 | 证据类别与限制 |
|---|---|---|
| 帧与CRC | 明写黄金请求/回执、123456789=29B1；184种请求/448种回执单bit破坏 | 主机，非线缆噪声实测 |
| STOP优先 | 真实UART ISR→command→runtime_stop；TX忙、待STATUS/待STOP、已借用ACK、重复/旧seq、FAULT、嵌套mask | 主机寄存器/门控替身；不模拟实际桥电气状态 |
| 边界恢复 | 错CRC/前缀/长行+合法尾部、半行、字符与整帧超时、ORE/FE/NE/PE | 真实board_uart ISR；不模拟NVIC延迟 |
| RV-001寄存器竞态 | 写0清RXNE/TC、SR读写间新字节、23个STOP字节位置；旧实现负向对照 | 同一生产ISR的语义模型，非板端实测 |
| 时序/快照 | seq/时基回绕、旧now交错、反馈持续推进、故障抢占、冻结快照、同时RXNE+TXE | 真实runtime和通信路径，原反馈门限不变 |
| 混合流 | runtime→UART→P1R1/M3B3→capture，分块保存和严格解析 | 主机合成来源明确标记，不冒充板端日志 |
| 工程闭包 | CubeMX再生成、原五组host、命令集成、ARM/正式MDK、严格verify、资源/警告 | 最终SHA和哈希写PR/忽略build证据 |

入口为 `tools/test_command.py --gcc D:/mingw64/bin/gcc.exe`，保留失败证据。
同一工具目录的g++.exe用于主机寄存器语义模型；目标固件仍全部按C编译，没有引入C++运行时。
本批到软件审查交付停止；RX物理可达、实际UART IRQ耗时和所有输出测试均未执行。
