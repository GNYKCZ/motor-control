# P1a-2 A 软件准备与验证

Refs [Issue #1](https://github.com/GNYKCZ/motor-control/issues/1)，基线main
`440086e391e99759a92a9469372dcd68c80ef23d`，依据 [P1a-2设计](p1a2_pwm_gate_design.md)。
本批交付完整门控软件准备，生产profile数量为0，P1C1 ARM/PULSE仍明确DENIED；
没有串口/Reset到非零输出的正常路径。没有完成P1a-2板测、P1b零偏验收或任何电流闭环。

## 实际改动

- `.ioc`经CubeMX6.12.0/FW_F1 1.8.6生成TIM1 PWM1 CH1/2/3、PA8/9/10 AF、有效低PB12 BKIN，
  BKE开启、AOE关闭、idle低及OSSI/OSSR开启；内部CH4仅用于共同低窗口，不引到PA11。
  新增TIM3 1MHz自由计数/内部CH1最近期限IRQ。TIM1 ARR1799/RCR3/TRGO UPDATE、双ADC、TIM2 50us、SPI/DMA不变。
- `board_pwm.c`唯一拥有运行期三相CCR/MOE/CCER/引脚模式与TIM3期限。装载时MOE0，清旧UIF后等待自然UEV，
  再等上数共同低窗；设置MOE前后均检查Break/输入/窗口。没有运行期UG/UDIS或逐相HAL PWM Start。
- 生成TIM1初始化在PostInit切AF之前，由USER CODE准备MOE0/三路idle低；生成后仍回到GPIO低。
  停止顺序为清MOE→撤销待启动→GPIO接管低→清CCER，保留TIM1采样计数与BKE。
  `regenerate_m1.py`现保留main.c和tim.c的具名USER CODE，其余生成区仍精确比较。
- runtime新增ARMED/STARTING/ACTIVE内部状态、单次资格/撤销代次；采样、反馈、邮箱、失联、通信错误均能撤销。
  受保护状态继续stride10反馈，不放宽300us/3000us门限。STOP先禁用和撤销，再处理回执，不清故障。
  TIM3检查最近绝对期限，不依赖主循环推进，长于16bit期限分段唤醒，比较点越过则立即处理而非等待回绕。
- 新旧协议和M3B3格式均未升级。新状态只在主机合成profile构建中可到达；生产只会发布原state0/1/2。
  `pwm_is_permanently_blocked`在生产仍为true，含义是没有批准profile；不能用它证明功率桥高阻。

生产唯一profile查询返回0，不提供动态注入。测试宏`MOTOR_GATE_TEST_PROFILES`只给host合成固定轨迹，
在ARM架构定义下明确编译失败；目标工程无此宏。合成tick、余量和时长均不是实际电机安全参数，不能拷入生产。
后续开放输出仍需实际U4、已批准profile与延迟界限、用户选定的会话防重放策略和新版本协议；本批不实现Flash epoch或按钮。

## 可复现的软件入口

从仓库根目录执行，均不开串口：

```powershell
python -B tools/test_pwm_gate.py --gcc D:/mingw64/bin/gcc.exe
python -B tools/test_m1.py --gcc D:/mingw64/bin/gcc.exe
python -B tools/test_command.py --gcc D:/mingw64/bin/gcc.exe
python -B tools/build_m1.py --compiler-bin D:/Keil_v5/ARM/ARMCLANG/bin
python -B tools/verify_m1.py
```

严格verify/正式MDK用精确提交的干净工作树，不能直接把保留的用户下载配置当canonical配置。
本次用户`m1_sampling.uvprojx`工作文件保持原字节，不添加条目、不覆盖、不暂存用户设置；
提交中的canonical工程从`project_sources.json`另生成，只增加`board_pwm.c`源项。
用户本地工程若继续使用旧源清单，需要另行安排合并源清单后才能用于本批Keil构建，不自动改下载配置。

测试模型编译同一生产`board_pwm.c`，由真实runtime/command调用；C++仅用于寄存器行为模型，目标仍为C。
区分生产空profile构建和合成profile构建，不靠“mock start成功”覆盖输出路径。

| 验收 | 实际方法与证据限制 |
|---|---|
| 生产无输出 | profile count0，遍历256个ID与PULSE/无请求IRQ均不开MOE；旧P1C1黄金帧ARM/PULSE仍拒绝 |
| 装载/共同低窗 | 自然UEV转移preload、方向/窗口迟到拒绝、三相一致；无UG写、不破坏GPIO高位。是寄存器模型，不是PA波形测量 |
| 停止/故障抢占 | 枚举生产寄存器访问点插入UEV、异步短Break、屏蔽期间排队STOP；旧代次/迟到IRQ不能重开 |
| 独立期限 | main停滞，raw年龄300us不提前报错、301us停机；保持采样时PULSE期限不延长，ARM到期、16/32bit回绕、比较点越过均覆盖 |
| 输入/通信 | 反馈邮箱积压、空样本、CRC/UART错误、低nFAULT、陈旧输入、重复ARM/PULSE、Reset撤销；原采样/UART TC竞态与混合流回归 |
| 工程闭包 | 16项CubeMX再生成精确一致，生产ARM与canonical正式MDK、HEX一致、map含真实IRQ；ROM/RAM/警告按最终提交证据记录 |

本地证据：`build/host_pwm_gate/test_evidence.json`、原host测试证据、
`build/p1a2a/`下目标/MDK/最终汇总，以及`build/cubemx_*/regeneration_evidence.json`。
失败尝试保留在各验证目录，不以单次PASS替代独立Review。最终PR列被测完整SHA、资源与实际检查结果。

## 本批板端观察/停止清单（未执行）

本轮禁止COM9、下载、上12V、运行PWM。若用户后续安排只采样验证：

1. 记录确切HEX/固件SHA、板卡、电机和实际供电；下载仍保持12V断开，不自动运行任何输出profile。
2. 查看BOOT后仍为DISABLED或锁存FAULT，raw序号/时间推进；USB-only的f4不屏蔽，不能清故障强行ARM。
3. 显式STATUS/STOP单次往返仍按原工具保存原始流；ARM/PULSE应被拒绝，不用“听不到转动”替代软件检查。
4. 若出现非预期运动/声音/故障或失联，立即停止且不重试；通信不可用时采用已确认的现有断电方式。

EN/nSLEEP固定高的高阻缺口仍在。无示波器不能确认外部首末脉冲、三相边沿、死区、电流峰值或Reset瞬态。
硬件IRQ延迟、最长临界区与电气停机均NOT RUN；profile空表保证不开放实验，不给任何虚构的安全电流/占空比/时长。
