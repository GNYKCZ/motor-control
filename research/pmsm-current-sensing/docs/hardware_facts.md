# 参数证据与冲突处理

基线SHA：f2703e22cb4acde0b9caee0424f3e35aba2b7611。

|量|值|证据性质/路径|
|---|---|---|
|R,Ld,Lq,psi|5.5Ω,0.7185mH,0.905mH,0.00257Wb|用户提供参考例程名义值，非实测|
|极对数|7|current_loop.c候选标定，非完整标定|
|Vbus|12V|current_loop.c硬编码名义值，无母线ADC|
|PWM|20kHz|TIM1 PSC0 ARR1800 CMS3，72MHz|
|UEV/ADC|1kHz|RCR39，40个半周期|
|ADC|12bit，3.3V名义参考，12MHz|board_config.h；main.c PCLK2/6|
|U/V|ADC1_IN0 / ADC2_IN1|adc.c双ADC同步注入；附件中V为ADC1_IN1是引脚通道名，实际使用ADC2|
|采样/转换|7.5/12.5cycle|ADC采样配置；孔径起点延迟仍待确认|
|触发相位|225/250us|TIM2 OC2REF；不自动等同所有duty下的窗中心|
|电阻/增益|20mΩ/50|board_config.h名义值|
|PI|Kp0.5 Ki150 tracking200|current_loop.c|
|参考限值|40mA|current_loop.c；非物理硬电流限流器|
|电压圆限幅|12*0.0577350269/sqrt(3)=0.4V|current_loop.c、current_control.c|
|提交延迟验收|740～810us|current_loop_applied；并非板端保证|
|电零位|7453，方向+1|current_loop.c实验候选|
|INA199A1带宽|80kHz典型|TI SBOS469H第6页 https://www.ti.com/lit/ds/symlink/ina199.pdf|
|REF电路|R12/R13各10kΩ，分压1.65V共接两INA，无图示缓冲|A1原理图第1页；实板一致性待核对，见ref_root_cause.md|
|J/B/实际温度与Vbus范围|TBD|参数化研究，不能称实测|

默认ARM构建DQ_VEC0，只固定CCR900，无PI输出；闭环编译路径另行建模。闭环没有duty补偿。
40kHz 0/50/70%读数是历史近似摘录；20kHz仅50%近似U60/V59mA，无法独立确认斜率。无完整原始流。
附件中的fault_flags=4不作为dq故障证据；以MOTOR_FAULT_EXPERIMENT枚举和日志来源核对。DRV8313为电荷泵结构，100%异常不得命名为已证实自举故障。三相相同duty不保证实际电流严格零。
