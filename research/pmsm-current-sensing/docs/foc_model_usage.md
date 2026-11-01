# 完整FOC模型运行说明

在MATLAB中进入本研究目录：
```matlab
addpath('matlab')
run_foc_simulink
open_system('models/pmsm_foc_model.slx')
```
需要先执行Model A/离线辨识生成参数结果，并用Python运行`c/build_bridge.py`构建原始C控制器MEX。当前构建脚本支持本机MATLAB附带Clang；可通过MATLAB_ROOT环境变量更改安装目录。

模型由三个真实连接模块构成：
- SPMSM_dq_and_mechanics：id/iq方程、反电动势、磁阻项、机械转矩与惯量方程。
- Two_phase_sensor_and_ADC：真实UV只进入传感器生成器，叠加偏移与噪声，再量化为ADC原始数。
- Firmware_PI_SVPWM_and_guard：只接收ADC与角度，调用未修改生产C核心；1kHz采样、CCR1800量化、775us提交延迟、保护状态。

默认保存配置使用原固件保护；打开运行出现raw_W停止是预期复现，不代表模型坏了。候选interval保护只用于研究，未经授权不修改板端。查看`run_foc_simulink`的cases表可以运行完整7种对照。

输出：Plant_signals记录UV、theta、id、iq、机械速度和电磁转矩；Controller_signals记录积分、三相duty、使能、故障、反馈等。细节见control_tick.m。结果与图保存在results。

故障码：1 ADC饱和，2原始UV阈值，3原始W阈值，4区间证书/限值拒绝，5控制核心拒绝，6dq绝对值，7跟踪误差，8动作期限结束。故障处理是已核实关键逻辑的仿真镜像，不是完整motor_runtime.c状态机或真实硬件验证。

本阶段假设：电零位正确；初始角度零；角度测量没有加入SSI年龄误差。默认锁转子，转动用已知机械速度；动力学模式J/B为研究假设。电机参数为名义值，不能据此声称2208参数已实测。关闭输出后用零相电压等效模拟电流衰减，不宣称DRV8313所有实际停机状态都等价短路。

REF、偏移、噪声、外部真实Vbus为仿真参数。控制核心的母线输入保持固件的名义12V，植物使用实际Vbus。候选补偿若用Vbus特征，则假设该信息可用；原固件无母线ADC，部署须明确获取途径。

合成M3历史统一矩阵采用高窗，现在以cfg.adc_phase_s=250e-6显式记录，不能冒充固件默认225us低窗。run_phase_features用独立TIM1中心对齐计数公式验证225/250的低/高状态，并分别运行M3闭环。相位参数仅改变共模/时序特征；ADC触发到实际孔径的板端延迟仍需实测。Model B为平均电压对象，开关纹波由Model C研究。
