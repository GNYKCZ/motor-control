# M3a 连续机械角与速度估算

Refs #1、#7；[设计 v1.7](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6397865457)。
基线 main `2e0bd496056d6240f68cb03cbe0330841dc73bb2`。仅主机模块，不接固件。

## 输入、状态与失败

`feedback_init`复制配置；调用者独占持有状态，不在运行中修改内部字段，变更配置须重新初始化。
所有接口禁止状态、配置和输出对象相互重叠。`reset`保留配置并清空历史。
固定存储32点，`window_samples`为完整窗口长度2..32；未满时窗口速度无效。
raw是M2c相同的14 bit原始计数，单圈角为raw×2π/16384 rad。
首帧连续角为相对原点0，速度无效；direction ±1仅作用于连续角和速度。
读取起止为1 MHz uint32设备时钟；读取duration允许0，须不超过配置上限且小于半周期。
近似时标为start+floor(duration/2)，不是传感器真实采样沿。
后续读取区间不可重叠（端点相接允许），序号须模2^32严格加1。
中点dt须大于0、不超过maximum_gap_us，配置时间上限均小于2^31 us。
这些有界模差不能检测整时钟周期丢失；调用者须保证真实间隔小于半周期。

模差唯一范围为(-8192,8192)，半圈拒绝。
Design确认的量化包络：B=max_speed_rad_s×dt_us/1e6÷(2π/16384)，允许abs(delta)<=B+1 count。
若B+1>=8192则整拍不可辨，拒绝；不能以观测静止推断没有隐藏整圈。
包络仅判有效性，不裁剪报告速度；max_speed是调用者对真实运动的先验约束，软件无法证明物理速度满足该约束。
直接速度=有向delta×2π/16384÷真实dt；窗口速度=窗口首尾有向累计差÷真实跨窗dt。
连续计数int64、累计时间uint64均检查溢出；double角度在大于2^53计数后不再逐计数精确。
任一失败返回false，输出清零且有效标志false，整个状态对象表示不变；调用者决定故障或显式reset，禁止静默跳过坏帧继续。
首次有效帧位置有效、两速度无效；窗口未满时窗口dt/delta/速度为0，点数报告进度。

## 实施前固定的验收矩阵

| 验收来源 | 独立依据 | 工况/方式 |
|---|---|---|
| v1.7角度/时间 | 先生成整数展开真值，再取模生成输入；期望从真值和时间算 | 正反跨圈、静止、稀疏计数、抖动、时钟/序号回绕、方向±1 |
| v1.7失败事务 | 失败前后memcmp与清零输出 | 首帧无效、raw越界、零/反向/过旧dt、读取过长/重叠、跳号、半圈、超速、别名上限、整数溢出 |
| v1.7生命周期 | 独立轨迹结果与交错实例比较 | reset、无效配置、NULL、两实例隔离、窗口2/9/32 |
| 两方法比较 | 同一预定raw序列和真值，不从DUT生成期望 | N=9、10ms、每8拍1count；静止附加确定性±1count量化噪声；1→2count/拍阶跃 |
| M2c兼容 | 原解析器测试 | 运行现有sensor_input及capture回归，解析器不改 |
| 隔离 | 实际依赖、符号/section | 仅本模块头/源，无可变全局/静态，fast-math拒绝 |

预定浮点容差：相对/绝对 `2e-12×max(1,abs(expected))`，不随结果放宽。
稀疏轨迹预期直接RMS=sqrt(0.125×0.875)×(2π/16384)/0.01，窗口满后RMS约0。
静止噪声使用8拍周期[0,1,0,-1,0,1,0,-1]；窗口跨8间隔抵消该周期，不能推广为任意噪声。
阶跃的窗口完全响应比直接法晚7拍=70ms；比较说明量化/噪声与响应延迟的权衡，不声称实物改善。

## A13 例程逐函数对应与许可

参考为同级stm32_foc_2原始版本`672e32f1005d9019140d1a9c754fb8ca2e8eedde`（本地参考，不加入提交）。
本轮实际读取的三个文件SHA256依次为：

- Core/Src/tim.c：`0913881fc94de89c586a6af00240ce355ba70ef35412ced48500d46066a871f0`
- Drivers/motor/foc.c：`e3e8d1d4b70c5014086f1a8d338a32eb624c337718806bf5552ac3821debccf2`
- algorithm/filter.c：`c45fc0a230f6f0c882d9f475225594ff8069210aad188f9cbf13a8d886a29ba0`

| 参考位置/函数 | 新模块对应 | 行为改变/未引入 |
|---|---|---|
| Core/Src/tim.c:403-436 TIM3回调 | feedback_step | 参考回绕差/真实时间；显式状态、1us时钟、首帧无速度、序号/歧义拒绝；不引入中断、LED、共享角度、速度/位置环 |
| Drivers/motor/foc.c:293 cycle_diff_fast | 模差计算 | 用有界加减替代负数左移/带符号右移，恰好半圈拒绝 |
| algorithm/filter.c low_pass_filter | 无对应调用 | 固定alpha低通未引入；本批使用实际跨窗dt的窗口差分 |
| algorithm/filter.c kalman_filter_std | 无对应调用 | 静态z/p及Kalman滤波未引入 |
| M2c sensor_input/board_encoder_frame_t | feedback_sample_t值输入 | 保留raw/序号/起止时间含义；调用者未来自行复制，不依赖板级头 |
| tools/sensor_capture.py | 不修改 | M1/M2C1原始日志契约保持；本批无板端日志 |

原例程作者黄铮（best_pureer）；USER CODE许可未明。本模块按公开数学关系独立实现，不复制其源码或擅自声明原作者代码许可。

## 运行与证据

```powershell
python -X utf8 tools/test_feedback.py --gcc D:/mingw64/bin/gcc.exe
python -X utf8 tools/test_sensor_input.py --gcc D:/mingw64/bin/gcc.exe
```

实际Python路径按环境选择；本机Python3.12.14、GCC8.1.0。
忽略目录`build/host_feedback/`保存test_evidence.json（完整SHA、工作区状态、输入SHA256、工具版本、命令和原始输出）、synthetic.csv与metrics.json。
数据为确定性合成，无随机种子，无删点/重采样/调参；失败证据另存。
实际合成结果（窗口9点、10ms间隔）：稀疏轨迹直接RMS误差0.0126829115 rad/s、窗口约3.01e-20 rad/s；
指定周期噪声直接RMS误差0.0383495197 rad/s、窗口0；阶跃完全响应位于第64/71拍，相差70ms。
RMS固定统计第8..127拍共120点，CSV保留所有129点×3轨迹，包括首帧和未满窗口。
目标构建/资源变化 NOT APPLICABLE（未接固件）；硬件、WCET和真实日志比较 NOT RUN。
方向、电角度零位、相序均未板端标定。PWM继续永久禁止；功率桥高阻关断仍是未满足项。
