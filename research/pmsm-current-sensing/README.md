# PMSM两相电流采样研究

基线 f2703e22cb4acde0b9caee0424f3e35aba2b7611；研究分支 research/issue28-validated-inference。
此目录为新的可验收研究实现。仓库外旧 simulation 只作探索记录，不继承其完成声明。

先看 [研究报告](docs/comparison_report.md) 或 [最终推荐](docs/final_recommendation.md)。只看结论可直接打开Markdown；结果表和曲线位于results/data、results/figures。早期selection_decision.md保留阶段历史，以研究报告为最新结论。

最简运行：先启动MATLAB R2024b，打开本目录START_HERE.m，点击“运行”。它重新执行7组对照，显示真值/反馈/参考曲线，并打开完整模型；不操作硬件。

也可在命令窗口输入：
```matlab
cd('D:/codex/motor/firmware_source/research/pmsm-current-sensing')
addpath('matlab')
run_foc_simulink
open_system('models/pmsm_foc_model.slx')
```
本机已有C桥和辨识数据，以上重新执行7组对照。默认模型因原始W阈值停止是预期复现；成功案例采用研究候选判据，不代表板端保护已改。

全部阶段入口为run_all；分阶段可用run_all('A')、run_all('B')、run_all('C')。重跑会更新同名结果。依赖MATLAB、Simulink、Control System Toolbox及C桥构建环境。c/build_bridge.py使用本机MATLAB附带Clang及Python，默认MATLAB根路径D:/Matalab，可用MATLAB_ROOT环境变量覆盖。其他机器须先检查编译环境，不能只复制SLX就保证运行。

当前已有完整平均FOC模型、351组统一闭环结果、O1/O2/O3分析、开关模型及闭环交叉、候选C接口、Issue2可复用内环和研究报告。研究文件为本地未提交修改，生产固件未改变。验收依据见docs/requirements.md，进展见仓库根目录progress.md；不烧录或操作实物。

Issue2演示：运行run_issue2_interface，再打开models/issue2_drive_demo.slx。便携C候选测试：在研究目录运行python c/test_bias_compensation.py；它不会改动生产固件。记录当前源码/结果哈希：python c/write_provenance.py。
