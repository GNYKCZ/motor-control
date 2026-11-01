# 研究进展

目标保持用户附件第0–13节完整范围；未完成，不宣称方案验收。

## 2026-11-01 当前工作
- 已读取附件、仓库AGENTS/README/工作流、Issue2、核心与ADC/PWM源码。
- 已确认基线SHA f2703e22cb4acde0b9caee0424f3e35aba2b7611，原工作区干净。
- 创建研究分支 research/issue28-validated-inference。
- 已建立research/pmsm-current-sensing参数、采样链、M1/M2/M3特征、LS拟合及实验设计源文件。
- A测试/SLX与算法比较运行正在实施；本条不代表已通过。

## 命令与证据
- git rev-parse HEAD / git status --short；git switch -c research/issue28-validated-inference。
- gh api repos/GNYKCZ/motor-control/issues/2：已读取接口范围。
- 源码静态核对：12MHz ADC，1kHz控制，20kHz PWM，0.4V电压圆限幅，默认DQ_VEC0。

## 问题
旧报告缺完整辨识闭环；不继承验收。附件fault_flags/自举/A1带宽与更新证据冲突，见hardware_facts.md。

## 下一任务
完成Model A独立测试和实际SLX；运行LS/RLS/梯度与门控；然后编译真实C控制器并进入Model B。

## 阻塞
完整实测日志、实际电流参考缺失；不阻塞合成研究。编译器未在PATH，待检查MATLAB已安装支持。

## 2026-11-01 本轮续接结果（有实质进展）

### 已完成及验证
- Model A实际SLX与独立MATLAB链路：7项检查PASS，最大差9.02e-17 A。
- M1/M2/M3：9组调参/留出比较，189组算法/噪声/延迟组合；等duty与单矢量零序注入秩2、联合变化秩3；门控/真实电流误扣反例PASS。
- 未修改的current_control.c/foc_math.c已通过本机Clang编译为MATLAB可调用MEX。1000组随机参数的独立double公式对照，最大差6.20491e-08。
- Model B第一批30组受保护闭环/诊断对照已运行。10→5us细化最大轨迹差8.43211e-05 A。
- 发现关键事实：原始W相判据在current_loop补偿前执行；M1/M2约60mA双通道偏移会在首个样本触发raw_W，所有下游补偿都无法进入闭环。未关闭或修改该保护。
- 20ms DQ_VEC0通过仅说明固定矢量诊断，非闭环通过。

### 新增/修改文件
- matlab/run_model_a.m、run_identification.m、run_model_b.m及参数/采样/拟合函数。
- models/sensor_bias_model.slx；results/data、figures、logs下实际运行证据。
- c/firmware_core_mex.c、c/build_bridge.py；编译证据build/research_host/build_evidence.json（生成物）。

### 实际命令
- matlab -batch "addpath('D:\codex\motor\firmware_source\research\pmsm-current-sensing\matlab'); run_all; disp(mex.getCompilerConfigurations('C','Installed'));"
- python research/pmsm-current-sensing/c/build_bridge.py
- matlab -batch "addpath('D:\codex\motor\firmware_source\research\pmsm-current-sensing\matlab'); run_model_b"
- git diff --check（通过；新增未跟踪文件尚需最后全面检查）。

### 发现和修复
- run_all生成字符串换行错误已修复；Simulink输入外推设置及Delay初值尺寸已修复，失败未包装成功。
- MATLAB没有已配置MEX编译器；LCC不支持生产代码局部初始化。使用本机自带Clang17、SLDRT头文件和Windows UCRT导入库，原生产源文件不改。编译路径属于本地主机适配，不宣称官方MEX配置已安装。
- Model B目前为锁转子场景、运行时保护关键路径的数学镜像；尚非完整runtime C执行或全dq机械动态。step后施加dq电压未完全从量化duty反算，下一步必须修正；不能用此初批结果宣称整个B通过。
- 初期下载llvm-mingw的curl进程曾在session 66637运行，替代本地Clang路径已成功。下一轮先核实/结束不再必要下载（仅任务自身进程），不要重复启动。

### 下一步明确任务
1. 扩展B：按量化CCR重构施加电压、实际采样/提交事件、零参考和±40mA；构建真正pmsm_foc_model.slx及同核心对照。
2. 研究保护与补偿的误差区间契约：保持现有保护基线，候选方案必须保留有效保护，不通过关闭判据制造闭环结果。
3. O2测量偏置ESO与物理扰动ESO明确方程、失配/噪声/带宽对照；扩展O1收敛/漂移及模型外推。
4. C开关、死区、模拟前端、孔径与平均模型交叉验证。
5. 统一模型/算法/数值/失败证据，Issue2接口、最终推荐、完整要求审计。

### 当前阻塞
实测原始流、独立电流/电压参考仍缺；不阻塞名义研究。编译器阻碍已解除。全目标保持ACTIVE，未完成。

## 2026-11-01 续接：CCR、保护区间、ESO、开关级模型（实质进展）

### 完成和测试
- 修正B施加电压，由实际量化CCR反算三相电压，不直接使用PI请求dq值。
- 首次10/5us步长检查失败，查明775us事件取整为780us；改用5/2.5us准确表示同一提交事件。复跑30组，核心公式差6.20491e-08，轨迹细化差3.1225e-16 A。
- compensation_guard候选：偏移证书有效时使用电流区间上界，未知/饱和/超域停止。50000组验证，14294允许、界内超限误允许0；另保留错误证书会失败的反例。未改固件保护。
- O2新增真正扩张状态ESO而非Kalman替代：物理电压扰动与测量偏置不同方程。120组噪声/带宽/失配比较；同时未知两者时3状态可观测秩2，显式不可观测方向验证通过。
- C第一批24组理想MOSFET/二极管/死区/模拟响应/孔径模型完成。KCL/等duty零电流/平均电压/步长细化通过；平均电压差5.997mV，采样电流细化差0.490mA。已查看ESO图。
- 终止已确认仍在下载但不再需要的任务curl PID22456；本地Clang路径已成功，无需额外编译器下载。

### 文件
新增matlab/compensation_guard.m、run_guard_validation.m、run_eso.m、run_model_c.m；修改run_model_b.m。
新增docs/protection_contract.md、eso_analysis.md、model_c_initial.md；结果在results/data与figures。

### 实际命令
matlab -batch "addpath('D:\codex\motor\firmware_source\research\pmsm-current-sensing\matlab'); run_guard_validation; run_eso; run_model_b"
（前三项中的B首次失败，已记录原因并单独修复重跑）
matlab -batch "addpath('D:\codex\motor\firmware_source\research\pmsm-current-sensing\matlab'); run_model_b"
matlab -batch "addpath('D:\codex\motor\firmware_source\research\pmsm-current-sensing\matlab'); run_model_c"
C源代码清理/直通不变量修正后已再次执行PASS。

### 尚未完成/下一项明确执行任务
1. 构建pmsm_foc_model.slx（完整dq植物、实际C控制器、量化CCR、事件时序）；目前B只有MATLAB锁转子数值实验，不得称完整B。
2. 零参考/±40mA、转动反电势、机械参数假设、噪声/母线/延迟/温漂/模型失配统一工况，ESO/O1/O3同条件比较。
3. 保护区间候选仅在独立误差证书下进行闭环仿真；保留原固件保护基线，不自动部署或放宽阈值。
4. C修正为U/V两采样电阻拓扑、解析事件或更准确死区、采样转换时间；评估Simscape支持，完成闭环交叉。
5. 更新run_all统一入口（当前只A/辨识），编译入口跨机器配置、全部测试与完整要求审计。
6. 最终决策报告/Issue2接口/实验最小数据/移植建议仍未完成。

### 阻塞
缺实测流/独立真实电流基准影响最终硬件系数；不阻塞以上名义仿真。全目标仍ACTIVE。

## 2026-11-01 完整FOC、O3失配、在线漂移（本轮续接已核验）
- pmsm_foc_model.slx已实际运行7个工况：原保护raw_W停止、未知证书停止、匹配候选、零参考、±40mA、转动；能量平衡误差2.77556e-17。
- 独立MATLAB调度器与Simulink相同方程交叉差约4e-323（数值零），仅为调度/执行交叉，物理独立判据另有能量和锁转RL。
- 局部离散线性54组R/L/延迟扫描通过；名义d/q带宽4.1702/4.2004Hz，不是实测带宽或全局稳定结论。
- O1新增16组激励/漂移试验，记录门控更新、收敛时间和错误吸收真实40mA电流的反例。
- O3新增120组对象恢复时间/非线性失配，47通过验证筛选、73拒绝；本批无验证通过却测试超3mA。不是无限域误差证书。
- C已改为真实U/V两采样电阻、W无分流器拓扑。三状态数值KCL检查失败后改为物理消元二状态，24组重跑PASS，平均电压差7.70474mV，细化差0.490824mA。
- 闭环统一压力测试273组（13情形×7算法×3种子）已结束：session64257退出0，foc_robustness_revised.txt完整。候选证书被故意失配，必须结合diagnostic_envelope_breaches解释，不能把未停机当成安全证据。
- 添加bias_ESO后控制器状态从24到28；需要重新生成/验证SLX及最新接口（旧7组结果来自24状态但不含ESO）。

### 新增文件与命令
新增closed_loop_config、pmsm_rhs、pmsm_outputs、三级S函数、control_initial/control_tick、sample_adc、simulate_foc、build_foc_model/run_foc_simulink、run_linear_analysis、run_adaptation_stress、run_o3_mismatch、run_foc_robustness；更新run_all支持A/B/C/all。
命令均为MATLAB -batch addpath研究matlab后执行同名run_*；完整过程见results/logs/foc_simulink.txt、foc_robustness_revised.txt。
新增docs/foc_model_usage.md、linear_analysis.md、o1_adaptation.md、o3_mismatch.md、interim_findings.md。

### 修复/限制
- Simulink期限事件最初未被输出记录，延长到期限后两个微步验证停止；没有放宽停止断言。
- 误差界须包含偏移、噪声和增益等；高斯噪声无确定有限界。已修正guard说明和压力统计，不把3sigma冒充绝对保证。
- C尚为MATLAB开关函数模型而非Simscape；真实模拟共模传递、REF电路仍未知。

### 下一可执行任务
统一结果摘要、当前SLX接口回归；补充20种子关键统计、饱和/故障/证书边界验收；C与平均闭环交叉和Simscape可用性检查；C移植接口和参数导出验证；正式报告与Issue2接口、requirements逐项证据审计。
目标仍ACTIVE；无新增硬件阻碍。

## 2026-11-01 远端核验与阶段选型交付

- 实际git fetch成功；远端work/issue-1-dq-current-loop与本地HEAD均为f2703e22cb4acde0b9caee0424f3e35aba2b7611。
- 28状态接口重新生成SLX并实际重跑7工况，全部既定断言通过。日志results/logs/foc_interface_regression.txt；能量残差2.77556e-17W。当前SLX与独立MATLAB调度器匹配场景交叉检查通过：植物最大差3.95253e-323，控制状态有限值差0；记录foc_crosscheck.mat。
- 新增docs/selection_decision.md，明确已实现算法、273组结果的每模型比较、实物电流仍受补偿残差影响、原始W保护先于电流环补偿、诊断模式不能当闭环通过，以及标定→模型选择→有效域→保护接入→独立电流验收的路径。
- README加入本机准确运行/查看方法；更新ESO文档已过期的“未进入完整闭环”表述。
- 阶段选型：O3离线辨识+固定补偿为主线；O1只在可信参考/激励条件研究更新；O2当前实现不作为直接替代。匹配M1/M2/M3各自补偿约1.4mA dq测量RMSE；错模型可约35mA。均为合成对象结果，不能作实物精度声明。
- 本次没有修改生产固件、提交、推送或创建PR；没有实物操作。全研究仍未完成：O1在线统一闭环、C/平均闭环交叉、完整移植接口及Issue2交付等继续保持待验收，不将阶段选型冒充最终完成。

## 2026-11-01 完整研究交付与最终审计

### 已完成
- 前一目标轮为实质进展：新增并执行统一算法闭环、C/平均闭环交叉、故障边界、REF根因候选计算、便携C接口和Issue2接口；本轮完成最终入口及交付核验。
- O1可信标定RLS/梯度后冻结78组，与原273组合并为351组、9方法、13工况；并非无独立参考在线盲估计。原有门控/漂移/吞掉真实电流反例继续保留。
- C闭环平均/开关交叉通过：40mA周期平均差2.413mA，25/12.5ns细化差1.809mA。明确是锁转子、等电感、无死区共同子集；完整模拟瞬态未冒充已知。
- 异常7项、C核心持续饱和/恢复、20种子复核通过。名义匹配20种子均值1.382mA，种子间SD0.0587mA，仅合成噪声重复统计。
- A1原理图确认REF共用10kΩ/10kΩ无图示缓冲；TI第15页解释源阻抗效应。未拟合的简化节点模型预测58.25mA/50%，形成优先验证候选，而不是确诊动态机理。
- 便携C候选5万组主机对照通过，非法校准/饱和/越域等退出；加正浮点余量。没有接入或修改生产固件。
- Issue2电流内环函数/S函数/SLX实际运行，独立线性对照误差1.39e-16；量化、残差、775us分段施加延迟、限幅和名义Kt均有接口说明。
- 最后START_HERE.m实际执行7工况、显示曲线并打开SLX；当前相位接口225低/250高独立计数公式验收通过。完整run_all正常结束；交付一致性审计PASS，正式要求逐项见docs/requirements.md。

### 文件
新增run_o1_closed_loop、run_control_boundaries、run_ref_hypothesis、run_switching_crosscheck、run_seed_validation、run_phase_features、issue2_drive_config/step/sfun、run_issue2_interface、compare_compensation、START_HERE；新增bias_compensation.h/.c、test_bias_compensation.py、audit_delivery.py、write_provenance.py；新增完整报告、最终推荐、实验设计、REF分析、移植和Issue2接口文档。更新run_all、README、相位配置、参数事实/假设及审计表。结果在research/pmsm-current-sensing/results。

### 实际运行命令与证据
- MATLAB：`addpath('research/pmsm-current-sensing/matlab'); run_all`，日志results/logs/run_all_20261101_183950.txt，末行REQUESTED_RESEARCH_STAGE_COMPLETE: all。
- MATLAB：`run_seed_validation`、非法stage拒绝检查；日志seed_validation.txt。
- MATLAB：`run_phase_features; run('research/pmsm-current-sensing/START_HERE.m')`，后接simulate_foc独立调度对照与compare_compensation；日志final_entry_phase.txt，进程退出0。
- Python：`python research/pmsm-current-sensing/c/test_bias_compensation.py`；`python research/pmsm-current-sensing/c/audit_delivery.py`；`python research/pmsm-current-sensing/c/write_provenance.py`。
- `git diff --check`及生产firmware/control_core无改动核对通过。图表已实际查看并修复热图下划线渲染。

### 发现与修复
- 饱和测试起初用0.2V母线，被真实C的9–15V契约拒绝；改为合法12V、强制误差测试饱和，未修改控制器限制。
- 开关粗步长导致14.47mA均值差、10.89mA细化差；细化网格后通过原判据，失败日志保留。
- 统一M3原隐式高窗明确写成250us；另验证默认固件225us低窗，不把两者混淆。
- 原始REF恒定假设被原理图证据修正；最终路线改为先核实REF/采样链，再对剩余偏移O3补偿。

### 完成范围与后续外部任务
附件13节A–E研究交付已逐项审计。最终结论属于附件允许的“物理模型仍未唯一确定”，已给最少补证据实验；不声明实物k或过流保护已认证。研究自审不是独立评审。
当前无剩余软件研究阻塞。下一项实物工作由用户安排：核对实板REF连接，记录REF/OUT同步波形与独立真实电流，补20kHz多duty/相位原始流，再确定实物模型和误差范围。MCU集成、目标构建及硬件运行均NOT RUN，未经授权不执行。没有commit/push/PR。

## 2026-11-01 用户授权提交与Issue范围复核
- 用户要求“把这些都提交，issue是不是可以关闭了”，本轮执行本地Git提交，不据此推送、合并或关闭Issue。
- 实际通过GitHub连接读取#28：其Phase C要求实测波形验证，第9节要求实测内环参数。当前研究交付满足附件允许的名义研究结论，但不满足原Issue全部实物验收；新增docs/issue28_acceptance_gap.md，使用Refs #28并保持Issue打开。
- 新增Git忽略规则排除slprj、slxc及slx.autosave；研究源码/SLX/报告/结果CSV、MAT轨迹、图表和日志保留为复核证据。编译产物与A1原始资料不入库。
- 提交前执行audit_delivery.py、test_bias_compensation.py、write_provenance.py与Git暂存检查。当前下一项仍为用户实物数据补证据，无自动硬件操作。
- 提交检查发现MATLAB日志行尾空白，已仅规范空白并刷新哈希，数值与失败记录保留。当前机器未配置Git身份，沿用仓库最近三次提交一致的GNYKCZ noreply身份，仅本次命令指定，不改全局配置。
