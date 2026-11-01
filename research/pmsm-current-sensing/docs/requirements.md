# 完成审计清单

依据用户附件0–13节，尤其13节A–E；对照实际代码、输出及日志人工自审，不冒充独立审查。2026-11-01。研究完成不包括真实硬件根因确诊或补偿部署；附件明确允许物理模型尚不能唯一确定，但要求完成可执行研究和最少补测说明。

| 验收项 | 当前证据与范围 | 判定 |
|---|---|---|
| A手算/量化/延迟/噪声/饱和/SLX | run_model_a；model_a_checks.csv 7项；model_a.mat；sensor_bias_model.slx实际运行，解析与独立链对照 | 完成 |
| M1/M2/M3辨识性 | run_identification；identifiability.csv的秩2/1/2/3；条件数、单矢量零序反例；model_selection.csv | 完成 |
| O3 LS/区间/留出/外推 | fit_parameters.csv、identification.mat；120组o3_mismatch.csv，47接受73拒绝；无实物置信度冒称 | 完成 |
| O1梯度/RLS/激励/门控/漂移 | adaptation_stress.csv；unsafe_adaptation_counterexample.csv；在线更新吸收真实电流反例；o1_closed_loop.csv另78组可信标定后冻结 | 完成 |
| O2偏置与物理扰动ESO | run_eso、eso_comparison.csv 120组、秩2/3不可观测方向；闭环bias_ESO冷启动失败保留 | 完成 |
| B实际C核心/平均dq电机/保护 | run_model_b oracle约6.2e-8；run_foc_simulink 7组，原W/未知校准拒绝；完整SLX、能量及调度交叉。保护是关键逻辑镜像，不是MCU全状态仿真 | 完成 |
| 统一指标与所有基线 | unified_closed_loop.csv 351组、comparison_summary.csv 117行；A的189组对照；未运行时不填零误差，O1运行段冻结有明示 | 完成 |
| 至少10项工况/稳定性/失效 | experiment_design.md逐项映射；54组局部极点；7项异常+持续饱和恢复；20额外种子；phase_features.csv低225/高250 | 完成 |
| C开关/死区/共模/采样 | run_model_c 24组，KCL/无直通/电压均值/步长细化；switching_crosscheck.csv两闭环工况；理想开关状态模型，非Simscape寄生模型 | 完成 |
| 机制分级/原理图/最小补测 | 原理图实际渲染核对；TI第15页REF源阻抗要求；ref_hypothesis.csv未调参预测；ref_root_cause.md与experiment_design.md。未确诊动态机理 | 完成 |
| STM32建议/有效域/退出 | stm32_integration.md；bias_compensation.h/.c主机5万组与非法输入检查；portable_c_checks.json。实际MCU集成/构建NOT RUN、不属于本研究部署授权 | 完成研究交付 |
| Issue2可复用内环 | issue2_drive_step/sfun、issue2_drive_demo.slx实际运行，解析对照误差1.39e-16；issue2_interface.csv与接口文档；未复制机械对象 | 完成 |
| 可复跑入口/数据/图/报告 | run_all从A至C实际结束；START_HERE入口单独执行7工况通过并显示曲线/打开SLX；当前相位接口的独立调度对照通过；三份SLX与完整数据/日志/图/报告 | 完成 |
| 最终模型/参数/适用域/限制 | final_recommendation.md；parameters.m；fit_parameters.csv区分合成；实物20kHz系数及误差界TBD；未以名义参数代替实测 | 完成 |

## 对最终条件A–E的覆盖

- A模型：前两行及参数来源；123mA/60°重现是数学一致性，不作为真实电流零的证明。
- B算法：O1/O2/O3及全部简单基线均比较；不适用方案以失败与不可辨识证据淘汰，不为凑数关闭保护。
- CFOC：真值/测量/参考分开记录，局部稳定与饱和分析、错误模型和温漂失效均保留。
- D决策：硬件REF优先验证＋剩余O3固定补偿；明确不推荐在线盲估计，给出数学模型、参数状态、有效域、退出、移植与实测门槛。
- E交付：源文件、入口、仿真数据、图、报告、Issue2接口均存在且已执行对应核心路径；最后入口回归正常退出。相位字段新增的默认250us与旧高窗特征数学等价，另测225us低窗；未以旧结果覆盖未验证的新工况。

## 复现日志

- results/logs/run_all_20261101_183950.txt：全A/B/C及C候选测试，末行REQUESTED_RESEARCH_STAGE_COMPLETE: all。
- results/logs/seed_validation.txt：20种子补充与非法stage拒绝检查。
- results/logs/final_entry_phase.txt：显式相位与START_HERE最新入口回归。
- results/data/provenance.json：本地未提交源码/结果哈希与基线；不等同Git提交或独立评审。

## 外部验证边界

独立真实电流、REF同步动态、实板原理图一致性、实际电机参数、温度/母线有效域及MCU性能仍待用户硬件实验。本研究已按附件允许的“模型尚未唯一确定”结论完成可执行任务；这些是实物部署门槛，不写成已通过，也不虚构最终k。
