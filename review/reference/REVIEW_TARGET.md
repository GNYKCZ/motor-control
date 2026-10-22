# 项目事实与需求对应

迁移基线为 motor-control commit `c14d769fe5c18e997c0c59cf923a1d7e19e078d5`：仅初始化文件；本轮未审阅用户本地参考固件。后续审查先更新实际代码事实，不能永久沿用“没有实现”的判断。

| 契约 | 当前路线图边界 | Review 重点 |
|---|---|---|
| [#1 实物项目](https://github.com/GNYKCZ/motor-control/issues/1) P0–P3 | 现有单电机/单磁编码器，不新增硬件；硬件规格目前来自 Issue 的资料描述，需后续实物核实 | FOC/三环、低速估计、采样时序、柔顺电流律、状态/故障、真实日志 |
| [#2 仿真项目](https://github.com/GNYKCZ/motor-control/issues/2) S0–S5 | MATLAB/Simulink 主线；两惯量、双编码器、单关节接触；Python 辅助 | 物理/单位、延迟、多速率、公平对照、收敛、不确定性与 C 模块对照 |

Review 从本轮子任务提取 Objective、In/Out of Scope、Dependencies、Acceptance Criteria；主路线图的未开始阶段不是当前 PR 的缺陷。当前阶段要求但缺失的内容是未满足验收，不可静默移到下一阶段。

论文/官方资料对应使用 `Exact / Equivalent / Adapted / Simplified / Missing / Not applicable` 并给依据。Adapted/Simplified 本身不是 Bug，错误在于与声明不一致或用简化证据支持更强结论。来源和适用限制见 [研究依据](../../docs/ai_review_basis.md)。

新的论文不是新的强制验收，也不授权复制算法或更改物理假设。实际契约冲突必须记录，不能用“代码已经这样写”消解需求。
