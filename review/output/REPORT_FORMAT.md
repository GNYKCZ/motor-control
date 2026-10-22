# Review 报告与 Finding 交接

## Scope

- Mode、仓库、Issue/子阶段、设计评论、PR、base/head 完整 SHA。
- 本轮验收与实际改动对应；自审或独立 Review 的真实身份。
- Completeness：COMPLETE / PARTIAL；按覆盖规范给 ledger。
- 验收矩阵：契约来源、oracle、实际命令/证据、环境/配置/版本、PASS / FAIL / NOT RUN / NOT APPLICABLE。

## Findings

按 P0→P3，只列证据门槛满足的问题。格式：

### RV-001 — [P?] 具体问题

- Location：path:line；模型用文件、块路径和可复核证据。
- Confidence：0–1 主观标记。
- Trigger / Evidence / Impact：可达条件、事实、具体后果。
- Violated contract：需求/方程/不变量来源。
- Verification：已排除的保护条件与验证结果。
- Suggested direction：在现有职责内恢复什么行为，不给强制 patch。
- Constraints / Non-goals：保持的接口、强度、范围与语义。
- Acceptance check / Suggested validation：可观察条件及最小验证。
- Residual uncertainty：仍缺什么证据。

复审保留 RV ID，分别核对 Fixed / Rejected / Already resolved / Superseded / Follow-up required / Blocked，不能把 Work 的状态直接当成结论。

## Audit 与实际验证

概述实际涉及的控制/时序、C/数值、模型/接触、实验/数据、测试 oracle 和需求/资料对应。仅列实际运行并观察到结果的命令；未运行写原因。区分软件与硬件证据，写明当前最多支持的 claim。

## 结论

- `PASS`：范围 COMPLETE，没有 P0–P2，当前验收与必要验证充分。
- `PASS WITH LIMITATIONS`：无 P0–P2，但有 P3 或明确限制；若 PARTIAL，只能表示已覆盖部分，不能视为可合并。
- `FAIL`：存在确认的 P0–P2 或明确未满足的验收。

合并建议单列：`APPROVE`（当前 head COMPLETE、无 P0–P2、必要证据齐全）、`REQUEST CHANGES`（需要当前 PR 修复的确认问题/未满足项）、`INCONCLUSIVE`（关键证据不可获得）。必要项 NOT RUN 时不得 APPROVE。

列遗留风险、未覆盖项、返工轮次和下一负责角色。发布前核对 base/head；新提交后旧报告仅作历史证据。建议合并不等于执行合并的授权，不等于硬件运行许可。
