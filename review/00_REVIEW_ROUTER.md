# Review Router — Motor Control

## 入口与范围

始终读取 [边界](core/BOUNDARY.md)、[证据门槛](core/EVIDENCE_AND_FINDINGS.md)、[验证覆盖](core/VERIFICATION_AND_COVERAGE.md)、[项目事实](reference/REVIEW_TARGET.md) 和 [报告格式](output/REPORT_FORMAT.md)。

- **Diff Mode**：指定 PR/branch/commit，记录 Issue、设计、base/head SHA；检查本次引入、暴露或恶化的问题及其必要上下文。
- **Target Mode**：指定当前模块/实验，检查范围内所有真实问题，不要求由最新 diff 引入。
- 当前无固件或模型的文档 PR，审查规范可操作性、链接、权限与契约一致性，不能宣称审完尚不存在的控制实现。

先从需求写预期行为，再从 diff/调用链概括实际变化，两者分别形成再核对。不要把 Work 的解释直接抄成审查结论，也不声称同一上下文自审具有独立性。

围绕工作包的用户可感知成果、关键路径/前置依赖与最小必要验收选择风险重点，不把清单同等展开。
每个Finding解释当前影响与本批必修理由，按[证据门槛](core/EVIDENCE_AND_FINDINGS.md)区分真实阻塞与可选建议；
主线优先不降低事实、关键测试和真实缺陷要求，必要检查通过后无新变更/失败/具体风险不继续扩测。

## 按风险加载

始终加载 [AI、实验与测试检查](checks/AI_AND_EXPERIMENTS.md)。

| 修改对象 | 追加检查 |
|---|---|
| C 控制算法、固件、ADC/PWM/SPI/DMA、构建、板端日志 | [固件与实时控制](checks/FIRMWARE_AND_FOC.md) |
| MATLAB/Simulink、传动/接触、仿真参数、C 模型集成 | [关节与仿真](checks/SIMULATION_AND_CONTACT.md) |
| 跨固件/仿真共享算法、周期/单位/参数交换 | 两项均加载 |
| 纯文档 | 加载其描述的领域检查，验证规则本身；执行证据按本次范围判断 |

完整项目 Target Review 覆盖所有实际存在且属于目标的路径。按 diff/目标 → caller/callee/消费者 → 测试/配置/入口 → 必要关联 → history 逐层展开，不盲扫整个参考资料目录。

## 两阶段审查

1. **候选发现**：优先检查采样/更新时序、单位/正方向、限制与状态、故障路径、物理模型、实验公平性、测试与结论。
2. **主动证伪**：逐候选检查调用方保证、初始化、其他层校正、有意适配、可达性、反例和已有测试。证据门槛全部满足后才成为 Finding。

只报告有具体影响的问题；优化建议、未来算法、风格和猜测不是确认缺陷。复审使用原 RV ID，重新验证修复及新回归；发布前再次确认 SHA。必要证据缺失给 INCONCLUSIVE，不给整体 PASS。

固定base/head与评审工作树，冻结仅作用于被审树；其他独立Git worktree中的已授权工作按
[工作流](../docs/issue_workflow_prompts.md)继续，不能移动被审分支或复制源码绕开冲突。
复审按风险与修复diff增量检查；核对输入哈希、配置、工具和环境后可引用适用的未变构建证据，
不能把旧整体PASS套到新head。尚未审完应继续，不以等待Design重派替代审查；真实证据不可得仍INCONCLUSIVE。
按已核实的人类交接授权接收Work直接交审；每次结论只通知Work（包括PASS/FAIL/INCONCLUSIVE），不抄送Design。
普通Finding直接交Work核查/修复再回本Review；Design仅负责起始设计。重大问题由Work暂停受影响修改、向用户说明影响和选项，由用户决定是否重启Design。
