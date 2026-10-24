# Review 与 AI 工作流的迁移依据

核对日期：2026-10-23。定向查阅一次来源和近期研究，不是系统综述，不宣称搜尽所有最新理论。工程规则是结合本项目需求做的选择，不能等同于论文定理或验证成果。

## 迁移范围

源仓库：[secure_pid_hvac](https://github.com/GNYKCZ/secure_pid_hvac/tree/32fbb11224ad4e48588e041583288d5bac1be896)，基于该提交的 `review/` v4、`docs/issue_workflow_prompts.md` 与 `AGENTS.md`。

目标初始化基线：`c14d769fe5c18e997c0c59cf923a1d7e19e078d5`。读取 [实物 #1](https://github.com/GNYKCZ/motor-control/issues/1) 与 [仿真 #2](https://github.com/GNYKCZ/motor-control/issues/2)；本轮按用户要求暂不审查本地参考资料，不引入固件，也不声称已核实硬件规格/时序。

| 原工程机制 | 电机工程处理 |
|---|---|
| Design/Work/Review、授权、稳定 RV ID、最多三轮返工 | 保留，并补重复消息识别、证据版本绑定和自审/独立审查区分 |
| core 场景无关、状态显式、最小修改 | 迁移为硬件无关控制核、板级适配、模型与数据分析的职责边界 |
| Python 数值/安全协议专项 | 换成 C/FOC/实时路径及 MATLAB/Simulink/传动/接触检查，不复制 Beaver/Trunc |
| 固定 Python 全套命令 | 改为按实际工程入口与变更类型验证；缺少环境如实 NOT RUN |
| 实现证据优先级 | 分开“契约应有行为”和“实现实际行为”，实现不能改写需求 |
| 仿真/部署 claim 限制 | 明确构建、主机、仿真、板端和真实负载/接触的证据边界 |

两个仓库独立维护文档；不建立跨仓库运行依赖，不把源工程的提示词全文照搬成电机要求。

## AI 验证研究

1. Konstantinou、Tambon、Papadakis (2026)，[On the risk of coding before testing: An empirical study on LLM-based test generation workflow](https://arxiv.org/html/2607.05139v1)。arXiv 预印本；核对摘要和研究设计。研究关注生成实现把错误传递到后续测试的风险。采用：设计先写验收/oracle，审查测试是否独立和能否拒绝旧错行为。限制：研究中的编程任务不直接证明本项目固件/物理模型正确，也不提供这里的错误率。
2. Li 等 (2026)，[Independent Patch Verification for Coding Agents with a Bidirectional Reconstruct-and-Verify Framework](https://arxiv.org/html/2608.08950v1)。arXiv 预印本；核对摘要和 RETRACE 方法。采用：分别概括需求目标与 diff 实际行为，再核对偏离。限制：本流程保留完整需求上下文，不是原论文的盲审重建实验；不承诺论文中的性能增益。

不以模型数量或主观 Confidence 代替证据，不强制增加模型服务或自动执行器。

## 控制与工具资料

| 一次来源与核对范围 | 对本项目的作用 | 适用限制 |
|---|---|---|
| ST，[UM1052 Rev 9，STM32F PMSM single/dual FOC SDK v4.3](https://www.st.com/resource/en/user_manual/um1052-stm32f-pmsm-singledual-foc-sdk-v43-stmicroelectronics.pdf)，FOC/电流采样章节 | 支持把电流采样与 PWM 时序作为联合审查对象 | 2016 官方手册，是基础资料而非最新 SDK；不表示套件使用 MCSDK，也不能直接套寄存器/采样拓扑或宣称 F103C8 兼容特定 SDK 功能 |
| MathWorks，[Choose a Solver](https://www.mathworks.com/help/simulink/ug/choose-a-solver.html)，当前官方文档 | solver、积分步长与模型类型相关；需要选择与验证 | 阅读时页面为 R2026b；项目按实际安装版本记录，不自动升级或假定工具箱存在 |
| MathWorks，[C Caller](https://www.mathworks.com/help/simulink/slref/ccaller.html)，功能与限制 | C 控制核与主机模型对照有明确接口路径 | 实际版本/编译器支持需核实；该软件集成不是 HIL |
| Weigand、Gafur、Ruskowski (2021)，[Flatness Based Control of an Industrial Robot Joint Using Secondary Encoders](https://arxiv.org/abs/2101.04992)，摘要/发表信息 | Issue #2 的双惯量与输出编码器建模方向参考 | 论文模型含非线性摩擦/刚度及齿隙；本项目简化模型不是完整复现，不自动引入其控制器 |
| Mühlbauer 等 (2026)，[Passive Variable Impedance For Shared Control](https://arxiv.org/html/2604.20557v1)，§II–III 及适用假设 | 时变刚度/控制混合需要检查能量注入，不能从正刚度直接推出被动性 | 预印本，研究对象为 shared control；仅借鉴审查问题，不据此声称本项目受限、离散、延迟闭环已被证明被动 |

## 本轮不改变的研究范围

保留低速估计专题和 A/B/C 仿真对照。不因有新文献就增加强化学习、自适应控制、能量罐、额外硬件或第二套主仿真；具体算法改变需要明确任务与验证。工作流包本身不提供 MCU 执行时间、接触稳定性或安全认证结论。
