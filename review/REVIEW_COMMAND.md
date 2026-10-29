# 可复制的审查指令

## PR 审查

```text
请只读审查 Issue <URL> 的 PR <URL>，子阶段为 <P?/S?/文档任务>。
设计评论为 <URL/无>。从 review/00_REVIEW_ROUTER.md 开始，按 Diff Mode 加载规则。
固定真实base/head与评审工作树绝对路径，先从需求提取验收，再从diff/调用链读出实际行为；只冻结被审树。
按适用领域检查固件/FOC、仿真/接触、数据与测试的独立性，主动证伪候选问题。
不要修改文件、补测试、烧录或启动电机。报告实际命令与证据等级、缺失项、稳定 RV ID、
有界修复方向及可观察验收。缺关键证据不能给整体 PASS 或 APPROVE。
发布/自动交接仅在我此前已明确授权对应目标时执行；否则在当前聊天给出完整报告。
在有效人类授权内，每次结论只通知Work（含PASS/FAIL/INCONCLUSIVE），不抄送Design；普通Finding直接回Work。
尚未审完继续完成，必要证据确实不可得才报告INCONCLUSIVE及补证据方式，不等Design重派。
```

## 复审

```text
请复审同一 Issue/PR，上一轮报告 <URL>，旧 head <SHA>，当前 head <SHA>，
累计返工轮数 <n>。重新读取实际远端状态，不以传入 SHA 代替核实。
逐项核对原 RV Finding 和 Work 回复，检查修复 diff 的新回归和验证 oracle。
引用首交契约，后续只传版本/差异/RV/证据；按风险增量复核，核对后可复用未变输入的旧构建证据，不继承旧整体PASS。
沿用 review/00_REVIEW_ROUTER.md；不要重置编号/轮数，不自动合并或运行硬件。
```
