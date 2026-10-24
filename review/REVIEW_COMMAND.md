# 可复制的审查指令

## PR 审查

```text
请只读审查 Issue <URL> 的 PR <URL>，子阶段为 <P?/S?/文档任务>。
设计评论为 <URL/无>。从 review/00_REVIEW_ROUTER.md 开始，按 Diff Mode 加载规则。
核对真实 base/head 和工作区，先从需求提取验收，再从 diff/调用链读出实际行为。
按适用领域检查固件/FOC、仿真/接触、数据与测试的独立性，主动证伪候选问题。
不要修改文件、补测试、烧录或启动电机。报告实际命令与证据等级、缺失项、稳定 RV ID、
有界修复方向及可观察验收。缺关键证据不能给整体 PASS 或 APPROVE。
发布/自动交接仅在我此前已明确授权对应目标时执行；否则在当前聊天给出完整报告。
```

## 复审

```text
请复审同一 Issue/PR，上一轮报告 <URL>，旧 head <SHA>，当前 head <SHA>，
累计返工轮数 <n>。重新读取实际远端状态，不以传入 SHA 代替核实。
逐项核对原 RV Finding 和 Work 回复，检查修复 diff 的新回归和验证 oracle。
沿用 review/00_REVIEW_ROUTER.md；不要重置编号/轮数，不自动合并或运行硬件。
```
