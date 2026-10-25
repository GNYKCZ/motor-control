# CubeMX 初始化修正验证

Refs #1/#7，依据 [设计 v1.4](https://github.com/GNYKCZ/motor-control/issues/7#issuecomment-6393651536)。
起始 main `896ac935cf6e73c3fa94e8feb7a6a3e896d2711d`；分支 `work/issue-1-cubemx-init`。
本文件记录 Work 软件证据；提交后再验证，并由 PR 绑定实际完整 head 和独立 Review 结果。
首次实施检查含未提交修改，不能把这些检查冒称起始 main 的能力。

## 验收依据与实际执行

| 验收 | 独立依据/工况 | 证据与结果 |
|---|---|---|
| A14实际生成/再生成 | 本机CubeMX6.12.0、FW_F1 1.8.6；目标.ioc；每次新空输出，逐文件比较，main仅保留同名USER CODE | `tools/regenerate_m1.py`，16个文件一致，PASS |
| A06禁用生命周期 | M1契约：BOOT/DISABLED/FAULT、首帧1ms、故障锁存、无自动恢复；引脚普通GPIO低、MOE/CCER=0 | 原M1真实runtime主机回归PASS；人工核对生成GPIO/TIM、main先建立BOOT与board门控；功率高阻关断仍未满足 |
| A07完整采样 | 协议多项式长除法oracle、全部16384角度/393216单bit破坏、半帧/状态/边界/时间回绕、交错快照 | 原 `tools/test_m1.py` 五组PASS；board双JEOC/DMA发布与临界区算法保留；实际电气波形NOT RUN |
| A09目标构建 | STM32F103C8标称64KiB/20KiB、ARM6.13.1/C11/O2/MicroLIB；实际源闭合与链接入口 | 命令行和MDK构建PASS；26个C文件+startup；生成时钟及9个外设入口进入实际链接图 |
| A12/A13来源/可读性 | 例程.ioc→目标.ioc→16个生成文件→board/app；ST Header、原字节/许可、USER CODE和IRQ所有权 | README/SOURCES逐项映射，47项厂家原文件哈希、文档链接与差异检查PASS |

运行入口从仓库根执行，具体工具位置见 [固件README](../firmware/stm32f103/README.md)：

```powershell
python tools/regenerate_m1.py --cubemx <CubeMX.exe路径> --package <FW_F1_1.8.6目录>
python tools/build_m1.py --compiler-bin <ARMCLANG_bin目录>
python tools/test_m1.py --gcc <gcc.exe路径>
python tools/generate_m1_mdk.py --check
python tools/verify_m1.py
```

MDK另按README只执行Rebuild，不下载。原M2a核心、测试和脚本未修改，也未接入固件。
软件检查不是板端实测；未运行旧main的CubeMX入口，因为旧版不存在该入口/目标.ioc。

## 工具、生成与输入绑定

- Python3.12.14、CubeMX6.12.0/DB6.0.120、Cube FW_F1 1.8.6；ARM Compiler6.13.1、MDKPlus5.29、GCC8.1.0。
- CubeMX完整 `project generate` 曾触发登录组件NavigationException，未成功；不把其返回0当作成功。
  实际交付用官方UM1718的 `generate code` 入口，生成初始化源码，再接入现有唯一构建。
  无需下载包；脚本显式指定本机包，Java显式UTF-8防止中文路径损坏。
- 实际命令脚本、完整输出及工具EXE/包标识/目标IOC/脚本/16项原始及整合哈希保存在
  `build/cubemx_<时间>/`。干净比较首次通过目录为 `build/cubemx_20261025_144634_291254/`；
  提交后的最新目录以PR交接为准。厂家生成区只统一LF；main的ST Header照常对比。
- `build/m1/build_evidence.json` 记录实际HEAD、状态、所有固件输入、编译命令和二进制哈希；
  `build/m1/mdk_build.log` 为真实MDK结果；`build/host_m1/test_evidence.json` 为主机输入/脚本哈希。
  日志留本地忽略目录，不上传工具安装资料或机器个人信息。

## 配置与运行操作核对

生成：HSE8MHz×9→72MHz，APB1=36MHz/定时器72MHz，APB2=72MHz，ADC=12MHz。
ADC1 PA0/ADC2 PA1，7.5周期，dual injected simultaneous，ADC1由TIM1 TRGO触发。
TIM1 CMS3/PSC0/ARR1799/RCR3/update TRGO，无输出通道；TIM2 ITR0 reset/PSC71/CCR1=50；
TIM4 PSC71、ARR65535。SPI1 CPOL低/CPHA2、8bit、分频64，双DMA normal/byte/MINC。
USART2 115200/8N1 TX，PA3仍是输入；PB12 nFAULT普通输入上拉，未启用硬件Break。

board中删去手写外设参数，只保留启动校准、SPI SPE、清UG/溢出标志、起动计数器/事件和每帧DMA。
NVIC优先级与使能由生成MSP/dma拥有；board/Cortex自编IRQ拥有标志消费和异常门控，生成IRQ不编译。
没有定时器参数的手工覆盖，没有HAL PWM启动、自动对齐或电流控制调用。
ADC2的注入触发字段由CubeMX双模式从机模板留零；HAL在双注入模式启动从机只使能ADC，
由ADC1主机产生同步转换，不依赖ADC2独立外部触发选择；不为此复制第二套初始化。

## 资源、警告与结论边界

命令行map：Code13192 B、RO436 B、RW12 B、ZI2140 B；ROM总13640 B、RAM总2152 B。
fromelf将对齐填充归到数据，显示RO440/RW16/ZI2136，总占用一致。
MDK同为0 Error(s)、1 Warning(s)，资源与命令行一致；唯一警告是保留原HAL ADCEx1216行多余括号。
自编与生成Core代码按-Wall/-Wextra/-Werror构建，无新警告。

本批不宣称实测WCET、栈峰值、实际DMA/ADC先后、串口波形或电气保护有效。
没有烧录、上电或启动电机。PA8/9/10低电平为停止调制/低侧制动；EN/nSLEEP固定接高的
功率高阻关断缺口仍未满足，实际U4型号和复位瞬态仍需硬件确认。当前阶段不接M2b、不自动合并。
