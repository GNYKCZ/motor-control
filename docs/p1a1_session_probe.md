# P1a1 同一串口会话诊断工具

用于区分开口积压与命令期间的破帧，不修复/剥除损坏前缀，不改变固件或线协议。
默认不打开串口；实际端口使用由用户另行安排。STATUS已有一次用户板测成功回执；本次新增STOP仅完成离线准备，未运行下面现场模板。

从仓库根目录运行预览，无serial依赖、无端口访问：

```powershell
python tools/session_probe.py
python tools/session_probe.py --status-seq 5
```

第一条预览全程被动接收4秒；第二条预览先只读2秒、一次STATUS seq5、写完后再读2秒。
独立参数--stop-seq可指定一次STOP，与--status-seq互斥；两者都省略则完全不发送。
工具不支持ARM/PULSE，不自动重发，不清空串口缓存、不复位。
序号5仅为示例，现场应选择该启动会话尚未使用且按协议向前的新序号。

以下是待用户安排的单次现场模板，不能在准备阶段执行。保持现有USB/DAP接法、12V断开，
先在VOFA断开COM9以释放占用；不拔插/转轴或额外按Reset。pyserial需3.5，
可使用此前项目本地依赖build/p0_closeout/python_deps。参数填写真实条件：

```powershell
$env:PYTHONPATH = (Resolve-Path 'build/p0_closeout/python_deps').Path
python tools/session_probe.py --port COM9 --output build/p1a1_session_board_01 --before 2 --after 2 --status-seq 5 --firmware-sha 78bceec2a7b847fd64071f1886e42917797ffe86 --board '现有板卡' --motor '电机接线未改动' --supply '仅USB/DAP供电，12V断开'
```

同时省略--status-seq和--stop-seq即全程只读，且仍只开关一次连接。output必须不存在；不能覆盖旧记录。
DTR/RTS在开口前设低，但具体DAP仍可能有开口控制线效应；记录中不能假定绝无重启。
主机read有50ms超时，收尾可以多出一次read等待；不是MCU硬实时采样器。
若系统调度或一次read使预接收超过原计划总时长，显式STATUS/STOP仍发送一次，
从写完时重新计满after接收窗口；不按原总期限静默跳过请求。异常/部分写入仍不重试。

输出包含raw.bin、records.jsonl、metadata.json，以及：

- events.jsonl：每次read/write的started/finished_monotonic、请求长度与返回长度、操作前rx_offset。
  rx_offset是原始接收流的0基偏移，空read长度0也保留。write另记tx_offset=0，
  requested_hex始终为请求的23B整帧；length是接口返回计数，reported_written_hex仅为该计数对应的前缀。
  返回23时write_report=complete，返回5时为partial且仅记录5B前缀；异常/无有效计数时为unknown，
  length与reported_written_hex均为null，不能把未知当0或整帧已发送。
- session_result.json：是否明确要求STATUS或STOP、是否尝试/完成一次写入、总接收字节、异常。
  部分写入/异常不重发；没有完整回执也不改成成功。
  write_completed只代表接口报告整帧写入，不证明字节已上电线或板端收到；未知状态另见write_report。

电脑单调时间只能界定系统调用前后，不能当作USB包或MCU字节到达时间。
read(256)返回边界也不是USB64B包的直接观测；偏移只能用于关联，不能据此单独确诊DAP。
所有字节先原样交保存层，严格解析保持；坏行/半行也留档，不能以CRC负载提取代替原物理行验收。
时间事件在正常结束/异常清理时写出；强杀进程不保证事件表完整，原字节保存沿用现有逐块写入。

离线验证：`python -B tests/m1/test_session_probe.py`，只用FakePort和可控单调时间，
验证无显式序号时0次write、显式参数时2秒后恰好一次黄金STATUS、部分写入不重试、
原字节/offset/时间对应、坏行不修饰、现有输出目录不覆盖。
RV-001回归另用首次read延迟5秒跨过默认4秒总期限：显式模式仍恰好一次STATUS并继续2秒，
被动模式仍为0次写入；旧版本会静默不发送，已留离线失败对照证据。
RV-002回归覆盖完整23B、部分5B、写入异常：分别核对整帧意图与接口报告前缀/未知值，均不重试。
证据在build/session_probe_test/evidence.json。工具只新增主机Python与文档，固件/IOC/MDK未改，
目标固件构建为NOT APPLICABLE；本次STOP板端使用NOT RUN。取得证据后由Design决定是否安排现场使用。

## 单次STOP准备

仅预览：`python tools/session_probe.py --stop-seq 5`，不会打开端口。
现场另安排后，将上面的STATUS模板改为--stop-seq和未使用的新序号，使用全新output目录；
依旧先只读2秒、只写一次23B STOP、写完再读2秒。不能同时指定两种命令。
STOP黄金帧seq2为 `P1C1,00000002,02,1A67` 后接CRLF，序号2仅用于离线测试。
事件、结果与metadata均记录requested_command和requested_sequence，避免把STOP误认成STATUS。
完整/部分/异常的接口报告语义与RV-002保持；无回执不能当作已停机证明。
USB-only下该项只验证命令往返与软件停止路径，不证明功率桥高阻、电机电气停机或独立急停。
新增STOP的完整、部分、异常和5秒预读延迟测试，并校验API/CLI互斥；未执行实际STOP板测。
