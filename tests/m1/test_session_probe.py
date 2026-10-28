"""离线串口替身：无serial模块、无端口/电源操作；核对原字节与明确的写入时刻。"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import subprocess
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from session_probe import capture_session


class FakePort:
    def __init__(self, chunks, partial=False, first_read_delay=0.05, write_error=False):
        self.chunks = iter(chunks)
        self.now = 100.0
        self.writes = []
        self.partial = partial
        self.first_read_delay = first_read_delay
        self.read_count = 0
        self.write_error = write_error

    def clock(self):
        return self.now

    def read(self, size):
        assert size == 256
        self.now += self.first_read_delay if self.read_count == 0 else 0.05
        self.read_count += 1
        return next(self.chunks, b"")

    def write(self, data):
        self.writes.append((self.now, bytes(data)))
        if self.write_error:
            raise RuntimeError("injected write failure; transmitted count unknown")
        return 5 if self.partial else len(data)


def main():
    root = Path(__file__).resolve().parents[2]
    dest = root / "build/session_probe_test"
    dest.mkdir(parents=True, exist_ok=True)
    samples = [b"M", b"P1R1,00000001,01,00,01,00000000,00000019,000009C4,0F49\r\n",
               b"P1R1,00000001,01,00,01,00000000,00000019,000009C4,0F49\r\n", b"tail"]
    results = []
    for mode in ("passive", "status", "partial", "exception"):
        port = FakePort(samples, mode == "partial", write_error=mode == "exception")
        output = Path(tempfile.mkdtemp(prefix=mode + "_", dir=dest)) / "capture"
        failed = False
        try:
            capture_session(port, output, {"source_kind": "synthetic"}, status_seq=None if mode == "passive" else 1,
                            clock=port.clock)
        except RuntimeError:
            failed = True
        assert failed == (mode in {"partial", "exception"})
        assert (output / "raw.bin").read_bytes() == b"".join(samples)
        events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
        offset = 0
        for event in events:
            assert event["finished_monotonic"] >= event["started_monotonic"]
            assert event["rx_offset"] == offset
            if event["operation"] == "read":
                offset += event["length"]
            else:
                assert event["requested_length"] == 23
                assert bytes.fromhex(event["requested_hex"]) == b"P1C1,00000001,01,A1E6\r\n"
                assert "raw_hex" not in event  # 整帧意图不得再冒充实际写入。
                expected = {"status": (23, "complete"), "partial": (5, "partial"), "exception": (None, "unknown")}[mode]
                assert (event["length"], event["write_report"]) == expected
                assert event["reported_written_hex"] == (None if mode == "exception" else b"P1C1,00000001,01,A1E6\r\n"[:expected[0]].hex())
        assert offset == len(b"".join(samples))
        summary = json.loads((output / "session_result.json").read_text())
        assert summary["write_report"] == {"passive": "not_attempted", "status": "complete",
                                            "partial": "partial", "exception": "unknown"}[mode]
        assert summary["write_completed"] == (mode == "status")
        assert bool(summary["error"]) == (mode in {"partial", "exception"})
        if mode == "passive":
            assert not port.writes
        else:
            assert port.writes == [(port.writes[0][0], b"P1C1,00000001,01,A1E6\r\n")]
            assert port.writes[0][0] >= 102.0
        records = [json.loads(line) for line in (output / "records.jsonl").read_text().splitlines()]
        assert records[0]["fields"] is None  # 不剥M，不修复损坏行。
        assert records[1]["fields"]["format"] == "P1R1"
        before = (output / "events.jsonl").read_bytes()
        try:
            capture_session(port, output, {}, clock=port.clock)
        except FileExistsError:
            pass
        else:
            raise AssertionError("existing directory overwritten")
        assert (output / "events.jsonl").read_bytes() == before
        results.append({"mode": mode, "writes": len(port.writes), "raw_bytes": offset, "result": "PASS"})
    # RV-001独立时间轨迹：首次read从100跳到105，跨过原104秒总期限。
    # 显式请求仍必须恰好写一次；写完继续满2秒，被动模式不能凭延迟产生写入。
    for passive in (False, True):
        port = FakePort([b"startup\n"], first_read_delay=5.0)
        output = Path(tempfile.mkdtemp(prefix="rv001_", dir=dest)) / "capture"
        result = capture_session(port, output, {"source_kind": "synthetic"},
                                 status_seq=None if passive else 1, clock=port.clock)
        events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
        assert (output / "raw.bin").read_bytes() == b"startup\n"
        if passive:
            assert not port.writes and not result["write_attempted"]
        else:
            assert port.writes == [(105.0, b"P1C1,00000001,01,A1E6\r\n")]
            assert result["write_completed"] and result["error"] is None
            write = next(e for e in events if e["operation"] == "write")
            assert events[-1]["finished_monotonic"] >= write["finished_monotonic"] + 2.0
        results.append({"mode": "delayed_passive" if passive else "delayed_status",
                        "writes": len(port.writes), "result": "PASS"})
    for mode in ("complete", "partial", "exception", "delayed"):
        port = FakePort([b"before\n"], partial=mode == "partial", write_error=mode == "exception",
                        first_read_delay=5.0 if mode == "delayed" else 0.05)
        output = Path(tempfile.mkdtemp(prefix="stop_", dir=dest)) / "capture"
        failed = False
        try:
            capture_session(port, output, {"source_kind": "synthetic"}, stop_seq=2, clock=port.clock)
        except RuntimeError:
            failed = True
        assert failed == (mode in {"partial", "exception"})
        assert len(port.writes) == 1 and port.writes[0][0] >= 102.0
        assert port.writes[0][1] == b"P1C1,00000002,02,1A67\r\n"  # 独立STOP黄金帧。
        event = next(json.loads(l) for l in (output / "events.jsonl").read_text().splitlines()
                     if json.loads(l)["operation"] == "write")
        summary = json.loads((output / "session_result.json").read_text())
        metadata = json.loads((output / "metadata.json").read_text())
        for item in (event, summary, metadata):
            assert item["requested_command"] == "stop" and item["requested_sequence"] == 2
        assert summary["stop_requested"] and not summary["status_requested"]
        count = 5 if mode == "partial" else None if mode == "exception" else 23
        assert event["length"] == count
        report = "partial" if mode == "partial" else "unknown" if mode == "exception" else "complete"
        assert event["write_report"] == summary["write_report"] == report
        assert summary["write_attempted"] and summary["write_completed"] == (count == 23)
        assert bool(summary["error"]) == failed
        assert event["reported_written_hex"] == (None if count is None else b"P1C1,00000002,02,1A67\r\n"[:count].hex())
        if mode == "delayed":
            assert port.writes[0][0] == 105.0 and port.now >= 107.0  # RV-001也覆盖STOP。
        results.append({"mode": "stop_" + mode, "writes": 1, "result": "PASS"})
    port = FakePort([])
    try:
        capture_session(port, dest / "mutually_exclusive", {}, status_seq=1, stop_seq=2, clock=port.clock)
    except ValueError:
        pass
    else:
        raise AssertionError("API accepted STATUS and STOP together")
    assert not port.writes
    cli = subprocess.run([sys.executable, "-B", str(root / "tools/session_probe.py"),
                          "--status-seq", "1", "--stop-seq", "2"], capture_output=True, timeout=10)
    assert cli.returncode == 2  # argparse在任何serial导入/打开之前拒绝互斥参数。
    (dest / "evidence.json").write_text(json.dumps({"result": "PASS", "cases": results,
        "source_sha256": {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in
         ["tools/session_probe.py", "tools/command_link.py", "tools/sensor_capture.py", "tests/m1/test_session_probe.py"]},
        "hardware": "NOT RUN"}, indent=2) + "\n", encoding="utf-8")
    print("PASS passive/STATUS/STOP golden/single-write/mutual exclusion; RV-001 and RV-002; no retry or hardware")


if __name__ == "__main__":
    main()
