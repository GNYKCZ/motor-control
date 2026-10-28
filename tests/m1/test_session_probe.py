"""离线串口替身：无serial模块、无端口/电源操作；核对原字节与明确的写入时刻。"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from session_probe import capture_session


class FakePort:
    def __init__(self, chunks, partial=False):
        self.chunks = iter(chunks)
        self.now = 100.0
        self.writes = []
        self.partial = partial

    def clock(self):
        return self.now

    def read(self, size):
        assert size == 256
        self.now += 0.05
        return next(self.chunks, b"")

    def write(self, data):
        self.writes.append((self.now, bytes(data)))
        return 5 if self.partial else len(data)


def main():
    root = Path(__file__).resolve().parents[2]
    dest = root / "build/session_probe_test"
    dest.mkdir(parents=True, exist_ok=True)
    samples = [b"M", b"P1R1,00000001,01,00,01,00000000,00000019,000009C4,0F49\r\n",
               b"P1R1,00000001,01,00,01,00000000,00000019,000009C4,0F49\r\n", b"tail"]
    results = []
    for mode in ("passive", "status", "partial"):
        port = FakePort(samples, mode == "partial")
        output = Path(tempfile.mkdtemp(prefix=mode + "_", dir=dest)) / "capture"
        failed = False
        try:
            capture_session(port, output, {"source_kind": "synthetic"}, status_seq=None if mode == "passive" else 1,
                            clock=port.clock)
        except RuntimeError:
            failed = True
        assert failed == (mode == "partial")
        assert (output / "raw.bin").read_bytes() == b"".join(samples)
        events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
        offset = 0
        for event in events:
            assert event["finished_monotonic"] >= event["started_monotonic"]
            assert event["rx_offset"] == offset
            if event["operation"] == "read":
                offset += event["length"]
        assert offset == len(b"".join(samples))
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
    (dest / "evidence.json").write_text(json.dumps({"result": "PASS", "cases": results,
        "source_sha256": {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in
         ["tools/session_probe.py", "tools/command_link.py", "tools/sensor_capture.py", "tests/m1/test_session_probe.py"]},
        "hardware": "NOT RUN"}, indent=2) + "\n", encoding="utf-8")
    print("PASS passive=0 writes; explicit STATUS once after2s; partial write no retry; raw/offset/timing/strict parse/existing directory")


if __name__ == "__main__":
    main()
