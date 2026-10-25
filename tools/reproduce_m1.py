"""将当前已声明的 M1 文件复制到全新构建输入目录，核对可独立重建且 HEX 完全一致。

用途仅为 A09 复现验证，不用于绕开共享工作区/Review 冻结。无 Git/参考目录复制。
目标目录已存在则拒绝，不覆盖/删除任何目录；只构建，不触碰硬件。
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path


def main():
    """复制已构建输入清单的精确字节，执行同工具链重建，比较输入哈希及固件 HEX。"""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler-bin", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=root / "build/repro_m1")
    arguments = parser.parse_args()
    target = arguments.output.resolve()
    if target.exists():
        raise SystemExit("复现目标已存在，请指定另一个全新 --output；不会覆盖原内容。")
    original = json.loads((root / "build/m1/build_evidence.json").read_text(encoding="utf-8"))
    workspace = target / "workspace"
    for name, expected_hash in original["inputs"].items():
        source = root / name
        content = source.read_bytes()
        if hashlib.sha256(content).hexdigest() != expected_hash:
            raise SystemExit("来源已在构建后变化，请先重建：" + name)
        destination = workspace / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
    (workspace / "tools").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(root / "tools/build_m1.py", workspace / "tools/build_m1.py")
    subprocess.run([sys.executable, str(workspace / "tools/build_m1.py"),
                    "--compiler-bin", str(arguments.compiler_bin.resolve())], check=True)
    repeated = json.loads((workspace / "build/m1/build_evidence.json").read_text(encoding="utf-8"))
    original_hex = (root / "build/m1/m1_sampling.hex").read_bytes()
    repeated_hex = (workspace / "build/m1/m1_sampling.hex").read_bytes()
    if original["inputs"] != repeated["inputs"] or original_hex != repeated_hex:
        raise SystemExit("复现不一致：保留两份构建产物供检查，不标 PASS。")
    evidence = {"source_head": original["head"], "source_worktree_status": original["worktree_status"],
                "input_count": len(original["inputs"]), "source_copy_head": repeated["head"],
                "hex_sha256": hashlib.sha256(original_hex).hexdigest(),
                "inputs_match": True, "hex_bytes_match": True,
                "copied_build_script_sha256": hashlib.sha256((root / "tools/build_m1.py").read_bytes()).hexdigest()}
    (target / "repro_evidence.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("PASS clean source copy: all input hashes and complete HEX bytes match")


if __name__ == "__main__":
    main()
