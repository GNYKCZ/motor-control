"""只读核对 M1 的源清单、厂商文件完整性、MDK 配置和 Markdown 相对链接。

该脚本不证明硬件行为，不代替实际目标构建/主机测试/独立 Review。
输入为本仓库已存在文件，输出为明确检查结果；失败立即非零退出，不修改项目。
"""
import hashlib
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote


def require(condition, message):
    """检查独立文件约束；失败给出原因，避免 Python -O 关闭 assert 后跳过验证。"""
    if not condition:
        raise SystemExit(message)


def main():
    """检查实际存在、路径闭合、原字节哈希和文档链接，不读取参考目录/工具安装资料。"""
    root = Path(__file__).resolve().parents[1]
    firmware = root / "firmware/stm32f103"
    project = json.loads((firmware / "project_sources.json").read_text(encoding="utf-8"))
    for relative in project["sources"] + project["includes"] + [project["startup"], project["scatter"]]:
        path = (firmware / relative).resolve()
        require(path.is_relative_to(firmware) and path.exists(), "missing/outside source: " + relative)
    source_names = [Path(name).stem for name in project["sources"]]
    require(len(source_names) == len(set(source_names)), "object filename collision")
    document = ET.parse(firmware / "MDK-ARM/m1_sampling.uvprojx")
    mdk_paths = [node.text for node in document.findall(".//FilePath")]
    expected_paths = ["../" + name for name in [project["startup"], *project["sources"]]]
    require(sorted(mdk_paths) == sorted(expected_paths), "MDK source list mismatch")
    require(document.findtext(".//OutputName") == "m1_sampling", "MDK output name mismatch")
    require(document.findtext(".//LDads/ScatterFile") == "m1_sampling.sct", "MDK scatter mismatch")
    subprocess.run([sys.executable, str(root / "tools/generate_m1_mdk.py"), "--check"], check=True)
    print(f"PASS source closure: {len(source_names)} C files + startup, all under firmware")
    dependency_files = list((root / "build/m1").glob("*.d"))
    if dependency_files:
        require(len(dependency_files) == len(source_names), "incomplete dependency evidence; rebuild target")
        build_evidence = json.loads((root / "build/m1/build_evidence.json").read_text(encoding="utf-8"))
        for name, expected_hash in build_evidence["inputs"].items():
            require(hashlib.sha256((root / name).read_bytes()).hexdigest() == expected_hash,
                    "target input changed after build: " + name)
        compiler_headers = Path(build_evidence["commands"][0][0]).parent.parent / "include"
        header_paths = set()
        for dependency_file in dependency_files:
            for line in dependency_file.read_text(encoding="utf-8").splitlines()[1:]:
                name = line.strip().removesuffix("\\").strip()
                if not name:
                    continue
                path = Path(name).resolve()
                require(path.exists() and (path.is_relative_to(firmware) or path.is_relative_to(compiler_headers)),
                        "external/missing dependency: " + name)
                header_paths.add(path)
        print(f"PASS actual compiler dependency records: {len(header_paths)} source/header paths")
    host_evidence_file = root / "build/host_m1/test_evidence.json"
    if host_evidence_file.exists():
        host_evidence = json.loads(host_evidence_file.read_text(encoding="utf-8"))
        require(host_evidence["exit_code"] == 0, "latest host software test did not pass")
        for name, expected_hash in host_evidence["source_sha256"].items():
            require(hashlib.sha256((root / name).read_bytes()).hexdigest() == expected_hash,
                    "host test input changed: " + name)
        require(host_evidence.get("test_script_sha256") == hashlib.sha256((root / "tools/test_m1.py").read_bytes()).hexdigest(),
                "host test script changed after test")
        print("PASS current host-test source/header/script hashes match the saved PASS")

    provenance = json.loads((firmware / "vendor_sources.json").read_text(encoding="utf-8"))
    for record in provenance["files"]:
        path = (firmware / record["destination"]).resolve()
        require(path.is_relative_to(firmware), "vendor path escapes firmware")
        require(hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"],
                "vendor hash mismatch: " + record["destination"])
    print(f"PASS vendor provenance: {len(provenance['files'])} exact file hashes")

    own_paths = [root / ".gitattributes", root / ".gitignore", root / "README.md", root / "docs/m1_sampling_contract.md",
                 root / "docs/m1_validation.md", *list((root / "tools").glob("*_m1*.py")),
                 root / "tools/generate_m1_mdk.py", root / "tests/m1/test_sampling.c"]
    own_paths += [path for path in firmware.rglob("*") if path.is_file()
                  and "Drivers" not in path.relative_to(firmware).parts
                  and "build" not in path.relative_to(firmware).parts
                  and "DebugConfig" not in path.relative_to(firmware).parts
                  and path.name not in {"system_stm32f1xx.c", "startup_stm32f103xb.s"}]
    for path in set(own_paths):
        content = path.read_bytes().decode("utf-8", errors="strict")
        require(content.endswith("\n"), "missing newline: " + str(path.relative_to(root)))
        for number, line in enumerate(content.splitlines(), start=1):
            require(line == line.rstrip(), f"trailing whitespace: {path.relative_to(root)}:{number}")
    print(f"PASS UTF-8/newlines/whitespace: {len(set(own_paths))} authored files")

    documents = [root / "README.md", root / "docs/m1_sampling_contract.md", root / "docs/m1_validation.md",
                 firmware / "README.md", firmware / "SOURCES.md"]
    checked_links = 0
    for path in documents:
        content = path.read_text(encoding="utf-8")
        require(content.count("```") % 2 == 0, "unclosed code fence: " + str(path))
        for destination in re.findall(r"\]\(([^)]+)\)", content):
            if "://" in destination or destination.startswith("#"):
                continue
            target = (path.parent / unquote(destination.split("#")[0])).resolve()
            require(target.is_relative_to(root) and target.exists(),
                    f"bad relative link: {path.relative_to(root)} -> {destination}")
            checked_links += 1
    print(f"PASS Markdown fences and {checked_links} relative links")


if __name__ == "__main__":
    main()
