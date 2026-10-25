"""从同一项目源清单生成最小 MDK 工程，不依赖或复制参考工程的 .uvprojx。

只更新本项目生成的 MDK-ARM/m1_sampling.uvprojx；--check 只读核对一致性。
编译/链接不含下载器、Flash 算法和自动运行配置，硬件操作仍须用户另行安排。
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    """将清单映射到 MDK 的相对路径和 ARM Compiler 6.13.1/O2/微库选项。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    firmware = Path(__file__).resolve().parents[1] / "firmware/stm32f103"
    project = json.loads((firmware / "project_sources.json").read_text(encoding="utf-8"))
    document = ET.Element("Project", {"xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
                                     "xsi:noNamespaceSchemaLocation": "project_projx.xsd"})

    def fields(parent, values):
        """向指定 XML 节点添加简单字段；仅用于生成工程元数据，不读取外部命令。"""
        for name, value in values.items():
            ET.SubElement(parent, name).text = str(value)

    fields(document, {"SchemaVersion": "2.1", "Header": "### uVision Project, (C) Keil Software"})
    target = ET.SubElement(ET.SubElement(document, "Targets"), "Target")
    fields(target, {"TargetName": project["target"], "ToolsetNumber": "0x4", "ToolsetName": "ARM-ADS",
                    "pCCUsed": "6130001::V6.13.1::ARMCLANG", "uAC6": "1"})
    options = ET.SubElement(target, "TargetOption")
    common = ET.SubElement(options, "TargetCommonOption")
    fields(common, {"Device": "STM32F103C8", "Vendor": "STMicroelectronics",
                    "PackID": "Keil.STM32F1xx_DFP.2.4.1",
                    "Cpu": 'IRAM(0x20000000-0x20004FFF) IROM(0x8000000-0x800FFFF) CLOCK(72000000) CPUTYPE("Cortex-M3") TZ',
                    "OutputDirectory": "build\\", "OutputName": "m1_sampling", "CreateExecutable": "1",
                    "CreateLib": "0", "CreateHexFile": "1", "DebugInformation": "1",
                    "BrowseInformation": "0", "ListingPath": "build\\", "HexFormatSelection": "1"})
    common_property = ET.SubElement(options, "CommonProperty")
    fields(common_property, {"UseCPPCompiler": 0, "RVCTCodeConst": 0, "RVCTZI": 0,
        "RVCTOtherData": 0, "ModuleSelection": 0, "IncludeInBuild": 1, "AlwaysBuild": 0,
        "GenerateAssemblyFile": 0, "AssembleAssemblyFile": 0, "PublicsOnly": 0, "StopOnExitCode": 3,
        "CustomArgument": "", "IncludeLibraryModules": "", "ComprImg": 0})
    fields(ET.SubElement(options, "DllOption"), {
        "SimDllName": "SARMCM3.DLL", "SimDllArguments": "-REMAP", "SimDlgDll": "DCM.DLL",
        "SimDlgDllArguments": "-pCM3", "TargetDllName": "SARMCM3.DLL", "TargetDllArguments": "",
        "TargetDlgDll": "TCM.DLL", "TargetDlgDllArguments": "-pCM3"})
    arm = ET.SubElement(options, "TargetArmAds")
    misc = ET.SubElement(arm, "ArmAdsMisc")
    fields(misc, {"AdsLmap": "1", "AdsLsiz": "1", "AdsLsym": "1", "AdsLinfo": "1", "AdsLcgr": "1",
                  "AdsLsun": "1", "AdsLven": "1", "AdsCpuType": '"Cortex-M3"',
                  "RvctClst": "0", "RvctROPI": "0", "RvctRWPI": "0", "useUlib": "1"})
    c_options = ET.SubElement(arm, "Cads")
    fields(c_options, {"interw": "1", "Optim": "3", "oTime": "0", "OneElfS": "1", "wLevel": "3",
                       "uC99": "1", "v6Lang": "3", "v6LangP": "3", "vShortEn": "0",
                       "vShortWch": "0", "v6Lto": "0", "v6WtE": "0"})
    fields(ET.SubElement(c_options, "VariousControls"), {
        "MiscControls": "--target=arm-arm-none-eabi -mcpu=cortex-m3 -std=c11 -O2 -Wall -Wextra", "Define": "USE_HAL_DRIVER,STM32F103xB",
        "Undefine": "",
        "IncludePath": ";".join("../" + path for path in project["includes"])})
    assembly = ET.SubElement(arm, "Aads")
    fields(assembly, {"interw": "1", "thumb": "0", "ClangAsOpt": "1"})
    fields(ET.SubElement(assembly, "VariousControls"), {
        "MiscControls": "--cpu=Cortex-M3", "Define": "", "Undefine": "", "IncludePath": ""})
    linker = ET.SubElement(arm, "LDads")
    fields(linker, {"umfTarg": "0", "useFile": "1", "noStLib": "0", "RepFail": "1",
                    "ScatterFile": "m1_sampling.sct", "Misc": "--strict --entry=Reset_Handler",
                    "IncludeLibs": "", "IncludeLibsPath": ""})
    # uVision 的老版本读取器对省略字段/空元素很敏感。显式写全结构，不依赖其缺省指针。
    def complete_fields(parent, names, string_fields=()):
        """按工程格式固定顺序补齐字段，字符串留空、数值为0；保留已显式指定的值。"""
        existing = {child.tag: child for child in parent}
        for child in list(parent):
            parent.remove(child)
        for name in names.split():
            child = existing.pop(name, None)
            if child is None:
                child = ET.Element(name)
                child.text = "" if name in string_fields else "0"
            parent.append(child)
        for child in existing.values():
            parent.append(child)

    complete_fields(common,
        "Device Vendor PackID PackURL Cpu FlashUtilSpec StartupFile FlashDriverDll DeviceId RegisterFile "
        "MemoryEnv Cmp Asm Linker OHString InfinionOptionDll SLE66CMisc SLE66AMisc SLE66LinkerMisc "
        "SFDFile UseEnv BinPath IncludePath LibPath RegisterFilePath DBRegisterFilePath TargetStatus "
        "OutputDirectory OutputName CreateExecutable CreateLib CreateHexFile DebugInformation BrowseInformation "
        "ListingPath HexFormatSelection Merge32K CreateBatchFile BeforeCompile BeforeMake AfterMake "
        "SelectedForBatchBuild SVCSIdString",
        "PackURL FlashUtilSpec StartupFile FlashDriverDll RegisterFile MemoryEnv Cmp Asm Linker OHString "
        "InfinionOptionDll SLE66CMisc SLE66AMisc SLE66LinkerMisc SFDFile BinPath IncludePath LibPath "
        "RegisterFilePath DBRegisterFilePath SVCSIdString".split())
    status = common.find("TargetStatus")
    status.text = None
    fields(status, {"Error": 0, "ExitCodeStop": 0, "ButtonStop": 0, "NotGenerated": 0, "InvalidFlash": 1})
    for action in ["BeforeCompile", "BeforeMake", "AfterMake"]:
        node = common.find(action)
        node.text = None
        fields(node, {"RunUserProg1": 0, "RunUserProg2": 0, "UserProg1Name": "", "UserProg2Name": "",
                      "UserProg1Dos16Mode": 0, "UserProg2Dos16Mode": 0})
    complete_fields(misc,
        "GenerateListings asHll asAsm asMacX asSyms asFals asDbgD asForm ldLst ldmm ldXref BigEnd "
        "AdsALst AdsACrf AdsANop AdsANot AdsLLst AdsLmap AdsLcgr AdsLsym AdsLszi AdsLtoi AdsLsun "
        "AdsLven AdsLsxf RvctClst GenPPlst AdsCpuType RvctDeviceName mOS uocRom uocRam hadIROM hadIRAM "
        "hadXRAM uocXRam RvdsVP hadIRAM2 hadIROM2 StupSel useUlib EndSel uLtcg RoSelD RwSelD CodeSel "
        "OptFeed NoZi1 NoZi2 NoZi3 NoZi4 NoZi5 Ro1Chk Ro2Chk Ro3Chk Ir1Chk Ir2Chk Ra1Chk Ra2Chk "
        "Ra3Chk Im1Chk Im2Chk RvctStartVector", ["RvctDeviceName", "RvctStartVector"])
    memories = ET.Element("OnChipMemories")
    for name in [*("Ocm" + str(number) for number in range(1, 7)), "IRAM", "IROM", "XRAM",
                 *("OCR_RVCT" + str(number) for number in range(1, 11))]:
        memory = ET.SubElement(memories, name)
        if name in {"IRAM", "OCR_RVCT9"}:
            fields(memory, {"Type": 0, "StartAddress": "0x20000000", "Size": "0x5000"})
        elif name in {"IROM", "OCR_RVCT4"}:
            fields(memory, {"Type": 1, "StartAddress": "0x08000000", "Size": "0x10000"})
        else:
            fields(memory, {"Type": 0, "StartAddress": "0x0", "Size": "0x0"})
    misc.insert(list(misc).index(misc.find("RvctStartVector")), memories)
    for name, value in {"hadIROM": 1, "hadIRAM": 1, "StupSel": 8, "RoSelD": 3,
                        "RwSelD": 4, "Ir1Chk": 1, "Im1Chk": 1}.items():
        misc.find(name).text = str(value)
    complete_fields(c_options,
        "interw Optim oTime SplitLS OneElfS Strict EnumInt PlainCh Ropi Rwpi wLevel uThumb uSurpInc "
        "uC99 uGnu useXO v6Lang v6LangP vShortEn vShortWch v6Lto v6WtE v6Rtti VariousControls")
    complete_fields(assembly, "interw Ropi Rwpi thumb SplitLS SwStkChk NoWarn uSurpInc useXO ClangAsOpt VariousControls")
    complete_fields(linker,
        "umfTarg Ropi Rwpi noStLib RepFail useFile TextAddressRange DataAddressRange ScatterFile "
        "IncludeLibs IncludeLibsPath Misc LinkerInputFile DisabledWarnings",
        "TextAddressRange DataAddressRange LinkerInputFile DisabledWarnings".split())
    groups = ET.SubElement(target, "Groups")
    for group_name, paths in [
        ("Startup", [project["startup"]]),
        ("Application", [p for p in project["sources"] if not p.startswith("Drivers/")]),
        ("ST HAL", [p for p in project["sources"] if p.startswith("Drivers/")]),
    ]:
        group = ET.SubElement(groups, "Group")
        fields(group, {"GroupName": group_name})
        files = ET.SubElement(group, "Files")
        for path in paths:
            file = ET.SubElement(files, "File")
            fields(file, {"FileName": Path(path).name, "FileType": "2" if path.endswith(".s") else "1",
                          "FilePath": "../" + path})
    ET.indent(document, space="  ")
    content = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(
        document, encoding="unicode", short_empty_elements=False) + "\n"
    destination = firmware / "MDK-ARM/m1_sampling.uvprojx"
    if arguments.check:
        if destination.read_text(encoding="utf-8") != content:
            raise SystemExit("MDK 工程与清单不一致，请重新生成。")
        print("PASS MDK project/source manifest consistency")
    else:
        destination.write_bytes(content.encode("utf-8"))
        print("Generated MDK-ARM/m1_sampling.uvprojx")


if __name__ == "__main__":
    main()
