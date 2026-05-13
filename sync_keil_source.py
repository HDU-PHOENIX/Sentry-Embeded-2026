#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_keil_source.py  ——  Keil uVision 工程文件源文件依赖自动同步脚本

功能：
  以 Tasks/ 目录下的源文件为入口，递归解析 #include "xxx.h" 依赖链，
  找到所有被引用的 .c / .cpp 源文件，自动添加到 MDK-ARM/*.uvprojx 中。
  只添加真正被依赖的文件，而不是全量扫描目录。

用法：
    python sync_keil_source.py           # 正常执行，写入 .uvprojx
    python sync_keil_source.py --dry-run # 只打印将要添加的内容，不修改文件

配置：修改下方"配置区域"中的变量即可适配其他工程。
"""

import os
import re
import sys
import io
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

# ================================================================
#  配置区域 —— 根据你的工程情况修改这里
# ================================================================

# 相对于本脚本的 .uvprojx 文件路径
UVPROJX_PATH = "MDK-ARM/frame.uvprojx"

# 依赖分析的入口目录（从这些目录下的所有 .c/.h 文件出发，递归追踪 #include）
ENTRY_DIRS = [
    "Task",
]

# 依赖解析时的额外搜索目录（优先级低于工程文件中的 IncludePath）
EXTRA_SEARCH_DIRS = [
    "Core/Inc",
    "Core/Src",
]

# 找到依赖的源文件后，按哪个"根目录"来决定 Keil 分组名
# 格式：(根目录相对路径, 显示用的组前缀)
# 路径下的文件按子目录分组，例如：Bsp/Can/bsp_can.c → "Sentry_Libs/Bsp/Can"
SOURCE_ROOTS = [
    ("Sentry_Libs/phoenix_embedded_base_code", "Sentry_Libs"),
    ("Tasks",                                  "Task"),
]

# 是否将头文件 (.h / .hpp) 也写入工程（False = 仅源文件）
INCLUDE_HEADERS = False

# 解析 #include 时忽略这些目录（不递归进去）
EXCLUDE_DIRS = {
    ".git", ".svn", "__pycache__", "pic", "assets",
    "temp_informality_InsTask",
}

# 扩展名 → Keil FileType 代码
FILE_TYPE_MAP = {
    ".c":   1,
    ".cpp": 8,
    ".s":   2,
    ".asm": 2,
    ".h":   5,
    ".hpp": 5,
}

# ================================================================
#  以下无需修改
# ================================================================

INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)


def _parse_includes(filepath: Path) -> list[str]:
    """提取文件中所有 #include "xxx" 的 xxx 部分（忽略 <系统头文件>）。"""
    try:
        content = filepath.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return []
    return INCLUDE_RE.findall(content)


def _resolve_header(header_name: str, current_dir: Path,
                    search_dirs: list[Path]) -> Path | None:
    """
    按 C 编译器规则查找头文件：
      1. 当前文件所在目录
      2. search_dirs 中的每个目录
    """
    candidate = current_dir / header_name
    if candidate.is_file():
        return candidate.resolve()
    for d in search_dirs:
        candidate = d / header_name
        if candidate.is_file():
            return candidate.resolve()
    return None


def _find_source_for_header(header_path: Path) -> list[Path]:
    """
    在头文件所在目录查找同名的 .c / .cpp 源文件。
    例如 bsp_can.h → bsp_can.c
    """
    stem = header_path.stem
    parent = header_path.parent
    result = []
    for ext in (".c", ".cpp"):
        src = parent / (stem + ext)
        if src.is_file():
            result.append(src.resolve())
    return result


def _build_dependency_graph(entry_dirs: list[Path],
                             search_dirs: list[Path]) -> set[Path]:
    """
    BFS：从 entry_dirs 中所有 .c/.h 文件出发，
    递归追踪 #include "..." 依赖，返回所有涉及的源文件绝对路径集合。
    """
    visited_headers: set[Path] = set()
    found_sources: set[Path] = set()

    queue: list[Path] = []
    for entry_dir in entry_dirs:
        for root_str, dirs, files in os.walk(entry_dir):
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            for f in files:
                p = Path(root_str, f)
                if p.suffix.lower() in (".c", ".cpp", ".h", ".hpp"):
                    queue.append(p.resolve())
                    if p.suffix.lower() in (".c", ".cpp"):
                        found_sources.add(p.resolve())

    while queue:
        current = queue.pop()
        current_dir = current.parent

        for inc in _parse_includes(current):
            header = _resolve_header(inc, current_dir, search_dirs)
            if header is None or header in visited_headers:
                continue
            visited_headers.add(header)
            queue.append(header)

            # 找对应的源文件
            for src in _find_source_for_header(header):
                if src not in found_sources:
                    found_sources.add(src)
                    queue.append(src)  # 继续递归分析源文件的依赖

    return found_sources


def _group_name_for_file(abs_src: Path, source_roots: list[tuple],
                         project_root: Path) -> str:
    """
    根据 SOURCE_ROOTS 配置，确定该文件对应的 Keil 分组名。
    """
    for root_rel, prefix in source_roots:
        root_abs = (project_root / root_rel).resolve()
        try:
            rel = abs_src.relative_to(root_abs)
        except ValueError:
            continue
        parts = rel.parts[:-1]  # 去掉文件名，只保留目录部分
        if parts:
            return prefix + "/" + "/".join(parts)
        else:
            return prefix
    # 兜底：用文件所在目录名
    return abs_src.parent.name


def _indent_xml(elem, level=0):
    """兼容 Python < 3.9 的 XML 格式化缩进（2 空格）。"""
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            _indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad
    if level == 0:
        elem.tail = "\n"


def _make_file_elem(filename: str, filepath: str, file_type: int) -> ET.Element:
    file_elem = ET.Element("File")
    ET.SubElement(file_elem, "FileName").text = filename
    ET.SubElement(file_elem, "FileType").text = str(file_type)
    ET.SubElement(file_elem, "FilePath").text = filepath
    return file_elem


def _make_group_elem(group_name: str):
    group_elem = ET.Element("Group")
    ET.SubElement(group_elem, "GroupName").text = group_name
    files_elem = ET.SubElement(group_elem, "Files")
    return group_elem, files_elem


def main():
    dry_run = "--dry-run" in sys.argv

    script_dir = Path(__file__).parent.resolve()
    uvprojx_path = (script_dir / UVPROJX_PATH).resolve()
    uvprojx_dir = uvprojx_path.parent   # MDK-ARM/
    project_root = script_dir           # 工程根目录

    if not uvprojx_path.exists():
        print(f"[错误] 找不到工程文件: {uvprojx_path}")
        sys.exit(1)

    # ---------- 解析 XML ----------
    ET.register_namespace("xsi", "http://www.w3.org/2001/XMLSchema-instance")
    tree = ET.parse(uvprojx_path)
    root_xml = tree.getroot()

    groups_elem = root_xml.find(".//Groups")
    if groups_elem is None:
        print("[错误] 工程文件中找不到 <Groups> 元素")
        sys.exit(1)

    # ---------- 从 uvprojx 读取 IncludePath 作为头文件搜索目录 ----------
    inc_path_elem = root_xml.find(".//IncludePath")
    search_dirs: list[Path] = []
    if inc_path_elem is not None and inc_path_elem.text:
        for p in inc_path_elem.text.split(";"):
            p = p.strip()
            if p:
                resolved = (uvprojx_dir / p).resolve()
                if resolved.is_dir():
                    search_dirs.append(resolved)

    # 加入额外搜索目录
    for d in EXTRA_SEARCH_DIRS:
        resolved = (project_root / d).resolve()
        if resolved.is_dir() and resolved not in search_dirs:
            search_dirs.append(resolved)

    print(f"[信息] 已加载 {len(search_dirs)} 个头文件搜索路径")

    # ---------- 收集现有工程文件路径 ----------
    existing_paths: set[str] = set()
    existing_groups: dict[str, tuple] = {}

    for group_elem in groups_elem.findall("Group"):
        gn_elem = group_elem.find("GroupName")
        if gn_elem is None or not gn_elem.text:
            continue
        group_name = gn_elem.text
        files_elem = group_elem.find("Files")
        if files_elem is None:
            files_elem = ET.SubElement(group_elem, "Files")
        existing_groups[group_name] = (group_elem, files_elem)

        for file_elem in files_elem.findall("File"):
            fp_elem = file_elem.find("FilePath")
            if fp_elem is not None and fp_elem.text:
                abs_path = (uvprojx_dir / fp_elem.text.replace("\\", "/")).resolve()
                existing_paths.add(str(abs_path).lower())

    print(f"[信息] 工程中已有 {len(existing_paths)} 个文件\n")

    # ---------- 依赖分析 ----------
    entry_dirs_abs = []
    for d in ENTRY_DIRS:
        p = (project_root / d).resolve()
        if p.is_dir():
            entry_dirs_abs.append(p)
        else:
            print(f"[警告] 入口目录不存在，已跳过: {d}")

    print(f"[分析] 从 {[str(d) for d in entry_dirs_abs]} 开始递归解析依赖...")
    all_sources = _build_dependency_graph(entry_dirs_abs, search_dirs)
    print(f"[分析] 共追踪到 {len(all_sources)} 个源文件\n")

    # ---------- 过滤出需要新增的文件 ----------
    # {group_name: [(filename, rel_path, file_type), ...]}
    pending: dict[str, list] = {}

    source_roots_abs = [
        ((project_root / r).resolve(), prefix)
        for r, prefix in SOURCE_ROOTS
    ]

    for abs_src in sorted(all_sources):
        ext = abs_src.suffix.lower()
        if ext not in FILE_TYPE_MAP:
            continue
        if not INCLUDE_HEADERS and ext in (".h", ".hpp"):
            continue

        norm_key = str(abs_src).lower()
        if norm_key in existing_paths:
            continue

        group_name = _group_name_for_file(abs_src, SOURCE_ROOTS, project_root)
        rel_path = os.path.relpath(abs_src, uvprojx_dir).replace("\\", "/")
        file_type = FILE_TYPE_MAP[ext]
        pending.setdefault(group_name, []).append(
            (abs_src.name, rel_path, file_type)
        )

    total_added = sum(len(v) for v in pending.values())

    if total_added == 0:
        print("[完成] 没有需要添加的新文件，工程文件未改动。")
        return

    # ---------- 打印预览 ----------
    for group_name, files_list in sorted(pending.items()):
        tag = "[已有组]" if group_name in existing_groups else "[新建组]"
        print(f"  {tag} {group_name}  (+{len(files_list)} 个文件)")
        for fname, fpath, _ in sorted(files_list):
            print(f"         + {fname}  →  {fpath}")
    print()

    if dry_run:
        print(f"[dry-run] 共将添加 {total_added} 个文件（未实际修改）。")
        return

    # ---------- 备份 ----------
    backup_path = uvprojx_path.with_suffix(".uvprojx.bak")
    shutil.copy2(uvprojx_path, backup_path)
    print(f"[备份] {backup_path.name}")

    # ---------- 将文件写入 XML ----------
    for group_name, files_list in sorted(pending.items()):
        if group_name in existing_groups:
            _, files_elem = existing_groups[group_name]
        else:
            new_group_elem, files_elem = _make_group_elem(group_name)
            insert_idx = len(groups_elem)
            for i, g in enumerate(groups_elem):
                gn = g.find("GroupName")
                if gn is not None and gn.text and gn.text.startswith("::"):
                    insert_idx = i
                    break
            groups_elem.insert(insert_idx, new_group_elem)
            existing_groups[group_name] = (new_group_elem, files_elem)

        for filename, rel_path, file_type in sorted(files_list):
            files_elem.append(_make_file_elem(filename, rel_path, file_type))

    # ---------- 格式化 & 写回 ----------
    try:
        ET.indent(root_xml, space="  ")     # Python 3.9+
    except AttributeError:
        _indent_xml(root_xml)               # Python < 3.9 兼容

    buf = io.BytesIO()
    tree.write(buf, encoding="UTF-8", xml_declaration=True)
    content = buf.getvalue().decode("utf-8")
    content = content.replace(
        "<?xml version='1.0' encoding='UTF-8'?>",
        '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'
    )
    content = content.replace(" ns0:", " xsi:")
    content = content.replace("xmlns:ns0=", "xmlns:xsi=")

    uvprojx_path.write_text(content, encoding="utf-8")
    print(f"[完成] 共添加 {total_added} 个文件 → {uvprojx_path.name}")


if __name__ == "__main__":
    main()
