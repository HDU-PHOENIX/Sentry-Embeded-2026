import os
import xml.etree.ElementTree as ET
import json

def get_keil_info():
    root_dir = os.path.dirname(os.path.abspath(__file__))
    keil_proj_relative_path = os.path.join('MDK-ARM', 'frame.uvprojx')
    proj_file_path = os.path.join(root_dir, keil_proj_relative_path)
    
    if not os.path.exists(proj_file_path):
        print(f"Error: Could not find {proj_file_path}")
        return None

    tree = ET.parse(proj_file_path)
    xml_root = tree.getroot()

    defines = ""
    include_paths = ""
    c_files = []

    # Search for Cads element which contains C compiler settings
    for cads in xml_root.iter('Cads'):
        for define_elem in cads.iter('Define'):
            if define_elem.text:
                defines = define_elem.text
                break
        for inc_elem in cads.iter('IncludePath'):
            if inc_elem.text:
                include_paths = inc_elem.text
                break
        if defines or include_paths:
            break

    # Collect C source files from Keil project
    for file_node in xml_root.iter('File'):
        file_type_node = file_node.find('FileType')
        file_path_node = file_node.find('FilePath')
        if file_type_node is None or file_path_node is None or not file_path_node.text:
            continue
        file_type_text = (file_type_node.text or "").strip()
        file_path_text = file_path_node.text.strip()
        if file_type_text == '1' or file_path_text.lower().endswith('.c'):
            c_files.append(file_path_text)

    c_files = sorted(set(c_files))

    return {
        "defines": defines,
        "include_paths": include_paths,
        "root_dir": root_dir,
        "c_files": c_files
    }

def normalize_rel_path(path_text):
    normalized = path_text.replace('\\', '/').replace('//', '/')
    while '/./' in normalized:
        normalized = normalized.replace('/./', '/')
    return normalized


def generate_compile_commands(info):
    if not info:
        return

    root_dir = info['root_dir']
    mdk_dir = os.path.join(root_dir, 'MDK-ARM')

    # Process defines
    define_list = [d.strip() for d in info['defines'].split(',') if d.strip()]

    # Process include paths (relative to project root)
    include_flags = []
    for inc in [p.strip() for p in info['include_paths'].split(';') if p.strip()]:
        abs_inc = os.path.normpath(os.path.join(mdk_dir, inc))
        rel_inc = os.path.relpath(abs_inc, root_dir).replace('\\', '/')
        include_flags.append(f"-I{rel_inc}")

    common_flags = [
        "-xc",
        "-std=c11",
        "--target=arm-none-eabi",
        "-mfloat-abi=hard",
        "-mcpu=cortex-m4",
        "-mfpu=fpv4-sp-d16",
        "-ffreestanding",
        "-fno-builtin",
        "-D__CC_ARM",
        "-D__arm__",
        "-D__packed=__attribute__((packed))",
        "-D__align(x)=__attribute__((aligned(x)))",
        "-D__inline=inline",
        "-D__asm=__asm__",
        "-D__weak=__attribute__((weak))"
    ]

    for d in define_list:
        common_flags.append(f"-D{d}")

    common_flags.extend(include_flags)

    compile_db = []
    for file_path in info['c_files']:
        abs_src = os.path.normpath(os.path.join(mdk_dir, file_path))
        if not os.path.isfile(abs_src):
            continue
        rel_src = os.path.relpath(abs_src, root_dir).replace('\\', '/')
        command_parts = ["arm-none-eabi-gcc"] + common_flags + ["-c", rel_src]
        compile_db.append({
            "directory": normalize_rel_path(root_dir),
            "file": normalize_rel_path(rel_src),
            "command": " ".join(command_parts)
        })

    compile_commands_path = os.path.join(root_dir, 'compile_commands.json')
    with open(compile_commands_path, 'w', encoding='utf-8') as f:
        json.dump(compile_db, f, indent=2)

    print(f"Successfully generated {compile_commands_path} ({len(compile_db)} entries)")


def generate_clangd_config(info):
    root_dir = info['root_dir']
    clangd_path = os.path.join(root_dir, '.clangd')
    with open(clangd_path, 'w', encoding='utf-8') as f:
        f.write("CompileFlags:\n")
        f.write("  CompilationDatabase: .\n")
        f.write("  Add:\n")
        f.write("    - -D__CC_ARM\n")
        f.write("    - -D__arm__\n")
        f.write("    - -D__packed=__attribute__((packed))\n")
        f.write("    - -D__align(x)=__attribute__((aligned(x)))\n")
        f.write("    - -D__inline=inline\n")
        f.write("    - -D__asm=__asm__\n")
        f.write("    - -D__weak=__attribute__((weak))\n")
        f.write("  Compiler: arm-none-eabi-gcc\n")
        f.write("Diagnostics:\n")
        f.write("  UnusedIncludes: None\n")

    print(f"Successfully generated {clangd_path}")


if __name__ == "__main__":
    info = get_keil_info()
    if info:
        generate_compile_commands(info)
        generate_clangd_config(info)
