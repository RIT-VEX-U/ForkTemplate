from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import time
from pathlib import Path

from .toolchain import Toolchain, discover_toolchain
from .util import BUILD_DIR, ROOT, SETTINGS_FILE, blue, bold, green, print_step, yellow

SOURCE_GLOBS = ("src/**/*.c", "src/**/*.cpp", "core/src/**/*.c", "core/src/**/*.cpp")
PROJECT_INCLUDES = (
    ROOT / "include",
    ROOT / "core" / "include",
    ROOT / "vendor" / "eigen",
    ROOT / "vendor" / "gcem" / "include",
)

COMMON_FLAGS = [
    "-DVexV5",
    "-target",
    "armv7a-none-eabi",
    "-fshort-enums",
    "-mfpu=vfpv3",
    "-mfloat-abi=softfp",
    "-Os",
    "-g3",
    "-fcolor-diagnostics",
]
C_FLAGS = ["-std=gnu99"]
CXX_FLAGS = [
    "-fno-rtti",
    "-fno-threadsafe-statics",
    "-fno-exceptions",
    "-std=gnu++23",
    "-ffunction-sections",
    "-fdata-sections",
]
WARNING_FLAGS = ["-Wall", "-Werror=return-type"]
LINK_LIBS = [
    "-lv5rt",
    "-lc++",
    "-lc++abi",
    "-lunwind",
    "-lm",
    "-lc",
    "-lnosys",
    "-lclang_rt.builtins",
]
OBJDUMP_FLAGS = ["--source", "--line-numbers", "--demangle", "--disassemble"]


def write_if_changed(path: Path, content: str) -> None:
    if not path.exists() or path.read_text(encoding="utf-8") != content:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


def get_project_name(args: argparse.Namespace) -> str:
    name = args.project_name
    if not name and SETTINGS_FILE.exists():
        name = json.loads(SETTINGS_FILE.read_text(encoding="utf-8")).get("project", {}).get("name")
    name = name or "VexProject"

    if any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for char in name):
        raise SystemExit(f"Project name must contain only letters, numbers, underscores, dots, and hyphens: {name}")
    return name


def generate_build_files(name: str, sources: list[Path], toolchain: Toolchain, quiet: bool) -> None:
    includes = [f"-I{path}" for path in PROJECT_INCLUDES]
    system_includes = [
        "-isystem",
        str(toolchain.cxx_include_dir),
        "-isystem",
        str(toolchain.resource_dir / "include"),
        "-isystem",
        str(toolchain.newlib_include_dir),
        "-isystem",
        str(toolchain.sdk_path / "include"),
    ]
    base_flags = [f"-resource-dir={toolchain.resource_dir}", *COMMON_FLAGS, *(["-w"] if quiet else WARNING_FLAGS)]

    commands = []
    for source in sources:
        obj = BUILD_DIR / "objects" / f"{source.relative_to(ROOT)}.obj"
        lang_flags = CXX_FLAGS if source.suffix == ".cpp" else C_FLAGS
        cmd_args = [*base_flags, *lang_flags, *includes, *system_includes, "-MMD", "-MP", "-MF", str(obj.with_suffix(".obj.d")), "-o", str(obj), "-c", str(source)]
        driver = "clang++" if source.suffix == ".cpp" else "clang"
        commands.append({"directory": str(ROOT), "arguments": [driver, *cmd_args], "file": str(source), "output": str(obj)})

    write_if_changed(BUILD_DIR / "compile_commands.json", json.dumps(commands, indent=2) + "\n")

    obj_list = " \\\n  ".join(f"$(OBJROOT)/{source.relative_to(ROOT).as_posix()}.obj" for source in sources)
    ld_flags = ["-z", "norelro", "-T", toolchain.linker_script.as_posix(), "--gc-sections", f"-L{toolchain.sdk_path.as_posix()}", f"-L{toolchain.newlib_lib_dir.as_posix()}"]
    makefile_path = BUILD_DIR / "Makefile"

    makefile = f"""\
.PHONY: all

PROJECT := {name}
BUILD_MAKEFILE := {makefile_path.relative_to(ROOT).as_posix()}
OBJROOT := build/objects

CLANG := "{toolchain.clang.as_posix()}"
LD := "{toolchain.linker.as_posix()}"
OBJCOPY := "{toolchain.objcopy.as_posix()}"
OBJDUMP := "{toolchain.objdump.as_posix()}"
SIZE := "{toolchain.size.as_posix()}"

ELF := build/$(PROJECT).elf
BIN := build/$(PROJECT).bin
ASM := build/$(PROJECT).S

COMMON_FLAGS := {shlex.join(base_flags)}
INCLUDES := {shlex.join(includes)}
SYSTEM_INCLUDES := {shlex.join(system_includes)}
CFLAGS := {shlex.join(C_FLAGS)}
CXXFLAGS := {shlex.join(CXX_FLAGS)}
LDFLAGS := {shlex.join(ld_flags)}
LDLIBS := {shlex.join(LINK_LIBS)}

OBJECTS := \\
  {obj_list}
DEPFILES := $(OBJECTS:.obj=.obj.d)

all: $(BIN) $(ASM)

$(OBJROOT)/%.cpp.obj: %.cpp $(BUILD_MAKEFILE)
\t@echo CXX      $<
\t@$(CLANG) $(COMMON_FLAGS) $(CXXFLAGS) $(INCLUDES) $(SYSTEM_INCLUDES) -MMD -MP -MF "$@.d" -o "$@" -c "$<"

$(OBJROOT)/%.c.obj: %.c $(BUILD_MAKEFILE)
\t@echo CC       $<
\t@$(CLANG) $(COMMON_FLAGS) $(CFLAGS) $(INCLUDES) $(SYSTEM_INCLUDES) -MMD -MP -MF "$@.d" -o "$@" -c "$<"

$(ELF): $(OBJECTS) $(BUILD_MAKEFILE)
\t@echo {'LINK':<8} $(PROJECT).elf
\t@$(LD) $(LDFLAGS) $(OBJECTS) -o "$@" --start-group $(LDLIBS) --end-group
\t@$(SIZE) "$@"

$(BIN): $(ELF)
\t@$(OBJCOPY) -O binary "$<" "$@"

$(ASM): $(ELF)
\t@$(OBJDUMP) {shlex.join(OBJDUMP_FLAGS)} "$<" > "$@"

-include $(DEPFILES)
"""
    write_if_changed(makefile_path, makefile)


def clean() -> int:
    if BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)
        print_step(green("Cleaned build directory"))
    else:
        print_step(yellow("Build directory does not exist"))
    return 0


def build(args: argparse.Namespace) -> int:
    start_time = time.time()
    name = get_project_name(args)
    toolchain = discover_toolchain()
    sources = sorted(source for glob in SOURCE_GLOBS for source in ROOT.glob(glob))
    jobs = args.parallel or os.cpu_count() or 1

    print_step(bold("VEX V5 Build"))
    print_step(f"{blue('Project:')} {name}")
    print_step(f"{blue('SDK:')} {toolchain.sdk_path}")
    print_step(f"{blue('Toolchain:')} {toolchain.toolchain_path}")
    print_step(f"{blue('Parallel jobs:')} {jobs}")

    for source in sources:
        (BUILD_DIR / "objects" / f"{source.relative_to(ROOT)}.obj").parent.mkdir(parents=True, exist_ok=True)

    generate_build_files(name, sources, toolchain, args.quiet)

    result = subprocess.run(
        [str(toolchain.make), "--silent", "--no-print-directory", "-f", str(BUILD_DIR / "Makefile"), f"-j{jobs}"],
        cwd=ROOT,
        check=False,
    )
    if result.returncode != 0:
        return result.returncode

    for suffix in ("elf", "bin", "S"):
        artifact = BUILD_DIR / f"{name}.{suffix}"
        if artifact.exists():
            print_step(green(f"Output: {artifact.name} ({artifact.stat().st_size:,} bytes)"))

    print_step(green(f"Build successful ({time.time() - start_time:.1f}s)"))
    return 0


def rebuild(args: argparse.Namespace) -> int:
    clean()
    return build(args)
