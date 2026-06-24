import json
import os
import platform
import shutil
import urllib.request
from dataclasses import dataclass
from pathlib import Path

VEX_DIR = Path.home() / ".vex" / "vexcode"
OS_MAP = {"Windows": "Windows-x86_64", "Darwin": "Darwin-universal", "Linux": "Linux-x86_64"}
SDK_MANIFEST = "https://content.vexrobotics.com/vexos/public/V5/vscode/sdk/cpp/manifest.json"
LSCRIPT_URL = "https://github.com/RIT-VEX-U/ForkTemplate/releases/download/22.1/lscript_llvm.ld"
ATFE_VERSION = "22.1.0"
ATFE_RELEASE = "22.1"
CLANG_VERSION = ATFE_VERSION.split(".", maxsplit=1)[0]
HOST = platform.system()

if HOST not in OS_MAP:
    raise SystemExit(f"Unsupported host platform: {HOST}")

ATFE_NAME = f"ATfE-{ATFE_VERSION}-{OS_MAP[HOST]}"
ATFE_URL = f"https://github.com/RIT-VEX-U/ForkTemplate/releases/download/{ATFE_RELEASE}/{ATFE_NAME}.tar.xz"


@dataclass
class Toolchain:
    sdk_path: Path
    toolchain_path: Path
    clang: Path
    linker: Path
    objcopy: Path
    objdump: Path
    size: Path
    make: Path
    resource_dir: Path
    cxx_include_dir: Path
    newlib_include_dir: Path
    newlib_lib_dir: Path
    linker_script: Path


def request(url: str) -> urllib.request.Request:
    return urllib.request.Request(url, headers={"User-Agent": "build"})


def download(url: str, dest: Path) -> None:
    print(f"Downloading {url}...", flush=True)
    with urllib.request.urlopen(request(url)) as response, dest.open("wb") as output:
        shutil.copyfileobj(response, output)


def get_sdk() -> Path:
    if sdks := sorted(VEX_DIR.glob("V5_*/vexv5"), reverse=True):
        return sdks[0]

    VEX_DIR.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(request(SDK_MANIFEST)) as response:
        latest = json.load(response)["latest"]

    sdk_path = VEX_DIR / latest / "vexv5"
    zip_path = VEX_DIR / f"{latest}.zip"

    download(f"https://content.vexrobotics.com/vexos/public/V5/vscode/sdk/cpp/{latest}.zip", zip_path)
    shutil.unpack_archive(zip_path, VEX_DIR)
    zip_path.unlink()
    download(LSCRIPT_URL, sdk_path / "lscript_llvm.ld")

    return sdk_path


def discover_toolchain() -> Toolchain:
    sdk_path = get_sdk()
    toolchain_path = VEX_DIR / ATFE_NAME

    if not toolchain_path.exists():
        VEX_DIR.mkdir(parents=True, exist_ok=True)
        tar_path = VEX_DIR / f"{ATFE_NAME}.tar.xz"
        download(ATFE_URL, tar_path)
        shutil.unpack_archive(tar_path, VEX_DIR)
        tar_path.unlink()

    exe = ".exe" if os.name == "nt" else ""
    bin_dir = toolchain_path / "bin"

    return Toolchain(
        sdk_path=sdk_path,
        toolchain_path=toolchain_path,
        clang=bin_dir / f"clang{exe}",
        linker=bin_dir / f"ld.lld{exe}",
        objcopy=bin_dir / f"llvm-objcopy{exe}",
        objdump=bin_dir / f"llvm-objdump{exe}",
        size=bin_dir / f"llvm-size{exe}",
        make=toolchain_path / "tools" / f"make{exe}",
        resource_dir=toolchain_path / f"lib/clang/{CLANG_VERSION}",
        cxx_include_dir=toolchain_path / "lib/clang-runtimes/newlib/arm-none-eabi/include/c++/v1",
        newlib_include_dir=toolchain_path / "lib/clang-runtimes/newlib/arm-none-eabi/include",
        newlib_lib_dir=toolchain_path / "lib/clang-runtimes/newlib/arm-none-eabi/armv7a_soft_vfpv3_d16_unaligned/lib",
        linker_script=sdk_path / "lscript_llvm.ld",
    )
