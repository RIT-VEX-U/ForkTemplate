from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = ROOT / "build"
SETTINGS_FILE = ROOT / ".vscode" / "vex_project_settings.json"


def color(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if sys.stdout.isatty() else text


def blue(text: str) -> str:
    return color("0;34", text)


def green(text: str) -> str:
    return color("0;32", text)


def yellow(text: str) -> str:
    return color("1;33", text)


def bold(text: str) -> str:
    return color("1", text)


def print_step(message: str) -> None:
    print(message, flush=True)
