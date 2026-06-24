from __future__ import annotations

import argparse

from .build import build, clean, rebuild


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_build_options(subparser: argparse.ArgumentParser) -> None:
        subparser.add_argument("-p", "--parallel", type=int, default=None, help="number of parallel compile jobs")
        subparser.add_argument("-q", "--quiet", action="store_true", help="suppress compiler warnings")
        subparser.add_argument("-n", "--project-name", help="change project name")

    add_build_options(subparsers.add_parser("build", help="build the project"))
    add_build_options(subparsers.add_parser("rebuild", help="clean and build the project"))
    subparsers.add_parser("clean", help="nuke build directory")

    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "clean":
        return clean()
    if args.command == "rebuild":
        return rebuild(args)
    return build(args)
