#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Compare CubeMX's literal CMake lists with our adapter, without executing CMake.

This is intentionally not a CMake interpreter. Supported lists use whitespace or
semicolon separators, optional quotes and line comments. Variable expansion,
bracket arguments/comments and computed lists require manual review.
"""

import argparse
from collections import Counter
from pathlib import Path
import re
import shlex
import sys


LIST_PAIRS = {
    "MX_Application_Src": "cubemx_board_sources",
    "STM32_Drivers_Src": "cubemx_driver_sources",
    "MX_Include_Dirs": "cubemx_include_dirs",
    "MX_Defines_Syms": "cubemx_definitions",
}
SOURCE_GROUPS = {name for name in LIST_PAIRS if name.endswith("_Src")}
GENERATED_PATH_PREFIX = "${CMAKE_CURRENT_SOURCE_DIR}/../../"
# Root CMake supplies libm; the generated build's object targets are renamed.
EXPECTED_LINK_LIBS = ["STM32_Drivers", "${TOOLCHAIN_LINK_LIBRARIES}"]
# The only source substitution: platform/ supplies our CCMRAM-aware startup.
GENERATED_STARTUP = "startup_stm32g474xx.s"
ADAPTER_PATH = Path("cmake/cubemx/CMakeLists.txt")
GENERATED_PATH = Path("third_party/cubemx/cmake/stm32cubemx/CMakeLists.txt")


class ManifestError(ValueError):
    """A manifest no longer uses the supported literal-list format."""


def read_list(text: str, name: str, label: str) -> list[str]:
    """Read exactly one set(NAME ...) declaration from the current template."""
    # Reject bracket syntax instead of accidentally parsing commented-out sets.
    if re.search(r"\[=*\[", text):
        raise ManifestError(f"{label}: bracket arguments/comments require review")
    # Remove line comments before finding ')', preserving hashes inside quotes.
    text = re.sub(
        r'"(?:\\.|[^"\\])*"|#[^\n]*',
        lambda match: "" if match[0].startswith("#") else match[0],
        text,
    )
    if re.search(rf"\b(?:list|string|file)\s*\([^)]*\b{re.escape(name)}\b", text):
        raise ManifestError(f"{label}: {name}: computed lists require review")
    declarations = re.findall(
        rf"^[ \t]*set\s*\(\s*{re.escape(name)}(?=\s|\))([^)]*)\)",
        text,
        re.MULTILINE,
    )
    if len(declarations) != 1:
        raise ManifestError(f"{label}: expected one literal set({name} ...)")
    try:
        tokens = shlex.split(declarations[0], comments=True, posix=True)
    except ValueError as error:
        raise ManifestError(f"{label}: invalid {name}: {error}") from error

    values = []
    for token in tokens:
        for value in token.split(";"):
            if not value:
                continue
            value = value.removeprefix(GENERATED_PATH_PREFIX)
            if "${" in value and value != "${TOOLCHAIN_LINK_LIBRARIES}":
                raise ManifestError(f"{label}: {name}: unsupported variable: {value}")
            if value.startswith("$ENV{") or value.startswith("$CACHE{"):
                raise ManifestError(f"{label}: {name}: unsupported variable: {value}")
            if value.startswith("[") or "(" in value or "\\" in value:
                raise ManifestError(f"{label}: {name}: unsupported syntax: {value}")
            values.append(value)
    return values


def compare_list(name: str, generated: list[str], expected: list[str]) -> list[str]:
    """Report actionable differences, including duplicates hidden by set equality."""
    problems = []
    for label, values in (("generated", generated), ("adapter", expected)):
        for value, count in sorted(Counter(values).items()):
            if count > 1:
                problems.append(f"{name}: duplicate in {label}: {value} ({count} copies)")
    for value in sorted(set(generated) - set(expected)):
        problems.append(f"{name}: only in generated: {value}")
    for value in sorted(set(expected) - set(generated)):
        problems.append(f"{name}: missing from generated: {value}")
    return problems


def check_manifests(generated: str, adapter: str) -> list[str]:
    """Compare the seven known lists and reject unreviewed source groups."""
    problems = []
    declared_groups = set(re.findall(
        r"^[ \t]*set\s*\(\s*([A-Za-z0-9_]+_Src)\b", generated, re.MULTILINE
    ))
    for name in sorted(declared_groups - SOURCE_GROUPS):
        problems.append(f"unknown generated source group: {name}")

    for name, adapter_name in LIST_PAIRS.items():
        actual = read_list(generated, name, "generated")
        expected = read_list(adapter, adapter_name, "adapter")
        if name == "MX_Application_Src":
            expected.append(GENERATED_STARTUP)
        problems.extend(compare_list(name, actual, expected))

    problems.extend(compare_list(
        "MX_LINK_DIRS", read_list(generated, "MX_LINK_DIRS", "generated"), []
    ))
    problems.extend(compare_list(
        "MX_LINK_LIBS", read_list(generated, "MX_LINK_LIBS", "generated"), EXPECTED_LINK_LIBS
    ))
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1],
        help="project root (defaults to the checkout containing this script)",
    )
    args = parser.parse_args()
    try:
        generated = (args.root / GENERATED_PATH).read_text(encoding="utf-8")
        adapter = (args.root / ADAPTER_PATH).read_text(encoding="utf-8")
        problems = check_manifests(generated, adapter)
    except (OSError, UnicodeError, ManifestError) as error:
        problems = [str(error)]

    if problems:
        print("CubeMX manifest check failed:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        print(
            f"Review the CubeMX changes and update {ADAPTER_PATH.as_posix()}; "
            "template syntax changes may also require updating this checker.",
            file=sys.stderr,
        )
        return 1

    print("CubeMX manifests match (platform startup substitution accounted for).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
