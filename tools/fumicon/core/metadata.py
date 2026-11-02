# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Load build identity and compile-time string dictionary from gen_fumicon_json output."""

from __future__ import annotations

import json
import re
from os import PathLike


class FirmwareMetadata:
    """Manages static string hash dictionary and build_id verification."""

    def __init__(self, path: str | PathLike[str] | None = None) -> None:
        self.strings: dict[int, str] = {}
        self.build_id: bytes | None = None

        if path is not None:
            self.load_file(path)

    def load_file(self, path: str | PathLike[str]) -> None:
        """Load and validate metadata from a JSON file."""
        with open(path, encoding="utf-8") as stream:
            data = json.load(stream)

        if not isinstance(data, dict):
            raise ValueError("firmware metadata must be a JSON object")  # noqa: TRY004

        build_id = data.get("build_id")
        if (
            not isinstance(build_id, str)
            or re.fullmatch(r"[0-9a-fA-F]{40}", build_id) is None
        ):
            raise ValueError("metadata build_id must be a 40-digit SHA-1 hex string")
        self.build_id = bytes.fromhex(build_id)

        entries = data.get("strhash")
        if not isinstance(entries, dict):
            raise ValueError("metadata strhash must be an object")  # noqa: TRY004

        for key, value in entries.items():
            if re.fullmatch(r"[0-9]+", key) is None or not (0 <= int(key) <= 0xFFFFFFFF):
                raise ValueError(f"invalid u32 string hash: {key!r}")
            if not isinstance(value, str):
                raise ValueError(f"string hash {key} must map to text string")  # noqa: TRY004
            number = int(key)
            if number in self.strings and self.strings[number] != value:
                raise ValueError(f"conflicting entries for string hash {number}")
            self.strings[number] = value

    @property
    def build_id_hex(self) -> str | None:
        """Return 40-character hex string of build_id."""
        return self.build_id.hex() if self.build_id is not None else None

    def lookup(self, string_hash: int) -> str | None:
        """Resolve a 32-bit FNV-1a string hash to its original literal string."""
        return self.strings.get(string_hash)

    def verify_build_id(self, device_build_id: bytes) -> bool:
        """Verify if the device's 20-byte SHA-1 matches this metadata file."""
        if self.build_id is None:
            return True
        return self.build_id == device_build_id
