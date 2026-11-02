# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""C99-compliant printf formatting engine for Fumi Log parameters."""

from __future__ import annotations

import math
import re
from collections.abc import Sequence
from dataclasses import dataclass
from functools import lru_cache
from typing import Literal

from .models import DataTypeTag, LogParam

MAX_FIELD_SIZE = 4096
INTEGER_BITS = {"": 32, "hh": 8, "h": 16, "l": 32, "ll": 64, "j": 64, "z": 32, "t": 32}
PRINTF_SPEC = re.compile(
    r"%([-+ #0]*)(\*|[0-9]*)(?:\.(\*|[0-9]*))?(hh|ll|[hljztL])?([diouxXfFeEgGcsp%])"
)

INTEGER_TAGS = {
    DataTypeTag.I8: 8,
    DataTypeTag.I16: 16,
    DataTypeTag.I32: 32,
    DataTypeTag.I64: 64,
    DataTypeTag.U8: 8,
    DataTypeTag.U16: 16,
    DataTypeTag.U32: 32,
    DataTypeTag.U64: 64,
    DataTypeTag.BOOL: 1,
}


class FormattingError(ValueError):
    """Raised when format string cannot be matched to the provided parameters."""


@dataclass(frozen=True, slots=True)
class Conversion:
    flags: str
    width: int | Literal["*"] | None
    precision: int | Literal["*"] | None
    length: str
    kind: str


def field_size(text: str | None) -> int | Literal["*"] | None:
    if text is None:
        return None
    if text == "*":
        return "*"
    digits = text.lstrip("0") or "0"
    if len(digits) > 4 or int(digits) > MAX_FIELD_SIZE:
        raise FormattingError(f"width/precision exceeds {MAX_FIELD_SIZE}")
    return int(digits)


@lru_cache(maxsize=512)
def parse_format(fmt: str) -> tuple[str | Conversion, ...]:
    tokens: list[str | Conversion] = []
    offset = 0
    while offset < len(fmt):
        start = fmt.find("%", offset)
        if start < 0:
            tokens.append(fmt[offset:])
            break
        tokens.append(fmt[offset:start])
        if fmt.startswith("%%", start):
            tokens.append("%")
            offset = start + 2
            continue
        match = PRINTF_SPEC.match(fmt, start)
        if match is None or match[5] == "%":
            raise FormattingError(
                f"unsupported printf conversion at offset {start}: {fmt[start : start + 16]!r}"
            )
        tokens.append(
            Conversion(
                match[1],
                field_size(match[2] or None),
                field_size(match[3]),
                match[4] or "",
                match[5],
            )
        )
        offset = match.end()
    return tuple(tokens)


def integer_value(param: LogParam) -> int:
    val = param.value
    if param.type_tag not in INTEGER_TAGS or not isinstance(val, int):
        raise FormattingError(f"expected integer, got {param.type_tag.name}")
    return val


def pad_field(text: str, width: int, flags: str, size: int | None = None) -> str:
    padding = " " * max(0, width - (len(text) if size is None else size))
    return text + padding if "-" in flags else padding + text


def number_sign(negative: bool, flags: str) -> str:
    if negative:
        return "-"
    if "+" in flags:
        return "+"
    return " " if " " in flags else ""


def format_integer(
    value: int, spec: Conversion, width: int, precision: int | None
) -> str:
    bits = INTEGER_BITS[spec.length]
    number = value & ((1 << bits) - 1)
    signed = spec.kind in "di"
    if signed and number & (1 << (bits - 1)):
        number -= 1 << bits
    sign = number_sign(number < 0, spec.flags) if signed else ""
    base = {"o": "o", "x": "x", "X": "X"}.get(spec.kind, "d")
    digits = format(abs(number), base)
    if precision is not None:
        digits = "" if number == 0 and precision == 0 else digits.zfill(precision)
    prefix = ""
    if "#" in spec.flags:
        if spec.kind == "o" and not digits.startswith("0"):
            prefix = "0"
        elif spec.kind in "xX" and number:
            prefix = "0" + spec.kind
    if "0" in spec.flags and "-" not in spec.flags and precision is None:
        digits = digits.zfill(max(0, width - len(sign + prefix)))
    return pad_field(sign + prefix + digits, width, spec.flags)


def format_parameter(
    param: LogParam, spec: Conversion, width: int, precision: int | None
) -> str:
    kind = spec.kind
    if kind in "diouxX":
        if spec.length not in INTEGER_BITS:
            raise FormattingError(f"unsupported integer length {spec.length!r}")
        value = integer_value(param)
        arg_bits = max(32, INTEGER_BITS[spec.length])
        param_bits = INTEGER_TAGS.get(param.type_tag, 32)
        if param_bits > arg_bits:
            raise FormattingError(
                f"{param.type_tag.name} ({param_bits}-bit) is too wide for %{spec.length}{kind}"
            )
        return format_integer(value, spec, width, precision)

    if kind in "fFeEgG":
        val_float = param.value
        if (
            spec.length not in ("", "l")
            or param.type_tag not in (DataTypeTag.FLOAT, DataTypeTag.DOUBLE)
            or not isinstance(val_float, (float, int))
        ):
            raise FormattingError(f"%{spec.length}{kind} requires float or double")
        val_float = float(val_float)
        if not math.isfinite(val_float):
            name = "nan" if math.isnan(val_float) else "inf"
            if kind.isupper():
                name = name.upper()
            sign = number_sign(math.copysign(1.0, val_float) < 0, spec.flags)
            return pad_field(sign + name, width, spec.flags)
        precision_text = "" if precision is None else f".{precision}"
        return (f"%{spec.flags}{width or ''}{precision_text}{kind}") % val_float

    if spec.length:
        raise FormattingError(f"unsupported length {spec.length!r} for %{kind}")

    if kind == "s":
        if param.type_tag == DataTypeTag.STATIC_STRING:
            text = param.text
            if text is None:
                h_str = f"0x{param.hash:08x}" if param.hash is not None else "unknown"
                text = f"<unresolved hash {h_str}>"
        elif param.type_tag == DataTypeTag.IMM_STRING:
            text = str(param.value)
        else:
            raise FormattingError(
                f"%s requires a string, got {param.type_tag.name}"
            )
        raw = text.encode("utf-8")
        if precision is not None:
            raw = raw[:precision]
        return pad_field(
            raw.decode("utf-8", errors="replace"), width, spec.flags, len(raw)
        )

    if kind == "c":
        if precision is not None:
            raise FormattingError("precision is unsupported for %c")
        return pad_field(chr(integer_value(param) & 0xFF), width, spec.flags)

    if kind == "p":
        if param.type_tag == DataTypeTag.RAW_POINTER:
            addr = param.value if isinstance(param.value, int) else int(str(param.value), 0)
        else:
            raise FormattingError(f"%p requires a pointer, got {param.type_tag.name}")
        digits = f"{addr:08x}".zfill(precision or 0)
        if "0" in spec.flags and "-" not in spec.flags:
            digits = digits.zfill(max(0, width - 2))
        return pad_field("0x" + digits, width, spec.flags)

    raise FormattingError(f"unsupported conversion %{kind}")


def format_content(fmt: str, params: Sequence[LogParam]) -> str:
    """Apply C printf conversions and format arguments into plain text."""
    fragments: list[str] = []
    index = 0

    def take_parameter() -> LogParam:
        nonlocal index
        if index >= len(params):
            raise FormattingError(f"missing parameter {index}")
        param = params[index]
        index += 1
        return param

    for token in parse_format(fmt):
        if isinstance(token, str):
            fragments.append(token)
            continue
        flags = token.flags
        width = (
            integer_value(take_parameter()) if token.width == "*" else token.width or 0
        )
        precision = (
            integer_value(take_parameter())
            if token.precision == "*"
            else token.precision
        )
        if width < 0:
            flags += "-"
            width = -width
        if precision is not None and precision < 0:
            precision = None
        if width > MAX_FIELD_SIZE or (
            precision is not None and precision > MAX_FIELD_SIZE
        ):
            raise FormattingError(f"width/precision exceeds {MAX_FIELD_SIZE}")
        spec = Conversion(flags, width, precision, token.length, token.kind)
        fragments.append(format_parameter(take_parameter(), spec, width, precision))

    if index != len(params):
        raise FormattingError(f"{len(params) - index} unused parameters")
    return "".join(fragments)
