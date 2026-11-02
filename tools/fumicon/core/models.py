# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Strongly typed domain models and protocol enums for Fumi."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Any, TypeAlias


class Channel(IntEnum):
    """Protocol layer channel IDs."""

    LOG = 0x01
    CMD = 0x02
    RESPONSE = 0x03


class LogLevel(IntEnum):
    """Severity levels for log events matching firmware FUMI_LEVEL_*."""

    DEBUG = 0x00
    INFO = 0x01
    WARN = 0x02
    ERROR = 0x03
    SILENT = 0xFF


class ResponseFlag(IntFlag):
    """Flags carried in the Response channel header."""

    DONE = 0x01
    ERR = 0x02


class ErrorCode(IntEnum):
    """Standard Fumi protocol error codes."""

    ILL_FORMED_PACKET = 0x00
    INVALID_OPCODE = 0x01
    INVALID_OPERANDS = 0x02
    EXECUTION = 0x03


ERROR_NAMES: dict[int, str] = {
    ErrorCode.ILL_FORMED_PACKET: "ill-formed packet",
    ErrorCode.INVALID_OPCODE: "invalid opcode",
    ErrorCode.INVALID_OPERANDS: "invalid operands",
    ErrorCode.EXECUTION: "execution error"
}


class CmdOpcode(IntEnum):
    """Protocol-defined command opcodes (strictly standardized)."""

    HOST_HELLO = 0x01
    SET_LOG_LEVEL = 0x02
    GET_STATS = 0x04
    PING = 0x05

    # Reserved Trace Subsystem opcodes, may change in the future
    TRACE_GET_STATUS = 0x10
    TRACE_GET_SCHEMA = 0x11
    TRACE_ARM = 0x12
    TRACE_DISARM = 0x13
    TRACE_FORCE_TRIGGER = 0x14
    TRACE_PULL_CHUNK = 0x15


class DataTypeTag(IntEnum):
    """Tag identifier for Tagged<any> and Tagged<primitive> data values."""

    # Primitives (fixed length)
    I8 = 0x00
    I16 = 0x01
    I32 = 0x02
    I64 = 0x03
    U8 = 0x04
    U16 = 0x05
    U32 = 0x06
    U64 = 0x07
    BOOL = 0x08
    FLOAT = 0x09
    DOUBLE = 0x0A
    STATIC_STRING = 0x0B
    RAW_POINTER = 0x0C

    # Non-primitives (length-prefixed)
    IMM_STRING = 0x20
    IMM_BYTES = 0x21


@dataclass(frozen=True, slots=True)
class RawFrame:
    """A completely validated wire frame before channel-specific decoding."""

    channel: Channel
    payload: bytes
    wire_length: int
    crc16: int


@dataclass(frozen=True, slots=True)
class LogParam:
    """A single decoded parameter inside a log message."""

    type_tag: DataTypeTag
    value: Any = None
    hash: int | None = None
    text: str | None = None


@dataclass(frozen=True, slots=True)
class LogRecord:
    """A fully parsed log event with resolved strings and parameters."""

    level: LogLevel
    timestamp_ticks: int
    tag_hash: int
    tag_text: str | None
    format_hash: int
    format_text: str | None
    params: tuple[LogParam, ...]
    formatted_message: str | None = None


@dataclass(frozen=True, slots=True)
class Command:
    """A command envelope to be serialized into Channel::kCmd."""

    opcode: CmdOpcode | int
    transaction_id: int
    operands: tuple[tuple[DataTypeTag, Any], ...] = ()


@dataclass(frozen=True, slots=True)
class Response:
    """A response packet received from Channel::kResponse."""

    transaction_id: int
    sequence: int
    flags: ResponseFlag
    body: bytes


@dataclass(frozen=True, slots=True)
class DeviceInfo:
    """Device identity verified during HostHello handshake."""

    name: str
    description: str
    build_id: bytes


@dataclass(frozen=True, slots=True)
class DeviceStats:
    """Logger and transport statistics queried from the device."""

    sent: int
    uart_tx_error: int
    encode_error: int
    resource_unavail_error: int
    drop_no_free_slot: int
    drop_in_isr: int
    drop_unknown: int


@dataclass(frozen=True, slots=True)
class PanicStarted:
    """Signal indicating entry into emergency panic text mode."""


@dataclass(frozen=True, slots=True)
class PanicText:
    """Chunk of emergency panic ASCII text received from the device."""

    text: str


@dataclass(frozen=True, slots=True)
class PanicEnded:
    """Signal indicating exit from panic text mode."""

    error: str | None = None


PanicEvent: TypeAlias = PanicStarted | PanicText | PanicEnded
