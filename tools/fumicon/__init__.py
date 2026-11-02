# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Fumi - Embedded logging, diagnostics, and waveform trace protocol engine."""

from fumicon.core.client import FumiClient
from fumicon.core.metadata import FirmwareMetadata
from fumicon.core.models import (
    Channel,
    CmdOpcode,
    DataTypeTag,
    DeviceInfo,
    DeviceStats,
    ErrorCode,
    LogLevel,
    LogParam,
    LogRecord,
    PanicEnded,
    PanicEvent,
    PanicStarted,
    PanicText,
    RawFrame,
    Response,
    ResponseFlag,
)
from fumicon.core.protocol import (
    DeviceError,
    ProtocolError,
    decode_log,
    decode_ping_response,
    decode_response,
    encode_command,
    encode_frame,
    encode_ping_request,
)
from fumicon.core.wire import (
    StreamParser,
    cobs_decode,
    cobs_encode,
    compute_crc16,
    decode_frame,
    encode_wire,
)

__all__ = [
    "Channel",
    "CmdOpcode",
    "DataTypeTag",
    "DeviceError",
    "DeviceInfo",
    "DeviceStats",
    "ErrorCode",
    "FirmwareMetadata",
    "FumiClient",
    "LogLevel",
    "LogParam",
    "LogRecord",
    "PanicEnded",
    "PanicEvent",
    "PanicStarted",
    "PanicText",
    "ProtocolError",
    "RawFrame",
    "Response",
    "ResponseFlag",
    "StreamParser",
    "cobs_decode",
    "cobs_encode",
    "compute_crc16",
    "decode_frame",
    "decode_log",
    "decode_ping_response",
    "decode_response",
    "encode_command",
    "encode_frame",
    "encode_ping_request",
    "encode_wire",
]
