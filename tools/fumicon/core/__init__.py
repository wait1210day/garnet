# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Fumicon core protocol, codecs, data models and async client."""

from .client import FumiClient
from .metadata import FirmwareMetadata
from .models import (
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
from .protocol import (
    DeviceError,
    ProtocolError,
    decode_log,
    decode_ping_response,
    decode_response,
    encode_command,
    encode_frame,
    encode_ping_request,
)
from .wire import (
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
