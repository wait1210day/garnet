# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Protocol constants, codecs, packet serializers and deserializers for Fumi."""

from __future__ import annotations

import struct
from collections.abc import Sequence
from typing import Any

from .metadata import FirmwareMetadata
from .models import (
    ERROR_NAMES,
    Channel,
    CmdOpcode,
    DataTypeTag,
    DeviceInfo,
    DeviceStats,
    LogLevel,
    LogParam,
    LogRecord,
    RawFrame,
    Response,
    ResponseFlag,
)
from .printf import FormattingError, format_content

MAGIC = 0xD0
VERSION = 0x01
MAX_FRAME_BYTES = 512
CRC_BYTES = 2

# [Magic:u8] [Version:u8] [ChannelId:u8] [PayloadLen:u16]
FRAME_HEADER_STRUCT = struct.Struct("<BBBH")

# [Opcode:u8] [TransactionId:u16] [NbOperands:u8]
COMMAND_HEADER_STRUCT = struct.Struct("<BHB")

# [TransactionId:u16] [RespSeq:u16] [Flags:u8]
RESPONSE_HEADER_STRUCT = struct.Struct("<HHB")

# [Level:u8] [Timestamp:u64] [TagStr:sstr/u32] [FormatStr:sstr/u32] [NbParams:u8]
LOG_HEADER_STRUCT = struct.Struct("<BQIIB")

# [Sent:u32] [TxErr:u32] [EncErr:u32] [ResErr:u32] [DropSlot:u32] [DropIsr:u32] [DropUnk:u32]
STATS_STRUCT = struct.Struct("<IIIIIII")

PRIMITIVE_SPECS: dict[DataTypeTag, tuple[str, int]] = {
    DataTypeTag.I8: ("<b", 1),
    DataTypeTag.I16: ("<h", 2),
    DataTypeTag.I32: ("<i", 4),
    DataTypeTag.I64: ("<q", 8),
    DataTypeTag.U8: ("<B", 1),
    DataTypeTag.U16: ("<H", 2),
    DataTypeTag.U32: ("<I", 4),
    DataTypeTag.U64: ("<Q", 8),
    DataTypeTag.BOOL: ("<?", 1),
    DataTypeTag.FLOAT: ("<f", 4),
    DataTypeTag.DOUBLE: ("<d", 8),
    DataTypeTag.STATIC_STRING: ("<I", 4),
    DataTypeTag.RAW_POINTER: ("<I", 4),
}


class ProtocolError(ValueError):
    """Base error for protocol violation or ill-formed packet."""


class DeviceError(ProtocolError):
    """An error response returned explicitly by the target device."""

    def __init__(self, response: Response, code: int) -> None:
        self.response = response
        self.code = code
        name = ERROR_NAMES.get(code, f"unknown error (0x{code:02x})")
        super().__init__(f"device error: {name}")


def encode_tagged_operand(tag: DataTypeTag, value: Any) -> bytes:
    """Encode a single Tagged<any> value into [tag:u8] [data...]."""
    if tag in PRIMITIVE_SPECS:
        fmt, _ = PRIMITIVE_SPECS[tag]
        try:
            return bytes((tag,)) + struct.pack(fmt, value)
        except (struct.error, OverflowError, TypeError) as exc:
            raise ProtocolError(f"cannot encode {value!r} as {tag.name}: {exc}") from exc

    if tag == DataTypeTag.IMM_STRING:
        if isinstance(value, str):
            encoded = value.encode("utf-8")
        elif isinstance(value, (bytes, bytearray)):
            encoded = bytes(value)
        else:
            raise ProtocolError(f"expected str/bytes for IMM_STRING, got {type(value)}")
        if len(encoded) > 0xFFFF:
            raise ProtocolError(f"IMM_STRING length {len(encoded)} exceeds 65535")
        return bytes((tag,)) + struct.pack("<H", len(encoded)) + encoded

    if tag == DataTypeTag.IMM_BYTES:
        if not isinstance(value, (bytes, bytearray)):
            raise ProtocolError(f"expected bytes for IMM_BYTES, got {type(value)}")
        if len(value) > 0xFFFF:
            raise ProtocolError(f"IMM_BYTES length {len(value)} exceeds 65535")
        return bytes((tag,)) + struct.pack("<H", len(value)) + bytes(value)

    raise ProtocolError(f"unsupported DataTypeTag 0x{tag:02x}")


def decode_tagged_value(data: bytes, offset: int = 0) -> tuple[DataTypeTag, Any, int]:
    """Decode a Tagged<any> value at offset. Returns (tag, value, next_offset)."""
    if offset >= len(data):
        raise ProtocolError("unexpected end of buffer reading DataTypeTag")
    try:
        tag = DataTypeTag(data[offset])
    except ValueError as exc:
        raise ProtocolError(f"invalid DataTypeTag 0x{data[offset]:02x}") from exc
    offset += 1

    if tag in PRIMITIVE_SPECS:
        fmt, size = PRIMITIVE_SPECS[tag]
        if offset + size > len(data):
            raise ProtocolError(f"truncated primitive {tag.name}")
        val = struct.unpack_from(fmt, data, offset)[0]
        return tag, val, offset + size

    if tag in (DataTypeTag.IMM_STRING, DataTypeTag.IMM_BYTES):
        if offset + 2 > len(data):
            raise ProtocolError(f"truncated length header for {tag.name}")
        length = struct.unpack_from("<H", data, offset)[0]
        offset += 2
        if offset + length > len(data):
            raise ProtocolError(f"truncated payload for {tag.name}")
        raw_val = data[offset : offset + length]
        if tag == DataTypeTag.IMM_STRING:
            try:
                str_val = raw_val.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise ProtocolError("invalid UTF-8 in IMM_STRING") from exc
            return tag, str_val, offset + length
        return tag, raw_val, offset + length

    raise ProtocolError(f"unsupported DataTypeTag 0x{tag:02x}")


def encode_frame(channel: Channel, payload: bytes) -> bytes:
    """Build a protocol layer frame: [Magic] [Version] [Channel] [Length] [Payload]."""
    total_len = FRAME_HEADER_STRUCT.size + len(payload)
    if total_len > MAX_FRAME_BYTES:
        raise ProtocolError(
            f"frame size {total_len} exceeds MAX_FRAME_BYTES ({MAX_FRAME_BYTES})"
        )
    return FRAME_HEADER_STRUCT.pack(MAGIC, VERSION, channel, len(payload)) + payload


def decode_frame_header(frame_bytes: bytes) -> tuple[Channel, bytes]:
    """Validate frame header and return (channel, payload)."""
    if len(frame_bytes) < FRAME_HEADER_STRUCT.size:
        raise ProtocolError("frame smaller than header size")
    magic, version, channel_raw, declared_len = FRAME_HEADER_STRUCT.unpack_from(frame_bytes)
    if magic != MAGIC:
        raise ProtocolError(f"invalid magic byte 0x{magic:02x} (expected 0x{MAGIC:02x})")
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    try:
        channel = Channel(channel_raw)
    except ValueError as exc:
        raise ProtocolError(f"unknown channel ID 0x{channel_raw:02x}") from exc

    payload = frame_bytes[FRAME_HEADER_STRUCT.size :]
    if len(payload) != declared_len:
        raise ProtocolError(
            f"payload length mismatch: declared {declared_len}, actual {len(payload)}"
        )
    return channel, payload


def encode_command(
    opcode: CmdOpcode | int,
    transaction_id: int,
    operands: Sequence[tuple[DataTypeTag, Any]] = (),
) -> bytes:
    """Serialize a complete command into a frame ready for wire encoding."""
    if not (0 <= opcode <= 0xFF):
        raise ProtocolError(f"invalid opcode {opcode}")
    if not (0 <= transaction_id <= 0xFFFF):
        raise ProtocolError(f"invalid transaction_id {transaction_id}")
    if len(operands) > 0xFF:
        raise ProtocolError("too many operands")

    encoded_operands = b"".join(
        encode_tagged_operand(tag, val) for tag, val in operands
    )
    cmd_header = COMMAND_HEADER_STRUCT.pack(opcode, transaction_id, len(operands))
    return encode_frame(Channel.CMD, cmd_header + encoded_operands)


def decode_response(payload: bytes) -> Response:
    """Parse the Response channel payload into a Response object."""
    if len(payload) < RESPONSE_HEADER_STRUCT.size:
        raise ProtocolError("truncated response header")
    transaction_id, sequence, raw_flags = RESPONSE_HEADER_STRUCT.unpack_from(payload)
    try:
        flags = ResponseFlag(raw_flags)
    except ValueError as exc:
        raise ProtocolError(f"invalid response flags 0x{raw_flags:02x}") from exc

    body = payload[RESPONSE_HEADER_STRUCT.size :]
    return Response(
        transaction_id=transaction_id,
        sequence=sequence,
        flags=flags,
        body=body,
    )


def raise_if_error(response: Response) -> None:
    """Raise DeviceError if response has ERR flag set."""
    if response.flags & ResponseFlag.ERR:
        if len(response.body) < 1:
            raise ProtocolError("error response missing error code byte")
        raise DeviceError(response, response.body[0])


raise_for_error = raise_if_error


# Convenience Helpers for Standard Opcodes


def encode_host_hello_request(transaction_id: int) -> bytes:
    """Build HostHello query command."""
    return encode_command(CmdOpcode.HOST_HELLO, transaction_id)


def decode_host_hello_response(response: Response) -> DeviceInfo:
    """Parse HostHello response payload into DeviceInfo."""
    raise_if_error(response)
    data = response.body
    offset = 0

    # 1. device_name: istr
    if offset + 2 > len(data):
        raise ProtocolError("truncated device_name length")
    name_len = struct.unpack_from("<H", data, offset)[0]
    offset += 2
    if offset + name_len > len(data):
        raise ProtocolError("truncated device_name")
    name = data[offset : offset + name_len].decode("utf-8", errors="replace")
    offset += name_len

    # 2. device_desc: istr
    if offset + 2 > len(data):
        raise ProtocolError("truncated device_desc length")
    desc_len = struct.unpack_from("<H", data, offset)[0]
    offset += 2
    if offset + desc_len > len(data):
        raise ProtocolError("truncated device_desc")
    desc = data[offset : offset + desc_len].decode("utf-8", errors="replace")
    offset += desc_len

    # 3. build_id_sha1: ibytes
    if offset + 2 > len(data):
        raise ProtocolError("truncated build_id length")
    bid_len = struct.unpack_from("<H", data, offset)[0]
    offset += 2
    if offset + bid_len > len(data):
        raise ProtocolError("truncated build_id")
    build_id = data[offset : offset + bid_len]
    offset += bid_len

    if bid_len != 20:
        raise ProtocolError(f"expected 20-byte SHA-1 build_id, got {bid_len} bytes")
    if offset != len(data):
        raise ProtocolError(f"unexpected trailing {len(data) - offset} bytes in HostHello")

    return DeviceInfo(name=name, description=desc, build_id=build_id)


def encode_set_level_request(transaction_id: int, level: LogLevel) -> bytes:
    """Build SetLogLevel command."""
    return encode_command(
        CmdOpcode.SET_LOG_LEVEL,
        transaction_id,
        operands=((DataTypeTag.U8, int(level)),),
    )


def decode_get_stats_response(response: Response) -> DeviceStats:
    """Parse GetStats response payload into DeviceStats."""
    raise_if_error(response)
    if len(response.body) != STATS_STRUCT.size:
        raise ProtocolError(
            f"expected {STATS_STRUCT.size} bytes stats, got {len(response.body)}"
        )
    fields = STATS_STRUCT.unpack(response.body)
    return DeviceStats(*fields)


def encode_ping_request(transaction_id: int) -> bytes:
    """Build Ping command frame."""
    return encode_command(CmdOpcode.PING, transaction_id)


def decode_ping_response(response: Response) -> str:
    """Parse Ping response payload (fixed istr string)."""
    raise_if_error(response)
    data = response.body
    if len(data) < 2:
        raise ProtocolError("truncated ping response")
    length = struct.unpack_from("<H", data, 0)[0]
    if len(data) != 2 + length:
        raise ProtocolError(
            f"ping response length mismatch: declared {length}, actual {len(data) - 2}"
        )
    return data[2:].decode("utf-8", errors="replace")


def decode_log(
    frame: RawFrame, metadata: FirmwareMetadata | None = None
) -> LogRecord:
    """Decode a RawFrame on Channel::kLog into a LogRecord with resolved strings."""
    if frame.channel != Channel.LOG:
        raise ProtocolError(f"expected Channel::kLog (0x01), got 0x{frame.channel:02x}")

    payload = frame.payload
    if len(payload) < LOG_HEADER_STRUCT.size:
        raise ProtocolError("truncated log header")

    level_raw, timestamp, tag_hash, fmt_hash, count = LOG_HEADER_STRUCT.unpack_from(
        payload
    )
    try:
        level = LogLevel(level_raw)
    except ValueError:
        level = LogLevel.DEBUG

    params: list[LogParam] = []
    offset = LOG_HEADER_STRUCT.size

    for idx in range(count):
        if offset >= len(payload):
            raise ProtocolError(f"missing parameter {idx}")
        tag, val, offset = decode_tagged_value(payload, offset)
        if tag == DataTypeTag.STATIC_STRING:
            str_hash = int(val)
            text = metadata.lookup(str_hash) if metadata else None
            param = LogParam(
                type_tag=tag,
                value=str_hash,
                hash=str_hash,
                text=text,
            )
        elif tag == DataTypeTag.RAW_POINTER:
            param = LogParam(
                type_tag=tag,
                value=int(val),
            )
        else:
            param = LogParam(type_tag=tag, value=val)
        params.append(param)

    if offset != len(payload):
        raise ProtocolError(f"{len(payload) - offset} trailing log payload bytes")

    tag_text = metadata.lookup(tag_hash) if metadata else None
    fmt_text = metadata.lookup(fmt_hash) if metadata else None

    formatted_msg: str | None = None
    if fmt_text is not None:
        try:
            formatted_msg = format_content(fmt_text, params)
        except FormattingError:
            formatted_msg = None

    return LogRecord(
        level=level,
        timestamp_ticks=timestamp,
        tag_hash=tag_hash,
        tag_text=tag_text,
        format_hash=fmt_hash,
        format_text=fmt_text,
        params=tuple(params),
        formatted_message=formatted_msg,
    )

