# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""COBS framing, CRC16-CCITT calculation, stream framing and Panic mode detection."""

from __future__ import annotations

import binascii
import codecs
import struct
import time
from collections.abc import Iterator
from typing import TypeAlias

from .models import (
    Channel,
    PanicEnded,
    PanicEvent,
    PanicStarted,
    PanicText,
    RawFrame,
)
from .protocol import (
    CRC_BYTES,
    FRAME_HEADER_STRUCT,
    MAGIC,
    MAX_FRAME_BYTES,
    VERSION,
    ProtocolError,
)

MAX_COBS_INPUT_BYTES = MAX_FRAME_BYTES + CRC_BYTES
COBS_HEADROOM = MAX_COBS_INPUT_BYTES // 254 + 1
MAX_WIRE_BYTES = MAX_COBS_INPUT_BYTES + COBS_HEADROOM + 1  # Including 0x00 delimiter

PANIC_BEGIN = b"\x00\r\n=== DEVICE PANIC BEGIN ===\r\n"
PANIC_END = b"\r\n=== DEVICE PANIC END ===\r\n\x00"
PANIC_TIMEOUT_SECONDS = 3.0


class OversizedFrameError(ProtocolError):
    """Received frame exceeded buffer limit."""


class CrcMismatchError(ProtocolError):
    """Calculated CRC does not match received CRC."""


def cobs_encode(data: bytes) -> bytes:
    """Encode arbitrary bytes using Consistent Overhead Byte Stuffing (no trailing zero)."""
    encoded = bytearray(b"\0")
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            encoded[code_index] = code
            code_index = len(encoded)
            encoded.append(0)
            code = 1
        else:
            encoded.append(byte)
            code += 1
            if code == 0xFF:
                encoded[code_index] = code
                code_index = len(encoded)
                encoded.append(0)
                code = 1
    encoded[code_index] = code
    if code == 1 and data and data[-1] != 0:
        encoded.pop()
    return bytes(encoded)


def cobs_decode(encoded: bytes | bytearray) -> bytes:
    """Decode a COBS-encoded block (excluding delimiters)."""
    if not encoded:
        raise ProtocolError("empty COBS block")
    result = bytearray()
    offset = 0
    while offset < len(encoded):
        code = encoded[offset]
        end = offset + code
        if code == 0 or end > len(encoded):
            raise ProtocolError("invalid COBS block pointer")
        block = encoded[offset + 1 : end]
        if 0 in block:
            raise ProtocolError("zero byte inside COBS block")
        result.extend(block)
        offset = end
        if code != 0xFF and offset < len(encoded):
            result.append(0)
    return bytes(result)


def compute_crc16(data: bytes) -> int:
    """Compute CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection)."""
    return binascii.crc_hqx(data, 0xFFFF)


def encode_wire(frame_bytes: bytes) -> bytes:
    """Append little-endian CRC16, COBS encode, and append 0x00 delimiter."""
    crc = compute_crc16(frame_bytes)
    with_crc = frame_bytes + struct.pack("<H", crc)
    return cobs_encode(with_crc) + b"\x00"


def decode_frame(encoded_block: bytes | bytearray) -> RawFrame:
    """Decode a single delimiter-stripped COBS frame, verify CRC and header."""
    if len(encoded_block) + 1 > MAX_WIRE_BYTES:
        raise OversizedFrameError(f"wire frame exceeds {MAX_WIRE_BYTES} bytes")

    frame = cobs_decode(encoded_block)
    if len(frame) > MAX_COBS_INPUT_BYTES:
        raise OversizedFrameError(f"protocol frame exceeds {MAX_FRAME_BYTES} bytes")
    if len(frame) < FRAME_HEADER_STRUCT.size + CRC_BYTES:
        raise ProtocolError("truncated frame header or CRC")

    magic, version, channel_raw, payload_len = FRAME_HEADER_STRUCT.unpack_from(frame)
    if magic != MAGIC:
        raise ProtocolError(f"invalid magic 0x{magic:02x}")
    if version != VERSION:
        raise ProtocolError(f"unsupported version {version}")
    try:
        channel = Channel(channel_raw)
    except ValueError as exc:
        raise ProtocolError(f"unknown channel {channel_raw}") from exc

    if len(frame) != FRAME_HEADER_STRUCT.size + payload_len + CRC_BYTES:
        raise ProtocolError(f"payload length mismatch (declared {payload_len})")

    received_crc = struct.unpack_from("<H", frame, len(frame) - CRC_BYTES)[0]
    expected_crc = compute_crc16(frame[:-CRC_BYTES])
    if received_crc != expected_crc:
        raise CrcMismatchError(
            f"CRC mismatch: received 0x{received_crc:04x}, expected 0x{expected_crc:04x}"
        )

    payload = frame[FRAME_HEADER_STRUCT.size : -CRC_BYTES]
    return RawFrame(
        channel=channel,
        payload=payload,
        wire_length=len(encoded_block) + 1,
        crc16=received_crc,
    )


StreamEvent: TypeAlias = RawFrame | PanicEvent | ProtocolError


class StreamParser:
    """Sans-I/O incremental stream parser for Fumi.

    Feeds raw bytes and yields parsed RawFrame instances, Panic events, or ProtocolErrors.
    Accurately recognizes emergency ASCII panic banners out-of-band.
    """

    def __init__(self) -> None:
        self._pending = bytearray()
        self._discarding = False
        self._begin = bytearray()
        self._end = bytearray()
        self._panic_start_time: float | None = None
        self._text_decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")

    @property
    def in_panic(self) -> bool:
        return self._panic_start_time is not None

    def _leave_panic(self, error: str | None = None) -> Iterator[PanicEvent]:
        tail = self._text_decoder.decode(b"", final=True)
        if tail:
            yield PanicText(tail)
        self._panic_start_time = None
        self._end.clear()
        self._begin.clear()
        self._pending.clear()
        self._discarding = False
        yield PanicEnded(error)

    def _emit_panic_text(self, data: bytes | bytearray) -> Iterator[PanicText]:
        text = self._text_decoder.decode(data)
        if text:
            yield PanicText(text)

    def feed(self, chunk: bytes) -> Iterator[StreamEvent]:
        """Feed a chunk of bytes from the wire and yield all parsed events."""
        if (
            self._panic_start_time is not None
            and time.monotonic() - self._panic_start_time >= PANIC_TIMEOUT_SECONDS
        ):
            yield from self._leave_panic("panic end marker timeout (3s)")

        pending_panic_text = bytearray()

        for byte in chunk:
            if self.in_panic:
                self._end.append(byte)
                if len(self._end) > len(PANIC_END):
                    del self._end[0]
                if self._end == PANIC_END:
                    yield from self._emit_panic_text(pending_panic_text)
                    pending_panic_text.clear()
                    yield from self._leave_panic()
                else:
                    pending_panic_text.append(byte)
                continue

            if self._begin:
                if byte == PANIC_BEGIN[len(self._begin)]:
                    self._begin.append(byte)
                    if self._begin == PANIC_BEGIN:
                        self._begin.clear()
                        self._pending.clear()
                        self._discarding = False
                        self._text_decoder.reset()
                        self._panic_start_time = time.monotonic()
                        yield PanicStarted()
                        yield PanicText(PANIC_BEGIN[1:].decode("ascii"))
                    continue
                # False panic start marker: replay buffered bytes into frame decoder
                held = bytes(self._begin[1:])
                self._begin.clear()
                yield from self._process_frame_bytes(held)

            if byte == 0:
                self._begin.append(0)

            yield from self._process_frame_bytes(bytes((byte,)))

        if pending_panic_text:
            yield from self._emit_panic_text(pending_panic_text)

    def _process_frame_bytes(self, data: bytes) -> Iterator[StreamEvent]:
        for byte in data:
            if byte == 0:
                block = bytes(self._pending)
                discarding = self._discarding
                self._pending.clear()
                self._discarding = False
                if block and not discarding:
                    try:
                        yield decode_frame(block)
                    except ProtocolError as exc:
                        yield exc
            elif not self._discarding:
                self._pending.append(byte)
                if len(self._pending) >= MAX_WIRE_BYTES:
                    self._pending.clear()
                    self._discarding = True
                    yield OversizedFrameError("oversized wire frame discarded")

    def finish(self) -> Iterator[StreamEvent]:
        """Signal EOF on stream; checks for incomplete frames or unterminated panic."""
        if self.in_panic:
            yield from self._leave_panic("stream ended before panic end marker")
        elif self._pending or self._discarding:
            yield ProtocolError("incomplete frame at stream EOF")
