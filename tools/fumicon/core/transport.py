# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Stream abstractions, serial port transport and async I/O adapters."""

from __future__ import annotations

import asyncio
import sys
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Protocol


class InputStream(Protocol):
    """Protocol for reading raw bytes from a source."""

    def read(self, size: int = -1, /) -> bytes: ...


class DuplexStream(InputStream, Protocol):
    """Protocol for bidirectional byte streams (e.g. serial ports, pipes)."""

    def write(self, data: bytes, /) -> int: ...
    def flush(self) -> None: ...


class SerialUnavailableError(ImportError):
    """Raised when pyserial is missing."""


@contextmanager
def open_serial(port: str, baud: int) -> Iterator[DuplexStream]:
    """Open a serial port with robust timeout settings."""
    try:
        import serial  # type: ignore[import-untyped]
    except ImportError as exc:
        raise SerialUnavailableError(
            "serial transport requires pyserial; install with: pip install pyserial"
        ) from exc

    try:
        with serial.Serial(
            port, baudrate=baud, timeout=0.05, write_timeout=1.0
        ) as stream:
            yield stream
    except serial.SerialException as exc:
        raise OSError(f"cannot open serial port {port}: {exc}") from exc


@contextmanager
def open_input_stream(
    *, port: str | None = None, path: str | None = None, baud: int = 115200
) -> Iterator[InputStream]:
    """Open a serial port, a binary capture file, or standard input."""
    if path == "-":
        yield sys.stdin.buffer
    elif path is not None:
        with open(path, "rb") as stream:
            yield stream
    elif port is not None:
        with open_serial(port, baud) as stream:
            yield stream
    else:
        raise ValueError("either a serial port or a capture file path is required")


def read_chunk(stream: InputStream, max_size: int = 4096) -> bytes:
    """Read available bytes without blocking indefinitely on live serial streams."""
    size = min(int(getattr(stream, "in_waiting", 1)) or 1, max_size)
    return stream.read(size)


def write_all(stream: DuplexStream, data: bytes) -> None:
    """Ensure complete delivery of a frame byte sequence."""
    offset = 0
    while offset < len(data):
        written = stream.write(data[offset:])
        if written <= 0:
            raise OSError("serial write returned 0 bytes")
        offset += written
    stream.flush()


async def async_read_chunk(stream: InputStream, max_size: int = 4096) -> bytes:
    """Async wrapper for read_chunk offloaded to threadpool."""
    return await asyncio.to_thread(read_chunk, stream, max_size)


async def async_write_all(stream: DuplexStream, data: bytes) -> None:
    """Async wrapper for write_all offloaded to threadpool."""
    await asyncio.to_thread(write_all, stream, data)
