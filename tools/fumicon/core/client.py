# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Asynchronous FumiClient with strict Stop-and-Wait command scheduling."""

from __future__ import annotations

import asyncio
import secrets
import time
from collections.abc import Callable, Sequence
from contextlib import suppress
from types import TracebackType
from typing import Any, Self

from .metadata import FirmwareMetadata
from .models import (
    Channel,
    CmdOpcode,
    DataTypeTag,
    DeviceInfo,
    DeviceStats,
    LogLevel,
    LogRecord,
    PanicEnded,
    PanicEvent,
    PanicStarted,
    PanicText,
    RawFrame,
    Response,
)
from .protocol import (
    DeviceError,
    ProtocolError,
    decode_get_stats_response,
    decode_host_hello_response,
    decode_log,
    decode_ping_response,
    decode_response,
    encode_command,
    raise_if_error,
)
from .transport import DuplexStream, async_read_chunk, async_write_all
from .wire import StreamParser, encode_wire


class FumiClient:
    """Async Client engine managing connection, log streaming, and command transactions."""

    def __init__(
        self, stream: DuplexStream, metadata: FirmwareMetadata | None = None
    ) -> None:
        self._stream = stream
        self._metadata = metadata
        self._parser = StreamParser()

        self._running = False
        self._rx_task: asyncio.Task[None] | None = None

        self._tx_lock = asyncio.Lock()
        self._active_tx_id: int | None = None
        self._pending_future: asyncio.Future[Response] | None = None
        self._next_tx_id = secrets.randbelow(0x10000)

        self._log_listeners: list[Callable[[LogRecord], None]] = []
        self._panic_listeners: list[Callable[[PanicEvent], None]] = []
        self._diag_listeners: list[Callable[[str], None]] = []

        self.device_info: DeviceInfo | None = None

    def add_log_listener(self, cb: Callable[[LogRecord], None]) -> None:
        """Register a callback for incoming log messages."""
        if cb not in self._log_listeners:
            self._log_listeners.append(cb)

    def remove_log_listener(self, cb: Callable[[LogRecord], None]) -> None:
        """Unregister a log message callback."""
        if cb in self._log_listeners:
            self._log_listeners.remove(cb)

    def add_panic_listener(self, cb: Callable[[PanicEvent], None]) -> None:
        """Register a callback for emergency panic events."""
        if cb not in self._panic_listeners:
            self._panic_listeners.append(cb)

    def remove_panic_listener(self, cb: Callable[[PanicEvent], None]) -> None:
        """Unregister a panic event callback."""
        if cb in self._panic_listeners:
            self._panic_listeners.remove(cb)

    def add_diagnostic_listener(self, cb: Callable[[str], None]) -> None:
        """Register a callback for dropped frames or framing warnings."""
        if cb not in self._diag_listeners:
            self._diag_listeners.append(cb)

    def remove_diagnostic_listener(self, cb: Callable[[str], None]) -> None:
        """Unregister a diagnostic callback."""
        if cb in self._diag_listeners:
            self._diag_listeners.remove(cb)

    def _emit_diag(self, msg: str) -> None:
        for cb in self._diag_listeners:
            with suppress(Exception):
                cb(msg)

    def start(self) -> None:
        """Start background receiver task."""
        if not self._running:
            self._running = True
            self._rx_task = asyncio.create_task(
                self._rx_worker(), name="fumi-client-rx"
            )

    async def close(self) -> None:
        """Stop background receiver and clean up active transactions."""
        self._running = False
        if self._rx_task is not None:
            self._rx_task.cancel()
            try:
                await self._rx_task
            except asyncio.CancelledError:
                pass
            self._rx_task = None

        if self._pending_future is not None and not self._pending_future.done():
            self._pending_future.cancel()
            self._pending_future = None

    async def __aenter__(self) -> Self:
        self.start()
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        await self.close()

    async def _rx_worker(self) -> None:
        """Continuously read bytes from transport and dispatch parsed events."""
        try:
            while self._running:
                chunk = await async_read_chunk(self._stream)
                for event in self._parser.feed(chunk):
                    self._handle_event(event)
        except asyncio.CancelledError:
            pass
        except OSError as exc:
            self._emit_diag(f"transport read error: {exc}")
            raise

    def _handle_event(self, event: Any) -> None:
        if isinstance(event, RawFrame):
            if event.channel == Channel.LOG:
                try:
                    record = decode_log(event, self._metadata)
                    for log_cb in self._log_listeners:
                        log_cb(record)
                except (ProtocolError, ValueError) as exc:
                    self._emit_diag(f"cannot decode log frame: {exc}")
            elif event.channel == Channel.RESPONSE:
                try:
                    resp = decode_response(event.payload)
                    if (
                        self._pending_future is not None
                        and not self._pending_future.done()
                        and self._active_tx_id == resp.transaction_id
                    ):
                        self._pending_future.set_result(resp)
                    else:
                        self._emit_diag(
                            f"untracked response for tx {resp.transaction_id}"
                        )
                except (ProtocolError, ValueError) as exc:
                    self._emit_diag(f"cannot decode response: {exc}")
            else:
                self._emit_diag(f"unhandled channel 0x{event.channel:02x}")

        elif isinstance(event, (PanicStarted, PanicText, PanicEnded)):
            for panic_cb in self._panic_listeners:
                panic_cb(event)

        elif isinstance(event, ProtocolError):
            self._emit_diag(f"protocol error: {event}")

    async def send_command(
        self,
        opcode: int | CmdOpcode,
        operands: Sequence[tuple[DataTypeTag, Any]] = (),
        timeout: float = 1.0,
    ) -> Response:
        """Execute a command following the strict Stop-and-Wait single transaction model."""
        async with self._tx_lock:
            tx_id = self._next_tx_id
            self._next_tx_id = (self._next_tx_id + 1) & 0xFFFF

            loop = asyncio.get_running_loop()
            self._active_tx_id = tx_id
            self._pending_future = loop.create_future()

            proto_frame = encode_command(opcode, tx_id, operands)
            wire_bytes = encode_wire(proto_frame)

            await async_write_all(self._stream, wire_bytes)

            try:
                resp = await asyncio.wait_for(self._pending_future, timeout=timeout)
                raise_if_error(resp)
                return resp
            finally:
                self._active_tx_id = None
                self._pending_future = None

    async def handshake(
        self, timeout: float = 1.0, max_retries: int = 3
    ) -> DeviceInfo:
        """Perform HostHello handshake and strictly verify firmware identity."""
        last_error: Exception | None = None
        for attempt in range(max_retries):
            try:
                resp = await self.send_command(CmdOpcode.HOST_HELLO, timeout=timeout)
                dev = decode_host_hello_response(resp)
                if self._metadata is not None and not self._metadata.verify_build_id(
                    dev.build_id
                ):
                    expected_hex = self._metadata.build_id_hex or "unknown"
                    raise ValueError(
                        f"build_id mismatch: device is {dev.build_id.hex()}, "
                        f"metadata expected {expected_hex}"
                    )
                self.device_info = dev
                return dev
            except (TimeoutError, ProtocolError, DeviceError) as exc:
                last_error = exc

        raise TimeoutError(
            f"HostHello handshake failed after {max_retries} attempts: {last_error}"
        )

    async def set_log_level(self, level: LogLevel, timeout: float = 1.0) -> None:
        """Change the firmware log filter level dynamically."""
        await self.send_command(
            CmdOpcode.SET_LOG_LEVEL,
            operands=((DataTypeTag.U8, int(level)),),
            timeout=timeout,
        )

    async def get_stats(self, timeout: float = 1.0) -> DeviceStats:
        """Query firmware Logger and packet drop statistics."""
        resp = await self.send_command(CmdOpcode.GET_STATS, timeout=timeout)
        return decode_get_stats_response(resp)

    async def ping(self, timeout: float = 1.0) -> tuple[str, float]:
        """Send Ping command and measure round-trip time in milliseconds."""
        t0 = time.perf_counter()
        resp = await self.send_command(CmdOpcode.PING, timeout=timeout)
        rtt_ms = (time.perf_counter() - t0) * 1000.0
        reply = decode_ping_response(resp)
        return reply, rtt_ms

