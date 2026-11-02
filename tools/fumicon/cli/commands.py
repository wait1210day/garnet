# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Interactive REPL command handlers for Fumicon."""

from __future__ import annotations

import asyncio
import shlex
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import TypeAlias

from fumicon.core.client import FumiClient
from fumicon.core.models import LogLevel
from fumicon.core.protocol import DeviceError

from .terminal import Terminal

LEVEL_NAMES: dict[str, LogLevel] = {
    "debug": LogLevel.DEBUG,
    "info": LogLevel.INFO,
    "warn": LogLevel.WARN,
    "warning": LogLevel.WARN,
    "error": LogLevel.ERROR,
    "silent": LogLevel.SILENT,
}


@dataclass(frozen=True, slots=True)
class CommandContext:
    client: FumiClient
    terminal: Terminal


CommandHandler: TypeAlias = Callable[[list[str], CommandContext], Awaitable[bool]]


async def cmd_help(args: list[str], ctx: CommandContext) -> bool:
    """Show available interactive commands."""
    help_text = """
Available commands:
  help, ?                   Show this help message
  info, device              Show connected device information & build hash
  level [level_name]        Get or set firmware log level (debug, info, warn, error, silent)
  stats                     Query firmware logger transmission and drop counters
  ping [count]              Ping device and measure round-trip latency (RTT)
  trace <subcommand>        trigger and recording waveforms [not implemented yet]
  exit, quit                Disconnect and exit console
""".strip()
    ctx.terminal.print(help_text)
    return True


async def cmd_info(args: list[str], ctx: CommandContext) -> bool:
    """Show device information."""
    dev = ctx.client.device_info
    if dev is None:
        ctx.terminal.print("No device information available.")
        return True

    info_lines = [
        f"Device Name:        {dev.name}",
        f"Description:        {dev.description}",
        f"Build ID (SHA-1):   {dev.build_id.hex()}",
    ]
    ctx.terminal.print("\n".join(info_lines))
    return True


async def cmd_level(args: list[str], ctx: CommandContext) -> bool:
    """Get or set firmware log filter level."""
    if not args:
        ctx.terminal.print("Usage: level <debug|info|warn|error|silent>")
        return True

    target_name = args[0].lower()
    if target_name not in LEVEL_NAMES:
        ctx.terminal.diagnostic(
            f"unknown level '{args[0]}'; options: debug, info, warn, error, silent"
        )
        return True

    level = LEVEL_NAMES[target_name]
    try:
        await ctx.client.set_log_level(level)
        ctx.terminal.print(f"Log level updated to {level.name}.")
    except DeviceError as exc:
        ctx.terminal.diagnostic(f"device rejected log level change: {exc}")
    except (OSError, ValueError, TimeoutError) as exc:
        ctx.terminal.diagnostic(f"failed to set log level: {exc}")

    return True


async def cmd_stats(args: list[str], ctx: CommandContext) -> bool:
    """Query firmware logger stats."""
    try:
        stats = await ctx.client.get_stats()
        lines = [
            "Firmware Logger Statistics:",
            f"  Messages Sent:       {stats.sent}",
            f"  UART TX Errors:      {stats.uart_tx_error}",
            f"  Encoding Errors:     {stats.encode_error}",
            f"  Resource Unavail:    {stats.resource_unavail_error}",
            f"  Dropped (No Slot):   {stats.drop_no_free_slot}",
            f"  Dropped (In ISR):    {stats.drop_in_isr}",
            f"  Dropped (Unknown):   {stats.drop_unknown}",
        ]
        ctx.terminal.print("\n".join(lines))
    except (DeviceError, OSError, ValueError, TimeoutError) as exc:
        ctx.terminal.diagnostic(f"failed to query stats: {exc}")

    return True


async def cmd_ping(args: list[str], ctx: CommandContext) -> bool:
    """Ping device and measure round-trip latency."""
    count = 1
    if args:
        try:
            count = max(1, min(int(args[0]), 20))
        except ValueError:
            ctx.terminal.diagnostic(f"invalid ping count '{args[0]}'")
            return True

    for i in range(count):
        try:
            reply, rtt_ms = await ctx.client.ping()
            ctx.terminal.print(f"Reply from device: {reply}  time={rtt_ms:.2f}ms")
            if i + 1 < count:
                await asyncio.sleep(0.2)
        except (DeviceError, OSError, ValueError, TimeoutError) as exc:
            ctx.terminal.diagnostic(f"ping failed: {exc}")
            break

    return True


async def cmd_trace(args: list[str], ctx: CommandContext) -> bool:
    """Trace placeholder command."""
    ctx.terminal.print(
        "Trace waveform capture subsystem is reserved in protocol.h;\n"
        "not implemented yet."
    )
    return True


async def cmd_exit(args: list[str], ctx: CommandContext) -> bool:
    """Exit console."""
    return False


COMMANDS: dict[str, CommandHandler] = {
    "help": cmd_help,
    "?": cmd_help,
    "info": cmd_info,
    "device": cmd_info,
    "level": cmd_level,
    "stats": cmd_stats,
    "ping": cmd_ping,
    "trace": cmd_trace,
    "exit": cmd_exit,
    "quit": cmd_exit,
}


async def dispatch_command(line: str, ctx: CommandContext) -> bool:
    """Parse and dispatch a REPL command line. Returns False if console should exit."""
    stripped = line.strip()
    if not stripped:
        return True

    try:
        parts = shlex.split(stripped)
    except ValueError as exc:
        ctx.terminal.diagnostic(f"command parse error: {exc}")
        return True

    cmd_name = parts[0].lower()
    args = parts[1:]

    handler = COMMANDS.get(cmd_name)
    if handler is None:
        ctx.terminal.diagnostic(
            f"unknown command '{cmd_name}'. Type 'help' for available commands."
        )
        return True

    return await handler(args, ctx)
