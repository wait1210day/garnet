# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Terminal presentation, ANSI color rendering and formatting for Fumicon."""

from __future__ import annotations

import sys

from fumicon.core.models import (
    LogLevel,
    LogRecord,
    PanicEnded,
    PanicEvent,
    PanicStarted,
    PanicText,
)

LEVEL_LETTERS: dict[LogLevel, str] = {
    LogLevel.DEBUG: "D",
    LogLevel.INFO: "I",
    LogLevel.WARN: "W",
    LogLevel.ERROR: "E",
    LogLevel.SILENT: "S",
}

LEVEL_COLORS: dict[LogLevel, str] = {
    LogLevel.DEBUG: "\033[34m",  # Blue
    LogLevel.INFO: "\033[32m",   # Green
    LogLevel.WARN: "\033[33m",   # Yellow
    LogLevel.ERROR: "\033[31m",  # Red
    LogLevel.SILENT: "\033[90m", # Gray
}

RESET_COLOR = "\033[0m"
BOLD = "\033[1m"
RED_BOLD = "\033[1;31m"


class Terminal:
    """Manages stdout/stderr printing with ANSI coloring."""

    def __init__(self, color: bool = True) -> None:
        self.color = color

    def print(self, message: str) -> None:
        """Print normal output to stdout."""
        print(message, flush=True)

    def diagnostic(self, message: str) -> None:
        """Print diagnostic or error message to stderr."""
        prefix = f"{RED_BOLD}fumicon:{RESET_COLOR} " if self.color else "fumicon: "
        print(f"{prefix}{message}", file=sys.stderr, flush=True)

    def format_log(self, record: LogRecord) -> str:
        """Format a single LogRecord into colored terminal text."""
        letter = LEVEL_LETTERS.get(record.level, "?")
        color_code = LEVEL_COLORS.get(record.level, "") if self.color else ""
        reset_code = RESET_COLOR if self.color else ""

        tag_name = record.tag_text or f"0x{record.tag_hash:08x}"
        prefix = f"{letter} ({record.timestamp_ticks}) {tag_name}: "

        body = record.formatted_message
        if body is None:
            fmt_str = record.format_text or f"0x{record.format_hash:08x}"
            body = f"<unformatted: {fmt_str}>"

        body = body.rstrip("\r\n")
        if self.color and "\n" in body:
            body = body.replace("\n", f"{reset_code}\n{color_code}")

        return f"{color_code}{prefix}{body}{reset_code}"

    def print_log(self, record: LogRecord) -> None:
        """Render and print a log record."""
        print(self.format_log(record), flush=True)

    def print_panic(self, event: PanicEvent) -> None:
        """Render and print emergency panic stream."""
        if isinstance(event, PanicStarted):
            if self.color:
                sys.stdout.write(f"\n{RED_BOLD}--- EMERGENCY DEVICE PANIC ---{RESET_COLOR}\n")
            else:
                sys.stdout.write("\n--- EMERGENCY DEVICE PANIC ---\n")
            sys.stdout.flush()
        elif isinstance(event, PanicText):
            text = f"{RED_BOLD}{event.text}{RESET_COLOR}" if self.color else event.text
            sys.stdout.write(text)
            sys.stdout.flush()
        elif isinstance(event, PanicEnded):
            if event.error:
                self.diagnostic(f"panic ended with warning: {event.error}")
            sys.stdout.write("\n")
            sys.stdout.flush()
