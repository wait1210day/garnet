# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later

"""Interactive REPL with live logs in the terminal's normal scrollback."""

from __future__ import annotations

from fumicon.core.client import FumiClient

from .commands import COMMANDS, CommandContext, dispatch_command
from .terminal import Terminal


class ConsoleUnavailableError(ImportError):
    """Raised when prompt-toolkit is not installed."""


async def run_interactive_console(client: FumiClient, terminal: Terminal) -> int:
    """Keep the prompt immediately below output without interrupting user typing."""
    try:
        from prompt_toolkit import PromptSession
        from prompt_toolkit.completion import WordCompleter
        from prompt_toolkit.patch_stdout import patch_stdout
    except ImportError as exc:
        raise ConsoleUnavailableError(
            "interactive console requires prompt-toolkit; install with: pip install prompt-toolkit"
        ) from exc

    session: PromptSession[str] = PromptSession(
        completer=WordCompleter(list(COMMANDS), ignore_case=True),
        # Don't leave blank rows below the prompt for a potential completion menu.
        reserve_space_for_menu=0,
    )
    ctx = CommandContext(client=client, terminal=terminal)

    client.add_log_listener(terminal.print_log)
    client.add_panic_listener(terminal.print_panic)
    client.add_diagnostic_listener(terminal.diagnostic)
    try:
        terminal.print("Type 'help' for available commands, 'exit' or Ctrl-D to quit.")
        # Move incoming output above the current prompt, preserving ANSI colors
        # and the partially typed command. Scrolling belongs to the terminal.
        with patch_stdout(raw=True):
            while True:
                try:
                    line = await session.prompt_async("fumi> ")
                except (EOFError, SystemExit):
                    break
                except KeyboardInterrupt:
                    continue
                if not await dispatch_command(line, ctx):
                    break
    finally:
        client.remove_log_listener(terminal.print_log)
        client.remove_panic_listener(terminal.print_panic)
        client.remove_diagnostic_listener(terminal.diagnostic)

    return 0
