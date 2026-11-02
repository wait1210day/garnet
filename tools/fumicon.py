#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Start the fumi host console; implementation lives in tools/fumicon/."""

import sys

from fumicon.cli import main

if __name__ == "__main__":
    sys.exit(main())
