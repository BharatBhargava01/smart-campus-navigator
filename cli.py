"""Command-line interface wrapper for SAU Smart Campus Navigator.

Delegates to `navigator.cli.main`.
For direct module usage, you can also run:
    python -m navigator <command>
"""
from __future__ import annotations

import sys
from navigator.cli import main, parse_stop

if __name__ == "__main__":
    sys.exit(main())
