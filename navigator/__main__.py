"""Entry point for running the navigator as a module: `python -m navigator`."""
from __future__ import annotations

import sys
from .cli import main

if __name__ == "__main__":
    sys.exit(main())
