"""Mirror a CLI's stdout/stderr into {scriptname}.log, truncated per run.

Every top-level script calls tee_output(__file__) first thing in main(), so
the complete run transcript is always available in a predictable file without
remembering to pipe. Read results from the log instead of scrollback.
"""

from __future__ import annotations

import sys
from pathlib import Path


class _Tee:
    def __init__(self, stream, logfile) -> None:
        self.stream = stream
        self.logfile = logfile

    def write(self, text: str) -> int:
        self.stream.write(text)
        self.logfile.write(text)
        return len(text)

    def flush(self) -> None:
        self.stream.flush()
        self.logfile.flush()

    def isatty(self) -> bool:
        return self.stream.isatty()

    def fileno(self) -> int:
        return self.stream.fileno()


def tee_output(script_path: str) -> Path:
    log_path = Path(script_path).resolve().with_suffix(".log")
    logfile = log_path.open("w", encoding="utf-8", buffering=1)
    sys.stdout = _Tee(sys.stdout, logfile)
    sys.stderr = _Tee(sys.stderr, logfile)
    return log_path
