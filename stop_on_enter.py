"""Shared helper: pressing Enter stops the running motor script immediately."""

import sys
import threading
import time

_stop = None


def install():
    """Start watching stdin for Enter. Returns the stop Event."""
    global _stop
    if _stop is not None:
        return _stop
    return rearm(announce=True)


def rearm(announce=False):
    """Clear stop and start a fresh Enter watcher (for multi-trial scripts)."""
    global _stop
    _stop = threading.Event()

    def _watch():
        try:
            sys.stdin.readline()
        except Exception:
            return
        _stop.set()

    threading.Thread(target=_watch, daemon=True).start()
    if announce:
        print("(Press Enter at any time to stop)", flush=True)
    return _stop


def stopped():
    return _stop is not None and _stop.is_set()


def sleep(seconds):
    """Sleep up to ``seconds``, returning early if Enter was pressed.

    Returns True if interrupted by Enter, False if the full time elapsed.
    """
    if _stop is None:
        time.sleep(seconds)
        return False

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if _stop.is_set():
            return True
        time.sleep(0.05)
    return False
