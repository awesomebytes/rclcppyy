"""Cross-process ROS domain leases for benchmark matrices."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import tempfile
import time


DEFAULT_MIN = 100
DEFAULT_MAX = 230


class DomainLease:
    def __init__(self, domain_id: int, handle):
        self.domain_id = domain_id
        self._handle = handle

    def release(self) -> None:
        if self._handle is None:
            return
        fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        self._handle.close()
        self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()


def acquire_domain(lock_root: Path | None = None) -> DomainLease:
    minimum = int(os.environ.get("RCLCPPYY_BENCH_DOMAIN_MIN", DEFAULT_MIN))
    maximum = int(os.environ.get("RCLCPPYY_BENCH_DOMAIN_MAX", DEFAULT_MAX))
    if not 0 <= minimum <= maximum <= 232:
        raise ValueError("benchmark domain range must satisfy 0 <= min <= max <= 232")
    root = Path(
        lock_root or Path(tempfile.gettempdir()) / "rclcppyy-benchmark-domains")
    root.mkdir(parents=True, exist_ok=True)
    count = maximum - minimum + 1
    start = minimum + ((os.getpid() + time.monotonic_ns()) % count)
    for offset in range(count):
        domain_id = minimum + ((start - minimum + offset) % count)
        handle = (root / ("domain-%03d.lock" % domain_id)).open("a+", encoding="ascii")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.close()
            continue
        handle.seek(0)
        handle.truncate()
        handle.write("pid=%d acquired_ns=%d\n" % (os.getpid(), time.time_ns()))
        handle.flush()
        return DomainLease(domain_id, handle)
    raise RuntimeError("no free ROS_DOMAIN_ID in configured benchmark range")


__all__ = ["DomainLease", "acquire_domain"]
