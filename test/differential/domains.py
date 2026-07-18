"""Cross-process ROS domain leases for integration tests."""

import fcntl
import os
import tempfile
import time
from pathlib import Path


DEFAULT_DOMAIN_MIN = 100
DEFAULT_DOMAIN_MAX = 230


class DomainLease:
    """Own an advisory lock for one ROS domain until explicitly released."""

    def __init__(self, domain_id, lock_file, handle):
        self.domain_id = domain_id
        self.lock_file = lock_file
        self._handle = handle

    def environment(self, base=None):
        env = dict(os.environ if base is None else base)
        env["ROS_DOMAIN_ID"] = str(self.domain_id)
        return env

    def release(self):
        if self._handle is None:
            return
        fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        self._handle.close()
        self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()


def _configured_range():
    minimum = int(os.environ.get("RCLCPPYY_TEST_DOMAIN_MIN", DEFAULT_DOMAIN_MIN))
    maximum = int(os.environ.get("RCLCPPYY_TEST_DOMAIN_MAX", DEFAULT_DOMAIN_MAX))
    if not 0 <= minimum <= maximum <= 232:
        raise ValueError("ROS test domain range must satisfy 0 <= min <= max <= 232")
    return minimum, maximum


def acquire_domain(lock_root=None):
    """Acquire a unique domain lease without relying on pytest worker identity."""
    minimum, maximum = _configured_range()
    root = Path(lock_root or Path(tempfile.gettempdir()) / "rclcppyy-test-domains")
    root.mkdir(parents=True, exist_ok=True)
    count = maximum - minimum + 1
    start = minimum + ((os.getpid() + time.monotonic_ns()) % count)

    for offset in range(count):
        domain_id = minimum + ((start - minimum + offset) % count)
        lock_file = root / ("domain-%03d.lock" % domain_id)
        handle = lock_file.open("a+", encoding="ascii")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.close()
            continue
        handle.seek(0)
        handle.truncate()
        handle.write("pid=%d acquired_ns=%d\n" % (os.getpid(), time.time_ns()))
        handle.flush()
        return DomainLease(domain_id, lock_file, handle)
    raise RuntimeError("no free ROS_DOMAIN_ID in configured test range")
