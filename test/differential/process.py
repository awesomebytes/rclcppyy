"""Owned subprocess groups with bounded teardown and JSON diagnostics."""

import json
import os
import signal
import subprocess
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from differential.protocol import ProtocolError, parse_result_lines


@dataclass
class OwnedProcessResult:
    command: list
    returncode: int
    duration_s: float
    timed_out: bool
    termination_signals: list
    stdout: str
    stderr: str
    protocol_result: dict = None
    protocol_error: str = None

    def artifact(self):
        return {
            "schema": "rclcppyy.subprocess-artifact/v1",
            **asdict(self),
        }

    def diagnostics(self):
        return json.dumps(self.artifact(), indent=2, sort_keys=True)


def _signal_group(process, signum, sent):
    try:
        os.killpg(process.pid, signum)
        sent.append(signal.Signals(signum).name)
    except ProcessLookupError:
        pass


def _write_artifact(path, result):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp-%d" % os.getpid())
    temporary.write_text(
        json.dumps(result.artifact(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(target)


def run_owned_process(command, *, env=None, cwd=None, timeout_s=120,
                      terminate_grace_s=2, require_protocol=True,
                      artifact_path=None):
    """Run a new process group and tear down the whole group on timeout."""
    started = time.monotonic()
    process = subprocess.Popen(
        [str(item) for item in command],
        cwd=cwd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    timed_out = False
    sent = []
    try:
        stdout, stderr = process.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        _signal_group(process, signal.SIGTERM, sent)
        try:
            stdout, stderr = process.communicate(timeout=terminate_grace_s)
        except subprocess.TimeoutExpired:
            _signal_group(process, signal.SIGKILL, sent)
            stdout, stderr = process.communicate()
        else:
            # The group can outlive its leader. A final group signal guarantees
            # descendants do not leak after the timed-out command returns.
            _signal_group(process, signal.SIGKILL, sent)

    protocol_result = None
    protocol_error = None
    if require_protocol:
        try:
            protocol_result = parse_result_lines(stdout)
        except ProtocolError as exc:
            protocol_error = str(exc)

    result = OwnedProcessResult(
        command=[str(item) for item in command],
        returncode=process.returncode,
        duration_s=time.monotonic() - started,
        timed_out=timed_out,
        termination_signals=sent,
        stdout=stdout,
        stderr=stderr,
        protocol_result=protocol_result,
        protocol_error=protocol_error,
    )
    if artifact_path is not None:
        _write_artifact(artifact_path, result)
    return result
