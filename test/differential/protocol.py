"""Strict machine-readable protocol for differential child processes."""

import json


RESULT_PREFIX = "RCLCPPYY_DIFFERENTIAL_RESULT="
RESULT_SCHEMA = "rclcppyy.differential/v1"
RESULT_KEYS = {
    "schema",
    "scenario",
    "mode",
    "outcome",
    "observations",
    "backend_expectations",
    "backend_status",
    "backend_verified",
    "cleanup",
    "error",
}
EXPECTATION_KEYS = {"kind", "backend", "minimum", "metadata"}
ERROR_KEYS = {"type", "message", "traceback"}
RECORD_KINDS = {"nodes", "entities", "operations"}
BACKENDS = {"cpp", "python", "unsupported"}
MODES = {"stock", "activated"}
OUTCOMES = {"pass", "error"}


class ProtocolError(ValueError):
    """Raised when a child result does not satisfy the protocol."""


def _require_exact_keys(value, keys, location):
    actual = set(value)
    if actual != keys:
        missing = sorted(keys - actual)
        extra = sorted(actual - keys)
        raise ProtocolError(
            "%s keys differ: missing=%r extra=%r" % (location, missing, extra))


def validate_expectation(expectation):
    if not isinstance(expectation, dict):
        raise ProtocolError("backend expectation must be an object")
    _require_exact_keys(expectation, EXPECTATION_KEYS, "backend expectation")
    if expectation["kind"] not in RECORD_KINDS:
        raise ProtocolError("invalid backend expectation kind")
    if expectation["backend"] not in BACKENDS:
        raise ProtocolError("invalid expected backend")
    if not isinstance(expectation["minimum"], int) or expectation["minimum"] < 1:
        raise ProtocolError("backend expectation minimum must be a positive integer")
    if not isinstance(expectation["metadata"], dict):
        raise ProtocolError("backend expectation metadata must be an object")


def validate_backend_status(status):
    if not isinstance(status, dict):
        raise ProtocolError("backend status must be an object")
    if status.get("schema_version") != 1:
        raise ProtocolError("unsupported backend status schema")
    for kind in RECORD_KINDS:
        if not isinstance(status.get(kind), list):
            raise ProtocolError("backend status %s must be an array" % kind)


def verify_backend_expectations(status, expectations):
    """Return unmet expectation descriptions for a ``rclcppyy.status()`` value."""
    validate_backend_status(status)
    unmet = []
    for expectation in expectations:
        validate_expectation(expectation)
        matches = []
        for record in status[expectation["kind"]]:
            if record.get("backend") != expectation["backend"]:
                continue
            metadata = record.get("metadata", {})
            if all(metadata.get(key) == value
                   for key, value in expectation["metadata"].items()):
                matches.append(record)
        if len(matches) < expectation["minimum"]:
            unmet.append({
                "expectation": expectation,
                "observed": len(matches),
            })
    return unmet


def validate_result(result):
    if not isinstance(result, dict):
        raise ProtocolError("child result must be an object")
    _require_exact_keys(result, RESULT_KEYS, "child result")
    if result["schema"] != RESULT_SCHEMA:
        raise ProtocolError("unsupported child result schema")
    if not isinstance(result["scenario"], str) or not result["scenario"]:
        raise ProtocolError("scenario must be a non-empty string")
    if result["mode"] not in MODES:
        raise ProtocolError("invalid child mode")
    if result["outcome"] not in OUTCOMES:
        raise ProtocolError("invalid child outcome")
    if not isinstance(result["observations"], dict):
        raise ProtocolError("observations must be an object")
    if not isinstance(result["backend_expectations"], list):
        raise ProtocolError("backend_expectations must be an array")
    for expectation in result["backend_expectations"]:
        validate_expectation(expectation)
    if result["backend_status"] is not None:
        validate_backend_status(result["backend_status"])
    if not isinstance(result["backend_verified"], bool):
        raise ProtocolError("backend_verified must be boolean")
    if not isinstance(result["cleanup"], dict):
        raise ProtocolError("cleanup must be an object")
    error = result["error"]
    if error is not None:
        if not isinstance(error, dict):
            raise ProtocolError("error must be null or an object")
        _require_exact_keys(error, ERROR_KEYS, "error")
        if not all(isinstance(error[key], str) for key in ERROR_KEYS):
            raise ProtocolError("error values must be strings")
    if result["outcome"] == "pass" and error is not None:
        raise ProtocolError("passing result cannot contain an error")
    if result["outcome"] == "error" and error is None:
        raise ProtocolError("error result must contain error details")
    if result["mode"] == "stock":
        if result["backend_expectations"]:
            raise ProtocolError("stock result cannot claim backend expectations")
        if result["backend_status"] is not None:
            raise ProtocolError("stock result must not import backend status evidence")
        if not result["backend_verified"]:
            raise ProtocolError("stock result must verify its unmodified process mode")
    elif result["outcome"] == "pass":
        if not result["backend_expectations"]:
            raise ProtocolError("activated result requires backend expectations")
        if result["backend_status"] is None:
            raise ProtocolError("activated result requires backend status evidence")
        verified = not verify_backend_expectations(
            result["backend_status"], result["backend_expectations"])
        if result["backend_verified"] != verified:
            raise ProtocolError("backend_verified disagrees with backend status evidence")
    # Ensure artifacts can always be serialized with strict JSON settings.
    try:
        json.dumps(result, allow_nan=False, sort_keys=True)
    except (TypeError, ValueError) as exc:
        raise ProtocolError("child result is not strict JSON: %s" % exc) from exc
    return result


def encode_result(result):
    validate_result(result)
    return RESULT_PREFIX + json.dumps(
        result, allow_nan=False, sort_keys=True, separators=(",", ":"))


def parse_result_lines(stdout):
    markers = [
        line[len(RESULT_PREFIX):]
        for line in stdout.splitlines()
        if line.startswith(RESULT_PREFIX)
    ]
    if len(markers) != 1:
        raise ProtocolError(
            "expected exactly one child result marker, found %d" % len(markers))
    try:
        result = json.loads(markers[0])
    except json.JSONDecodeError as exc:
        raise ProtocolError("child result is not valid JSON: %s" % exc) from exc
    return validate_result(result)
