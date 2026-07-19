#!/usr/bin/env python3
"""Generate and verify the pinned Jazzy rclpy public API parity ledger."""

from __future__ import annotations

import argparse
import contextlib
import enum
import importlib
import importlib.metadata
import inspect
import io
import json
import os
from pathlib import Path
import pkgutil
import re
import subprocess
import sys
import tempfile
from typing import Any


_VOLATILE_ADDRESS = re.compile(r" at 0x[0-9a-fA-F]+")

LEDGER_SCHEMA = "rclcppyy.rclpy-api-ledger/v1"
OBSERVATION_SCHEMA = "rclcppyy.rclpy-api-observation/v1"
ANNOTATION_SCHEMA = "rclcppyy.rclpy-api-annotations/v1"
EXTRACTION_CONTRACT = "rclpy-public-python-surface-v1"
EXPECTED_RCLPY_VERSION = "7.1.11"
EXPECTED_ROS_DISTRO = "jazzy"
EXPECTED_RMW = "rmw_cyclonedds_cpp"
STATUSES = {
    "exact_direct_cpp_authority",
    "mixed_control_direct_data",
    "unchanged_stock_fallback",
    "explicit_fail_closed",
    "missing_mismatch",
    "unassessed",
}
REQUIRED_DUNDERS = {
    "__aenter__",
    "__aexit__",
    "__await__",
    "__bool__",
    "__call__",
    "__contains__",
    "__enter__",
    "__eq__",
    "__exit__",
    "__ge__",
    "__getitem__",
    "__gt__",
    "__hash__",
    "__init__",
    "__iter__",
    "__le__",
    "__len__",
    "__lt__",
    "__ne__",
    "__next__",
    "__repr__",
    "__setitem__",
    "__str__",
}
EXCLUDED_MODULE_PARTS = {"impl"}
DEFAULT_BASELINE = Path("compatibility/rclpy-api-ledger-jazzy.json")
DEFAULT_ANNOTATIONS = Path("compatibility/rclpy-api-annotations-jazzy.json")
DEFAULT_MANIFEST = Path("compatibility/jazzy.json")


class LedgerError(ValueError):
    """The extraction, annotation, or ledger contract is invalid."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise LedgerError(message)


def _json_read(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise LedgerError(f"cannot read {label} {path}: {exc}") from exc
    _require(isinstance(value, dict), f"{label} must be a JSON object")
    return value


def _atomic_json_write(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, indent=2, sort_keys=True) + "\n"
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(path)


def _type_path(value: Any) -> str:
    value_type = type(value)
    return f"{value_type.__module__}.{value_type.__qualname__}"


def _origin(value: Any) -> str | None:
    origin = getattr(value, "__module__", None)
    return origin if isinstance(origin, str) and origin else None


def _qualname(value: Any) -> str | None:
    name = getattr(value, "__qualname__", None)
    return name if isinstance(name, str) and name else None


def _signature(value: Any, *, applicable: bool = True) -> dict[str, str]:
    if not applicable:
        return {"state": "not_applicable"}
    try:
        result = str(inspect.signature(value))
    except (TypeError, ValueError):
        return {
            "state": "uninspectable",
            "reason": "inspect_signature_unsupported",
        }
    result = _VOLATILE_ADDRESS.sub("", result)
    return {"state": "inspectable", "value": result}


def _implementation(value: Any) -> str:
    if hasattr(value, "__cpp_name__"):
        return "cppyy"
    origin = _origin(value) or _type_path(value)
    if any(part.startswith("_") for part in origin.split(".")):
        return "extension"
    if inspect.isbuiltin(value) or inspect.ismethoddescriptor(value):
        return "extension"
    if inspect.isclass(value) or inspect.isroutine(value):
        try:
            source = inspect.getsourcefile(value)
        except TypeError:
            source = None
        return "python" if source else "extension"
    return "data"


def _constant_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, enum.Enum):
        return {
            "enum_type": f"{type(value).__module__}.{type(value).__qualname__}",
            "name": value.name,
            "value": _constant_value(value.value),
        }
    if isinstance(value, (list, tuple)) and len(value) <= 32:
        converted = [_constant_value(item) for item in value]
        if all(item is not _UNREPRESENTABLE for item in converted):
            return converted
    if isinstance(value, dict) and len(value) <= 32:
        converted = {
            str(key): _constant_value(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
        if all(item is not _UNREPRESENTABLE for item in converted.values()):
            return converted
    return _UNREPRESENTABLE


_UNREPRESENTABLE = object()


def _symbol_kind(value: Any) -> str | None:
    if inspect.isclass(value):
        return "class"
    if inspect.isroutine(value):
        return "function"
    if inspect.ismodule(value):
        return None
    return "constant"


def _is_public_symbol(module: Any, name: str, value: Any) -> bool:
    if name.startswith("_"):
        return False
    exported = getattr(module, "__all__", ())
    if exported and name in exported:
        return _symbol_kind(value) is not None
    kind = _symbol_kind(value)
    if kind in {"class", "function"}:
        origin = _origin(value) or ""
        return origin == module.__name__ or origin.startswith("rclpy.")
    if kind == "constant":
        if name.isupper():
            return True
        return type(value).__module__.startswith("rclpy.")
    return False


def _member_owner(class_type: type, name: str) -> type | None:
    try:
        lineage = inspect.getmro(class_type)
    except (AttributeError, TypeError):
        return None
    return next((owner for owner in lineage if name in vars(owner)), None)


def _is_public_member(class_type: type, name: str) -> bool:
    if not name.startswith("_"):
        return True
    if name not in REQUIRED_DUNDERS:
        return False
    owner = _member_owner(class_type, name)
    if owner is None or owner is object:
        return False
    owner_module = getattr(owner, "__module__", "")
    return owner is class_type or owner_module.startswith("rclpy.")


def _member_kind(raw: Any) -> tuple[str, Any, bool]:
    if isinstance(raw, property):
        return "property", raw.fget, raw.fget is not None
    if isinstance(raw, classmethod):
        return "classmethod", raw.__func__, True
    if isinstance(raw, staticmethod):
        return "staticmethod", raw.__func__, True
    if inspect.isclass(raw):
        return "nested_class", raw, True
    if inspect.isroutine(raw) or inspect.ismethoddescriptor(raw):
        return "method", raw, True
    if inspect.isdatadescriptor(raw):
        return "data_descriptor", raw, False
    return "class_attribute", raw, False


def _member_descriptor(class_path: str, class_type: type, name: str) -> dict[str, Any]:
    try:
        raw = inspect.getattr_static(class_type, name)
    except AttributeError:
        return {
            "name": name,
            "path": f"{class_path}.{name}",
            "kind": "uninspectable",
            "owner": None,
            "origin": None,
            "implementation": "extension",
            "signature": {
                "state": "uninspectable",
                "reason": "static_attribute_unavailable",
            },
        }
    kind, target, applicable = _member_kind(raw)
    owner = _member_owner(class_type, name)
    owner_path = None
    if owner is not None:
        owner_path = f"{owner.__module__}.{owner.__qualname__}"
    descriptor = {
        "name": name,
        "path": f"{class_path}.{name}",
        "kind": kind,
        "owner": owner_path,
        "origin": _origin(target),
        "implementation": _implementation(target),
        "signature": _signature(target, applicable=applicable),
    }
    if kind == "class_attribute":
        constant = _constant_value(raw)
        descriptor["value"] = (
            {"state": "captured", "value": constant}
            if constant is not _UNREPRESENTABLE
            else {"state": "type_only", "type": _type_path(raw)}
        )
    return descriptor


def _class_members(class_path: str, class_type: type) -> list[dict[str, Any]]:
    try:
        names = dir(class_type)
    except Exception:
        return []
    return [
        _member_descriptor(class_path, class_type, name)
        for name in sorted(set(names))
        if _is_public_member(class_type, name)
    ]


def _symbol_descriptor(module: Any, name: str, value: Any) -> dict[str, Any]:
    kind = _symbol_kind(value)
    path = f"{module.__name__}.{name}"
    descriptor = {
        "name": name,
        "path": path,
        "kind": kind,
        "origin": _origin(value),
        "qualname": _qualname(value),
        "implementation": _implementation(value),
        "signature": _signature(value, applicable=kind in {"class", "function"}),
    }
    if kind == "class":
        descriptor["members"] = _class_members(path, value)
    elif kind == "constant":
        constant = _constant_value(value)
        descriptor["value"] = (
            {"state": "captured", "value": constant}
            if constant is not _UNREPRESENTABLE
            else {"state": "type_only", "type": _type_path(value)}
        )
    return descriptor


def _public_module_names(rclpy_module: Any) -> list[str]:
    names = ["rclpy"]
    for module_info in pkgutil.walk_packages(
        rclpy_module.__path__, prefix="rclpy."
    ):
        parts = module_info.name.split(".")[1:]
        if any(
            part.startswith("_") or part in EXCLUDED_MODULE_PARTS
            for part in parts
        ):
            continue
        names.append(module_info.name)
    return sorted(set(names), key=lambda name: (name != "rclpy", name))


def _observe_module(name: str) -> dict[str, Any]:
    try:
        module = importlib.import_module(name)
    except BaseException as exc:
        return {
            "name": name,
            "import": {
                "state": "failed",
                "exception": type(exc).__name__,
                "message": str(exc),
            },
            "symbols": [],
        }
    symbols = []
    for symbol_name, value in sorted(vars(module).items()):
        if _is_public_symbol(module, symbol_name, value):
            symbols.append(_symbol_descriptor(module, symbol_name, value))
    return {"name": name, "import": {"state": "ok"}, "symbols": symbols}


def _generated_interfaces(direct_cpp_module: Any) -> list[dict[str, Any]]:
    groups = (
        ("message", getattr(direct_cpp_module, "_MESSAGE_INSTALLATION", None)),
        ("service", getattr(direct_cpp_module, "_SERVICE_INSTALLATION", None)),
        ("action", getattr(direct_cpp_module, "_ACTION_INSTALLATION", None)),
    )
    result = []
    for kind, installation in groups:
        for binding in getattr(installation, "bindings", ()):
            direct_type = getattr(binding, "cpp_type", None)
            if direct_type is None:
                direct_type = getattr(binding, "action_type", None)
            if direct_type is None:
                direct_type = getattr(binding, "service_type", None)
            result.append({
                "interface": binding.interface,
                "kind": kind,
                "cpp_type": (
                    getattr(binding, "cpp_type_name", None)
                    or getattr(getattr(binding, "cpp_types", None), "cpp_name", None)
                    or getattr(direct_type, "__cpp_name__", None)
                ),
                "public_type": (
                    f"{direct_type.__module__}.{direct_type.__qualname__}"
                    if direct_type is not None
                    and hasattr(direct_type, "__module__")
                    and hasattr(direct_type, "__qualname__")
                    else None
                ),
            })
    return sorted(result, key=lambda row: (row["kind"], row["interface"]))


def observe(mode: str) -> dict[str, Any]:
    """Collect one stock or direct observation in the current fresh process."""
    _require(mode in {"stock", "direct"}, "observation mode must be stock or direct")
    version = importlib.metadata.version("rclpy")
    _require(version == EXPECTED_RCLPY_VERSION, f"expected rclpy {EXPECTED_RCLPY_VERSION}, got {version}")
    generated_interfaces = []
    diagnostics = io.StringIO()
    with contextlib.redirect_stdout(diagnostics):
        if mode == "direct":
            import rclcppyy

            rclcppyy.enable_cpp_acceleration(profile="direct_cpp")
        import rclpy

        modules = [_observe_module(name) for name in _public_module_names(rclpy)]
        if mode == "direct":
            direct_cpp_module = importlib.import_module("rclcppyy.direct_cpp")
            generated_interfaces = _generated_interfaces(direct_cpp_module)
    return {
        "schema": OBSERVATION_SCHEMA,
        "mode": mode,
        "runtime": {
            "ros_distribution": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "rclpy_version": version,
        },
        "modules": modules,
        "generated_interfaces": generated_interfaces,
    }


def _run_observation(repo_root: Path, mode: str) -> dict[str, Any]:
    script = repo_root / "scripts" / "generate_rclpy_api_ledger.py"
    with tempfile.TemporaryDirectory(prefix=f"rclcppyy-ledger-{mode}-") as directory:
        output = Path(directory) / "observation.json"
        command = [
            sys.executable,
            str(script),
            "--observe",
            mode,
            "--observation-output",
            str(output),
        ]
        completed = subprocess.run(
            command,
            cwd=repo_root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=180,
            check=False,
        )
        if completed.returncode != 0:
            raise LedgerError(
                f"{mode} observation failed with code {completed.returncode}: "
                f"stdout={completed.stdout!r} stderr={completed.stderr!r}"
            )
        return _json_read(output, f"{mode} observation")


def _flatten(observation: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result = {}
    for module in observation["modules"]:
        for symbol in module["symbols"]:
            value = dict(symbol)
            members = value.pop("members", [])
            value["_level"] = "symbol"
            result[value["path"]] = value
            for member in members:
                member_value = dict(member)
                member_value["_level"] = "member"
                result[member["path"]] = member_value
    return result


def _brief(value: dict[str, Any] | None) -> dict[str, Any] | None:
    if value is None:
        return None
    keys = (
        "kind", "origin", "qualname", "owner", "implementation", "signature", "value",
    )
    return {key: value[key] for key in keys if key in value}


def _signature_comparison(
    stock: dict[str, Any] | None,
    direct: dict[str, Any] | None,
) -> str:
    if stock is None or direct is None:
        return "missing"
    left = stock["signature"]
    right = direct["signature"]
    if left["state"] == right["state"] == "inspectable":
        return "equal" if left["value"] == right["value"] else "different"
    if left["state"] == right["state"] == "not_applicable":
        return "not_applicable"
    return "uninspectable"


def _path_area(path: str) -> str:
    parts = path.split(".")
    module = parts[1] if len(parts) > 1 else "utilities"
    mapping = {
        "action": "action",
        "callback_groups": "executor",
        "client": "service",
        "clock": "time",
        "clock_type": "time",
        "constants": "utility",
        "context": "context",
        "destroyable": "context",
        "duration": "time",
        "event_handler": "qos",
        "exceptions": "utility",
        "executors": "executor",
        "expand_topic_name": "graph",
        "experimental": "executor",
        "guard_condition": "executor",
        "lifecycle": "lifecycle",
        "logging": "logging",
        "logging_service": "logging",
        "node": "node",
        "parameter": "parameter",
        "parameter_client": "parameter",
        "parameter_event_handler": "parameter",
        "parameter_service": "parameter",
        "publisher": "publisher",
        "qos": "qos",
        "qos_event": "qos",
        "qos_overriding_options": "qos",
        "serialization": "serialization",
        "service": "service",
        "service_introspection": "service",
        "signals": "context",
        "subscription": "subscription",
        "subscription_content_filter_options": "subscription",
        "task": "executor",
        "time": "time",
        "time_source": "time",
        "timer": "timer",
        "topic_endpoint_info": "graph",
        "topic_or_service_is_hidden": "graph",
        "type_description_service": "type_support",
        "type_hash": "type_support",
        "type_support": "type_support",
        "utilities": "utility",
        "validate_full_topic_name": "graph",
        "validate_namespace": "graph",
        "validate_node_name": "graph",
        "validate_parameter_name": "parameter",
        "validate_topic_name": "graph",
        "wait_for_message": "subscription",
        "waitable": "executor",
    }
    return mapping.get(module, "utility")


def validate_annotations(
    annotations: dict[str, Any], manifest: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    _require(
        set(annotations) == {"schema", "source_manifest", "entries"},
        "annotation fields differ from v1 contract",
    )
    _require(annotations["schema"] == ANNOTATION_SCHEMA, "unknown annotation schema")
    _require(
        annotations["source_manifest"] == "compatibility/jazzy.json",
        "annotations must seed compatibility/jazzy.json",
    )
    entries = annotations["entries"]
    _require(isinstance(entries, list), "annotation entries must be a list")
    manifest_ids = {entry["id"] for entry in manifest.get("entries", [])}
    result = {}
    for index, entry in enumerate(entries):
        prefix = f"annotation entries[{index}]"
        _require(
            isinstance(entry, dict)
            and set(entry) == {"path", "status", "manifest_entry_ids", "rationale"},
            f"{prefix} fields differ from v1 contract",
        )
        path = entry["path"]
        _require(isinstance(path, str) and path, f"{prefix}.path is required")
        _require(path not in result, f"duplicate annotation path: {path}")
        _require(entry["status"] in STATUSES - {"missing_mismatch"}, f"{path}: invalid annotation status")
        ids = entry["manifest_entry_ids"]
        _require(isinstance(ids, list) and ids, f"{path}: manifest_entry_ids are required")
        _require(len(ids) == len(set(ids)), f"{path}: duplicate manifest_entry_ids")
        _require(set(ids) <= manifest_ids, f"{path}: unknown manifest entry id")
        _require(
            isinstance(entry["rationale"], str) and entry["rationale"],
            f"{path}: rationale is required",
        )
        result[path] = entry
    return result


def _manifest_by_area(manifest: dict[str, Any]) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for entry in manifest["entries"]:
        result.setdefault(entry["area"], []).append(entry["id"])
    return {key: sorted(value) for key, value in result.items()}


def _comparison_state(
    stock: dict[str, Any] | None,
    direct: dict[str, Any] | None,
) -> dict[str, Any]:
    presence = (
        "both" if stock is not None and direct is not None
        else "stock_only" if stock is not None
        else "direct_only"
    )
    return {
        "presence": presence,
        "kind_match": (
            stock is not None
            and direct is not None
            and stock["kind"] == direct["kind"]
        ),
        "signature": _signature_comparison(stock, direct),
    }


def _structural_mismatch(comparison: dict[str, Any]) -> bool:
    return (
        comparison["presence"] == "stock_only"
        or not comparison["kind_match"]
        or comparison["signature"] == "different"
    )


def build_ledger(
    stock: dict[str, Any],
    direct: dict[str, Any],
    annotations: dict[str, Any],
    manifest: dict[str, Any],
) -> dict[str, Any]:
    """Combine fresh observations and explicit authority annotations."""
    stock_modules = [module["name"] for module in stock["modules"]]
    direct_modules = [module["name"] for module in direct["modules"]]
    _require(stock_modules == direct_modules, "stock and direct public module sets differ")
    stock_failed = sorted(
        module["name"] for module in stock["modules"] if module["import"]["state"] != "ok"
    )
    _require(
        not stock_failed,
        f"stock public module import failed: {', '.join(stock_failed)}",
    )
    direct_failed = sorted(
        module["name"] for module in direct["modules"] if module["import"]["state"] != "ok"
    )
    _require(
        not direct_failed,
        f"direct public module import failed: {', '.join(direct_failed)}",
    )
    annotation_by_path = validate_annotations(annotations, manifest)
    manifest_by_area = _manifest_by_area(manifest)
    stock_entries = _flatten(stock)
    direct_entries = _flatten(direct)
    entries = []
    for path in sorted(set(stock_entries) | set(direct_entries)):
        stock_value = stock_entries.get(path)
        direct_value = direct_entries.get(path)
        comparison = _comparison_state(stock_value, direct_value)
        annotation = annotation_by_path.get(path)
        if _structural_mismatch(comparison):
            status = "missing_mismatch"
            rationale = "Mechanical stock/direct presence, kind, or signature mismatch."
            annotation_ids = [] if annotation is None else annotation["manifest_entry_ids"]
        elif annotation is None:
            status = "unassessed"
            rationale = "Name and signature observations do not establish semantic parity."
            annotation_ids = []
        else:
            status = annotation["status"]
            rationale = annotation["rationale"]
            annotation_ids = annotation["manifest_entry_ids"]
        area = _path_area(path)
        level_source = stock_value if stock_value is not None else direct_value
        entries.append({
            "path": path,
            "level": level_source["_level"],
            "area": area,
            "stock": _brief(stock_value),
            "direct": _brief(direct_value),
            "comparison": comparison,
            "status": status,
            "annotation": {
                "manifest_entry_ids": annotation_ids,
                "area_manifest_entry_ids": manifest_by_area.get(area, []),
                "rationale": rationale,
            },
        })
    unknown_annotations = sorted(set(annotation_by_path) - {entry["path"] for entry in entries})
    _require(not unknown_annotations, f"annotations reference unknown paths: {', '.join(unknown_annotations)}")
    status_counts = dict.fromkeys(sorted(STATUSES), 0)
    for entry in entries:
        status_counts[entry["status"]] += 1
    stock_symbol_count = sum(len(module["symbols"]) for module in stock["modules"])
    stock_member_count = sum(
        len(symbol.get("members", []))
        for module in stock["modules"]
        for symbol in module["symbols"]
    )
    return {
        "schema": LEDGER_SCHEMA,
        "source": {
            "ros_distribution": EXPECTED_ROS_DISTRO,
            "rmw_implementation": EXPECTED_RMW,
            "rclpy_package_version": EXPECTED_RCLPY_VERSION,
            "extraction_contract": EXTRACTION_CONTRACT,
            "annotation_manifest": annotations["source_manifest"],
        },
        "contract": {
            "module_rule": (
                "rclpy plus recursively discoverable modules excluding private path "
                "components and rclpy.impl"
            ),
            "symbol_rule": (
                "public rclpy-origin classes/functions, explicit __all__ exports, "
                "uppercase constants, and rclpy-typed public constants"
            ),
            "member_rule": (
                "public class members plus explicitly declared rclpy-owned required "
                "dunders; inherited object dunders are excluded"
            ),
            "signature_rule": (
                "inspect.signature text or explicit uninspectable/not_applicable state"
            ),
            "semantic_rule": (
                "presence and matching signatures never imply semantic parity; only "
                "explicit annotations assign non-unassessed authority"
            ),
            "required_dunders": sorted(REQUIRED_DUNDERS),
        },
        "modules": [
            {
                "name": stock_module["name"],
                "stock_import": stock_module["import"],
                "direct_import": direct_module["import"],
            }
            for stock_module, direct_module in zip(stock["modules"], direct["modules"])
        ],
        "entries": entries,
        "generated_interface_aliases": direct["generated_interfaces"],
        "summary": {
            "modules": len(stock_modules),
            "stock_public_symbols": stock_symbol_count,
            "stock_public_class_members": stock_member_count,
            "ledger_entries": len(entries),
            "signature_mismatches": sum(
                entry["comparison"]["signature"] == "different"
                for entry in entries
            ),
            "stock_only_entries": sum(
                entry["comparison"]["presence"] == "stock_only"
                for entry in entries
            ),
            "direct_only_entries": sum(
                entry["comparison"]["presence"] == "direct_only"
                for entry in entries
            ),
            "uninspectable_signature_entries": sum(
                entry["comparison"]["signature"] == "uninspectable"
                for entry in entries
            ),
            "statuses": status_counts,
            "generated_interface_aliases": len(direct["generated_interfaces"]),
        },
    }


def validate_ledger(document: dict[str, Any]) -> dict[str, Any]:
    """Validate strict top-level shape, ordering, uniqueness, and accounting."""
    expected = {
        "schema", "source", "contract", "modules", "entries",
        "generated_interface_aliases", "summary",
    }
    _require(set(document) == expected, "ledger fields differ from v1 contract")
    _require(document["schema"] == LEDGER_SCHEMA, "unknown ledger schema")
    source = document["source"]
    _require(source["rclpy_package_version"] == EXPECTED_RCLPY_VERSION, "wrong rclpy version")
    _require(source["ros_distribution"] == EXPECTED_ROS_DISTRO, "wrong ROS distribution")
    modules = document["modules"]
    entries = document["entries"]
    _require(isinstance(modules, list) and modules, "ledger modules must be non-empty")
    _require(isinstance(entries, list) and entries, "ledger entries must be non-empty")
    module_names = [module["name"] for module in modules]
    _require(len(module_names) == len(set(module_names)), "duplicate ledger module")
    paths = [entry["path"] for entry in entries]
    _require(paths == sorted(paths), "ledger entries are not sorted")
    _require(len(paths) == len(set(paths)), "duplicate ledger entry path")
    _require(all(entry["status"] in STATUSES for entry in entries), "invalid ledger status")
    actual_statuses = dict.fromkeys(sorted(STATUSES), 0)
    for entry in entries:
        actual_statuses[entry["status"]] += 1
        if _structural_mismatch(entry["comparison"]):
            _require(
                entry["status"] == "missing_mismatch",
                f"{entry['path']}: structural mismatch is not fail-visible",
            )
    summary = document["summary"]
    _require(summary["modules"] == len(modules), "module summary drift")
    _require(summary["ledger_entries"] == len(entries), "entry summary drift")
    _require(summary["statuses"] == actual_statuses, "status summary drift")
    _require(
        summary["signature_mismatches"]
        == sum(entry["comparison"]["signature"] == "different" for entry in entries),
        "signature mismatch summary drift",
    )
    return summary


def generate(repo_root: Path, annotations_path: Path, manifest_path: Path) -> dict[str, Any]:
    annotations = _json_read(annotations_path, "API annotations")
    manifest = _json_read(manifest_path, "compatibility manifest")
    stock = _run_observation(repo_root, "stock")
    direct = _run_observation(repo_root, "direct")
    document = build_ledger(stock, direct, annotations, manifest)
    validate_ledger(document)
    return document


def _resolve(repo_root: Path, value: Path) -> Path:
    return value if value.is_absolute() else repo_root / value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observe", choices=("stock", "direct"), help=argparse.SUPPRESS)
    parser.add_argument("--observation-output", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--generate", action="store_true", help="write a fresh baseline")
    parser.add_argument("--verify", action="store_true", help="verify baseline against fresh observations")
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--annotations", type=Path, default=DEFAULT_ANNOTATIONS)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--json", action="store_true", help="print summary as JSON")
    args = parser.parse_args(argv)
    repo_root = Path(__file__).resolve().parent.parent
    try:
        if args.observe:
            _require(args.observation_output is not None, "observation output is required")
            _atomic_json_write(args.observation_output, observe(args.observe))
            return 0
        _require(args.generate != args.verify, "select exactly one of --generate or --verify")
        baseline_path = _resolve(repo_root, args.baseline)
        annotations_path = _resolve(repo_root, args.annotations)
        manifest_path = _resolve(repo_root, args.manifest)
        candidate = generate(repo_root, annotations_path, manifest_path)
        if args.generate:
            _atomic_json_write(baseline_path, candidate)
        else:
            baseline = _json_read(baseline_path, "API ledger baseline")
            validate_ledger(baseline)
            _require(
                baseline == candidate,
                "installed rclpy/direct public API differs from the committed ledger; "
                "regenerate and review the semantic annotations",
            )
        summary = candidate["summary"]
    except (LedgerError, subprocess.TimeoutExpired) as exc:
        print(f"rclpy API ledger error: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(
            f"rclpy API ledger: modules={summary['modules']} "
            f"symbols={summary['stock_public_symbols']} "
            f"members={summary['stock_public_class_members']} "
            f"signature_mismatches={summary['signature_mismatches']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
