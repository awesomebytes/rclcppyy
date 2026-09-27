#!/usr/bin/env python3
"""Scan a ROS 2 Python project into deterministic optimization inputs."""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import sys
from typing import Any


SCHEMA = "rclcppyy.project-scan/v2"
SCANNER_ONLY_MARKER = "SCANNER_ONLY.md"
EXCLUDED_DIRS = {
    ".git", ".pixi", ".pytest_cache", "__pycache__", "build", "install",
    "log", "output", "site-packages", "dist",
}
ENTITY_METHODS = {
    "create_publisher", "create_subscription", "create_timer", "create_service",
    "create_client", "create_guard_condition", "create_rate",
}
EXECUTOR_NAMES = {"SingleThreadedExecutor", "MultiThreadedExecutor", "Executor"}
CALLBACK_GROUP_NAMES = {"MutuallyExclusiveCallbackGroup", "ReentrantCallbackGroup"}
QOS_NAMES = {
    "QoSProfile", "QoSReliabilityPolicy", "QoSHistoryPolicy",
    "QoSDurabilityPolicy", "qos_profile_sensor_data", "qos_profile_services_default",
}
KNOWN_LARGE_MESSAGES = {"Image", "PointCloud2", "LaserScan", "OccupancyGrid"}
KNOWN_NATIVE_LIBRARIES = {
    "cv2": "opencv", "numpy": "array-kernel", "pcl": "pcl", "open3d": "point-cloud",
    "moveit": "moveit", "tf2_ros": "tf2", "rosbag2_py": "rosbag2",
}


def _name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        prefix = _name(node.value)
        return "%s.%s" % (prefix, node.attr) if prefix else node.attr
    return ""


def _literal(node: ast.AST) -> Any:
    try:
        return ast.literal_eval(node)
    except (ValueError, TypeError):
        return None


def _describe_expression(node: ast.AST) -> dict[str, Any]:
    result = {"expression": ast.unparse(node)}
    value = _literal(node)
    if value is not None or isinstance(node, ast.Constant):
        try:
            json.dumps(value, allow_nan=False)
        except (TypeError, ValueError):
            pass
        else:
            result["value"] = value
    return result


def _keyword(node: ast.Call, *names: str) -> ast.AST | None:
    for keyword in node.keywords:
        if keyword.arg in names:
            return keyword.value
    return None


def _call_inputs(node: ast.Call, positional_names: tuple[str, ...] = ()) -> dict[str, Any]:
    inputs = {}
    for index, value in enumerate(node.args):
        name = positional_names[index] if index < len(positional_names) else f"arg_{index}"
        inputs[name] = _describe_expression(value)
    for keyword in node.keywords:
        name = keyword.arg or "kwargs"
        inputs[name] = _describe_expression(keyword.value)
    return inputs


def _record(path: str, node: ast.AST, **values: Any) -> dict[str, Any]:
    return {"file": path, "line": node.lineno, **values}


def _is_super_init(node: ast.Call) -> bool:
    function = node.func
    return (
        isinstance(function, ast.Attribute) and function.attr == "__init__" and
        isinstance(function.value, ast.Call) and
        isinstance(function.value.func, ast.Name) and function.value.func.id == "super"
    )


class ProjectVisitor(ast.NodeVisitor):
    def __init__(self, path: str, tree: ast.AST):
        self.path = path
        self.imports: list[dict[str, Any]] = []
        self.nodes: list[dict[str, Any]] = []
        self.entities: list[dict[str, Any]] = []
        self.executors: list[dict[str, Any]] = []
        self.callback_groups: list[dict[str, Any]] = []
        self.message_types: list[dict[str, Any]] = []
        self.qos: list[dict[str, Any]] = []
        self.callbacks: list[dict[str, Any]] = []
        self._class_stack: list[str] = []
        self._function_stack: list[str] = []
        self._functions: dict[str, ast.FunctionDef | ast.AsyncFunctionDef] = {}
        self._native_bindings: dict[str, str] = {}
        self._message_bindings: dict[str, str] = {}
        self._node_classes: set[str] = set()
        self._node_records: dict[str, dict[str, Any]] = {}
        self._preindex(tree)

    def _preindex(self, tree: ast.AST) -> None:
        class_nodes = [item for item in ast.walk(tree) if isinstance(item, ast.ClassDef)]
        for item in ast.iter_child_nodes(tree):
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                self._functions[item.name] = item
        for class_node in class_nodes:
            for item in class_node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    self._functions[f"{class_node.name}.{item.name}"] = item

        changed = True
        while changed:
            changed = False
            for class_node in class_nodes:
                bases = {_name(base).rsplit(".", 1)[-1] for base in class_node.bases}
                if class_node.name not in self._node_classes and (
                        "Node" in bases or bool(bases & self._node_classes)):
                    self._node_classes.add(class_node.name)
                    changed = True

        for item in ast.walk(tree):
            if isinstance(item, ast.Import):
                for alias in item.names:
                    root = alias.name.split(".", 1)[0]
                    binding = alias.asname or root
                    if root in KNOWN_NATIVE_LIBRARIES:
                        self._native_bindings[binding] = root
            elif isinstance(item, ast.ImportFrom):
                module = item.module or ""
                root = module.split(".", 1)[0]
                for alias in item.names:
                    binding = alias.asname or alias.name
                    if root in KNOWN_NATIVE_LIBRARIES:
                        self._native_bindings[binding] = root
                    if any(part in module.split(".") for part in ("msg", "srv", "action")):
                        self._message_bindings[binding] = alias.name

    def _resolve_function(self, callback_name: str):
        short = callback_name.rsplit(".", 1)[-1]
        if self._class_stack:
            function = self._functions.get(f"{self._class_stack[-1]}.{short}")
            if function is not None:
                return function
        return self._functions.get(short)

    def _native_uses(self, function: ast.AST) -> list[dict[str, str]]:
        used_bindings = {
            item.id for item in ast.walk(function) if isinstance(item, ast.Name)
        }
        return [
            {
                "binding": binding,
                "import": self._native_bindings[binding],
                "capability": KNOWN_NATIVE_LIBRARIES[self._native_bindings[binding]],
            }
            for binding in sorted(used_bindings & self._native_bindings.keys())
        ]

    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            values = {"module": alias.name}
            if alias.asname:
                values["binding"] = alias.asname
            self.imports.append(_record(self.path, node, **values))
        self.generic_visit(node)

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        module = node.module or ""
        for alias in node.names:
            values = {"module": module, "symbol": alias.name}
            if alias.asname:
                values["binding"] = alias.asname
            self.imports.append(_record(self.path, node, **values))
            if any(part in module.split(".") for part in ("msg", "srv", "action")):
                self.message_types.append(
                    _record(self.path, node, module=module, type=alias.name,
                            binding=alias.asname or alias.name))
        self.generic_visit(node)

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        if node.name in self._node_classes:
            record = _record(
                self.path,
                node,
                kind="subclass",
                class_name=node.name,
                bases=[ast.unparse(base) for base in node.bases],
                name=None,
            )
            self.nodes.append(record)
            self._node_records[node.name] = record
        self._class_stack.append(node.name)
        self.generic_visit(node)
        self._class_stack.pop()

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self._function_stack.append(node.name)
        self.generic_visit(node)
        self._function_stack.pop()

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._function_stack.append(node.name)
        self.generic_visit(node)
        self._function_stack.pop()

    def visit_Call(self, node: ast.Call) -> None:
        called = _name(node.func)
        short = called.rsplit(".", 1)[-1]
        if short in ("create_node", "Node"):
            name = _literal(node.args[0]) if node.args else None
            self.nodes.append(_record(self.path, node, kind="call", call=called, name=name))
        if (_is_super_init(node) and self._class_stack and self._function_stack and
                self._function_stack[-1] == "__init__"):
            record = self._node_records.get(self._class_stack[-1])
            if record is not None and node.args:
                record["name"] = _literal(node.args[0])

        if short in ENTITY_METHODS:
            entity: dict[str, Any] = {"method": short}
            if short in ("create_publisher", "create_subscription"):
                if node.args:
                    message_type = _name(node.args[0])
                    entity["message_type"] = message_type
                    entity["message_type_resolved"] = self._message_bindings.get(
                        message_type.rsplit(".", 1)[-1], message_type.rsplit(".", 1)[-1])
                if len(node.args) > 1:
                    entity["topic"] = _literal(node.args[1])
            elif short in ("create_service", "create_client"):
                if node.args:
                    entity["service_type"] = _name(node.args[0])
                if len(node.args) > 1:
                    entity["service"] = _literal(node.args[1])
            elif short == "create_timer" and node.args:
                entity["period"] = _describe_expression(node.args[0])

            qos_positions = {"create_publisher": 2, "create_subscription": 3}
            qos_node = _keyword(node, "qos_profile", "qos")
            qos_position = qos_positions.get(short)
            if qos_node is None and qos_position is not None and len(node.args) > qos_position:
                qos_node = node.args[qos_position]
            if qos_node is not None:
                entity["qos"] = _describe_expression(qos_node)
                self.qos.append(_record(
                    self.path, node, entity=short, **entity["qos"]))

            callback_group = _keyword(node, "callback_group")
            if callback_group is not None:
                entity["callback_group"] = _describe_expression(callback_group)

            self.entities.append(_record(self.path, node, **entity))
            callback_index = {"create_subscription": 2, "create_timer": 1,
                              "create_service": 2}.get(short)
            if callback_index is not None and len(node.args) > callback_index:
                callback_name = _name(node.args[callback_index])
                function = self._resolve_function(callback_name)
                callback: dict[str, Any] = {"entity": short, "callback": callback_name}
                if entity.get("message_type_resolved"):
                    callback["message_type"] = entity["message_type_resolved"]
                if function is not None:
                    callback["definition_line"] = function.lineno
                    callback["ast_nodes"] = sum(1 for _ in ast.walk(function))
                    callback["has_loop"] = any(
                        isinstance(item, (ast.For, ast.AsyncFor, ast.While))
                        for item in ast.walk(function))
                    callback["native_library_uses"] = self._native_uses(function)
                else:
                    callback["resolution_blocker"] = "callback definition was not resolved"
                self.callbacks.append(_record(self.path, node, **callback))

        if short in EXECUTOR_NAMES:
            positional = ("num_threads",) if short == "MultiThreadedExecutor" else ("context",)
            self.executors.append(_record(
                self.path, node, executor=short, inputs=_call_inputs(node, positional)))
        if short in CALLBACK_GROUP_NAMES:
            self.callback_groups.append(_record(
                self.path, node, group=short, inputs=_call_inputs(node)))
        if short in QOS_NAMES or any(
            _name(keyword.value).rsplit(".", 1)[-1] in QOS_NAMES
            for keyword in node.keywords
        ):
            self.qos.append(_record(
                self.path, node, expression=called, inputs=_call_inputs(node)))
        self.generic_visit(node)


def _python_files(root: Path) -> list[Path]:
    files = []
    for path in root.rglob("*.py"):
        if any(part in EXCLUDED_DIRS for part in path.relative_to(root).parts):
            continue
        files.append(path)
    return sorted(files, key=lambda value: value.as_posix())


def _project_files(root: Path) -> list[Path]:
    return sorted(
        (
            path for path in root.rglob("*")
            if path.is_file() and not any(
                part in EXCLUDED_DIRS for part in path.relative_to(root).parts)
        ),
        key=lambda value: value.as_posix(),
    )


def _deduplicate(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen = set()
    result = []
    for record in records:
        key = json.dumps(record, sort_keys=True, separators=(",", ":"))
        if key not in seen:
            seen.add(key)
            result.append(record)
    return result


def _stable_strings(values: list[str]) -> list[str]:
    return list(dict.fromkeys(values))


def _recommendation(
    tier: int,
    reason: str,
    confidence: str,
    evidence: list[dict[str, Any]],
    blockers: list[str],
    **requirements: Any,
) -> dict[str, Any]:
    return {
        "tier": tier,
        "reason": reason,
        "confidence": confidence,
        "evidence": evidence,
        "blockers": _stable_strings(blockers),
        **requirements,
    }


def scan(root: Path) -> dict[str, Any]:
    root = root.resolve()
    observations: dict[str, list[dict[str, Any]]] = {
        key: [] for key in (
            "imports", "nodes", "entities", "executors", "callback_groups",
            "message_types", "qos", "callbacks",
        )
    }
    errors = []
    files = _python_files(root)
    for path in files:
        relative = path.relative_to(root).as_posix()
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=relative)
        except OSError as exc:
            error = f"{type(exc).__name__}: {exc.strerror or 'file read failed'}"
            errors.append({"file": relative, "error": error})
            continue
        except (UnicodeError, SyntaxError) as exc:
            errors.append({"file": relative, "error": str(exc)})
            continue
        visitor = ProjectVisitor(relative, tree)
        visitor.visit(tree)
        for key in observations:
            observations[key].extend(getattr(visitor, key))
    for key in observations:
        observations[key] = _deduplicate(observations[key])

    imported_roots = {
        item["module"].split(".", 1)[0]
        for item in observations["imports"] if item.get("module")
    }
    native_libraries = [
        {"import": name, "capability": KNOWN_NATIVE_LIBRARIES[name]}
        for name in sorted(imported_roots & KNOWN_NATIVE_LIBRARIES.keys())
    ]
    message_names = {item["type"] for item in observations["message_types"]}
    large_messages = sorted(message_names & KNOWN_LARGE_MESSAGES)
    complex_callbacks = [
        item for item in observations["callbacks"]
        if item.get("has_loop") or item.get("ast_nodes", 0) >= 40
    ]
    domain_callbacks = [
        item for item in observations["callbacks"]
        if item.get("native_library_uses") and
        item.get("message_type") in KNOWN_LARGE_MESSAGES
    ]
    entity_methods = {item["method"] for item in observations["entities"]}

    project_files = _project_files(root)
    test_files = [
        path.relative_to(root).as_posix() for path in project_files
        if path.suffix == ".py" and (
            path.name.startswith("test_") or path.name.endswith("_test.py") or
            any(part in ("test", "tests") for part in path.relative_to(root).parts[:-1])
        )
    ]
    benchmark_files = [
        path.relative_to(root).as_posix() for path in project_files
        if path.suffix == ".py" and (
            path.stem.startswith("bench") or "benchmark" in path.stem or
            any("benchmark" in part for part in path.relative_to(root).parts[:-1])
        )
    ]
    scanner_only = (root / SCANNER_ONLY_MARKER).is_file()
    evidence_blockers = []
    if scanner_only:
        evidence_blockers.append(
            "target is explicitly scanner-only and cannot establish runtime correctness or performance")
    if errors:
        evidence_blockers.append("scan contains parse errors and is incomplete")
    if not test_files:
        evidence_blockers.append(
            "no test files detected; establish a stock correctness command before editing")
    if not benchmark_files:
        evidence_blockers.append(
            "no benchmark harness detected; capture structured stock evidence before optimization")

    tiers = [_recommendation(
        0,
        "measure compatible activation and verify status before rewriting",
        "high",
        [{"kind": "project_inventory", "python_files": len(files)}],
        evidence_blockers + [
            "runtime backend selection requires an independent status assertion"],
        requires_measurement=True,
    )]
    if observations["executors"] or observations["callback_groups"]:
        tier_evidence = [
            {"kind": "executor", **item} for item in observations["executors"]
        ] + [
            {"kind": "callback_group", **item}
            for item in observations["callback_groups"]
        ]
        tiers.append(_recommendation(
            2,
            "executor or callback-group choices may justify explicit managed rclcpp",
            "medium",
            tier_evidence,
            evidence_blockers + [
                "managed-native changes Node, Context, or executor ownership and requires explicit opt-in",
                "the scheduling choice has not been benchmarked in isolation",
            ],
            requires_contract_opt_in=True,
        ))
    if complex_callbacks or entity_methods >= {"create_subscription", "create_publisher"}:
        tier_evidence = [
            {"kind": "complex_callback", **item} for item in complex_callbacks
        ]
        if entity_methods >= {"create_subscription", "create_publisher"}:
            tier_evidence.extend(
                {"kind": "relay_entity", **item}
                for item in observations["entities"]
                if item["method"] in ("create_subscription", "create_publisher")
            )
        tiers.append(_recommendation(
            3,
            "callback work or relay topology is a candidate for editable native lowering",
            "medium" if complex_callbacks else "low",
            tier_evidence,
            evidence_blockers + [
                "hot-path cost and Python boundary overhead are unmeasured",
                "wire/value parity and shutdown behavior require differential tests",
            ],
            requires_contract_opt_in=True,
        ))
    if domain_callbacks:
        tiers.append(_recommendation(
            4,
            "a large-message callback uses a known native library; inspect a lifetime-safe adapter",
            "low",
            [{"kind": "callback_native_library_use", **item} for item in domain_callbacks],
            evidence_blockers + [
                "native-library conversion and copy costs are unmeasured",
                "lifetime, ownership, and mutability require explicit proof",
            ],
            requires_contract_opt_in=True,
        ))

    launch_files = sorted(
        path.relative_to(root).as_posix()
        for path in project_files
        if path.name.endswith(".launch.py") or path.suffix in (".launch", ".xml")
    )
    return {
        "schema": SCHEMA,
        "root": ".",
        "files_scanned": len(files),
        "parse_errors": errors,
        "observations": observations,
        "signals": {
            "launch_files": launch_files,
            "known_large_messages": large_messages,
            "native_library_imports": native_libraries,
            "complex_callbacks": complex_callbacks,
            "callback_native_library_uses": domain_callbacks,
            "evidence": {
                "scanner_only": scanner_only,
                "test_files": test_files,
                "benchmark_files": benchmark_files,
                "blockers": evidence_blockers,
            },
        },
        "recommendation_inputs": tiers,
        "warnings": [
            "Static syntax cannot prove runtime backend selection or performance.",
            "Recommendation inputs are candidates, not implementation conclusions.",
            "Run differential tests and the same structured benchmark before and after a rewrite.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--strict", action="store_true",
        help="write the scan, then return non-zero when any file failed to parse")
    args = parser.parse_args(argv)
    if not args.target.is_dir():
        parser.error("target must be a directory")
    result = scan(args.target)
    encoded = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    if args.strict and result["parse_errors"]:
        print("scan is incomplete because one or more Python files failed to parse", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
