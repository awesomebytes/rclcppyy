#!/usr/bin/env python3
"""Scan a ROS 2 Python project into deterministic optimization inputs."""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import sys
from typing import Any


SCHEMA = "rclcppyy.project-scan/v1"
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


def _record(path: str, node: ast.AST, **values: Any) -> dict[str, Any]:
    return {"file": path, "line": node.lineno, **values}


class ProjectVisitor(ast.NodeVisitor):
    def __init__(self, path: str):
        self.path = path
        self.imports: list[dict[str, Any]] = []
        self.nodes: list[dict[str, Any]] = []
        self.entities: list[dict[str, Any]] = []
        self.executors: list[dict[str, Any]] = []
        self.callback_groups: list[dict[str, Any]] = []
        self.message_types: list[dict[str, Any]] = []
        self.qos: list[dict[str, Any]] = []
        self.callbacks: list[dict[str, Any]] = []
        self._functions: dict[str, ast.FunctionDef | ast.AsyncFunctionDef] = {}

    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            self.imports.append(_record(self.path, node, module=alias.name))
        self.generic_visit(node)

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        module = node.module or ""
        for alias in node.names:
            self.imports.append(
                _record(self.path, node, module=module, symbol=alias.name))
            if any(part in module.split(".") for part in ("msg", "srv", "action")):
                self.message_types.append(
                    _record(self.path, node, module=module, type=alias.name))
        self.generic_visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self._functions[node.name] = node
        self.generic_visit(node)

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._functions[node.name] = node
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call) -> None:
        called = _name(node.func)
        short = called.rsplit(".", 1)[-1]
        if short in ("create_node", "Node"):
            name = _literal(node.args[0]) if node.args else None
            self.nodes.append(_record(self.path, node, call=called, name=name))
        if short in ENTITY_METHODS:
            entity = {"method": short}
            if short in ("create_publisher", "create_subscription"):
                if node.args:
                    entity["message_type"] = _name(node.args[0])
                if len(node.args) > 1:
                    entity["topic"] = _literal(node.args[1])
            elif short in ("create_service", "create_client"):
                if node.args:
                    entity["service_type"] = _name(node.args[0])
                if len(node.args) > 1:
                    entity["service"] = _literal(node.args[1])
            elif short == "create_timer" and node.args:
                entity["period"] = _literal(node.args[0])
            self.entities.append(_record(self.path, node, **entity))
            callback_index = {"create_subscription": 2, "create_timer": 1,
                              "create_service": 2}.get(short)
            if callback_index is not None and len(node.args) > callback_index:
                callback_name = _name(node.args[callback_index])
                function = self._functions.get(callback_name.rsplit(".", 1)[-1])
                callback = {"entity": short, "callback": callback_name}
                if function is not None:
                    callback["ast_nodes"] = sum(1 for _ in ast.walk(function))
                    callback["has_loop"] = any(
                        isinstance(item, (ast.For, ast.AsyncFor, ast.While))
                        for item in ast.walk(function))
                self.callbacks.append(_record(self.path, node, **callback))
        if short in EXECUTOR_NAMES:
            self.executors.append(_record(self.path, node, executor=short))
        if short in CALLBACK_GROUP_NAMES:
            self.callback_groups.append(_record(self.path, node, group=short))
        if short in QOS_NAMES or any(
            _name(keyword.value).rsplit(".", 1)[-1] in QOS_NAMES
            for keyword in node.keywords
        ):
            self.qos.append(_record(self.path, node, expression=called))
        self.generic_visit(node)


def _python_files(root: Path) -> list[Path]:
    files = []
    for path in root.rglob("*.py"):
        if any(part in EXCLUDED_DIRS for part in path.relative_to(root).parts):
            continue
        files.append(path)
    return sorted(files, key=lambda value: value.as_posix())


def _deduplicate(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen = set()
    result = []
    for record in records:
        key = json.dumps(record, sort_keys=True, separators=(",", ":"))
        if key not in seen:
            seen.add(key)
            result.append(record)
    return result


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
        except (OSError, UnicodeError, SyntaxError) as exc:
            errors.append({"file": relative, "error": str(exc)})
            continue
        visitor = ProjectVisitor(relative)
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
    entity_methods = {item["method"] for item in observations["entities"]}
    tiers = [{
        "tier": 0,
        "reason": "measure compatible activation and verify status before rewriting",
        "requires_measurement": True,
    }]
    if observations["executors"] or observations["callback_groups"]:
        tiers.append({
            "tier": 2,
            "reason": "executor or callback-group choices may benefit from explicit managed rclcpp",
            "requires_contract_opt_in": True,
        })
    if complex_callbacks or entity_methods >= {"create_subscription", "create_publisher"}:
        tiers.append({
            "tier": 3,
            "reason": "callback work or relay topology is a candidate for editable native lowering",
            "requires_contract_opt_in": True,
        })
    if native_libraries and large_messages:
        tiers.append({
            "tier": 4,
            "reason": "large ROS messages and a native library coexist; inspect a lifetime-safe domain adapter",
            "requires_contract_opt_in": True,
        })

    launch_files = sorted(
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() and (
            path.name.endswith(".launch.py") or path.suffix in (".launch", ".xml")
        ) and not any(part in EXCLUDED_DIRS for part in path.relative_to(root).parts)
    )
    return {
        "schema": SCHEMA,
        "root": root.as_posix(),
        "files_scanned": len(files),
        "parse_errors": errors,
        "observations": observations,
        "signals": {
            "launch_files": launch_files,
            "known_large_messages": large_messages,
            "native_library_imports": native_libraries,
            "complex_callbacks": complex_callbacks,
        },
        "recommendation_inputs": tiers,
        "warnings": [
            "Static syntax cannot prove runtime backend selection or performance.",
            "Run differential tests and the same structured benchmark before and after a rewrite.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if not args.target.is_dir():
        parser.error("target must be a directory")
    result = scan(args.target)
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
