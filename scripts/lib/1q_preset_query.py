#!/usr/bin/env python3
"""Resolve CMakePresets.json (+ optional CMakeUserPresets.json) for 1q.sh guards."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any


def _load_preset_files(root: Path) -> list[dict[str, Any]]:
    presets: list[dict[str, Any]] = []
    for name in ("CMakePresets.json", "CMakeUserPresets.json"):
        path = root / name
        if not path.is_file():
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        presets.extend(data.get("configurePresets", []))
    return presets


def _load_build_presets(root: Path) -> list[dict[str, Any]]:
    presets: list[dict[str, Any]] = []
    for name in ("CMakePresets.json", "CMakeUserPresets.json"):
        path = root / name
        if not path.is_file():
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        presets.extend(data.get("buildPresets", []))
    return presets


def _inherit_names(value: Any) -> list[str]:
    """CMake presets 的 `inherits` 允许字符串或字符串数组两种写法。"""
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, list):
        return [item for item in value if isinstance(item, str)]
    raise SystemExit(f"invalid `inherits` value: {value!r}")


def _merge_configure_preset(
    by_name: dict[str, dict[str, Any]], name: str, chain: frozenset[str] = frozenset()
) -> dict[str, Any]:
    # chain 逐分支拷贝（而非共享可变集合），否则 A、B 同时继承 C 的菱形结构
    # 会被误判成循环继承。
    if name in chain:
        raise SystemExit(f"circular configure preset inheritance at {name}")
    if name not in by_name:
        raise SystemExit(f"unknown configure preset: {name}")

    preset = dict(by_name[name])
    next_chain = chain | {name}
    merged: dict[str, Any] = {}
    parent_cache: dict[str, Any] = {}
    # 数组形式按 CMake 语义合并：靠后的父 preset 覆盖靠前的，自身字段覆盖全部父。
    for parent_name in _inherit_names(preset.pop("inherits", None)):
        parent = _merge_configure_preset(by_name, parent_name, next_chain)
        parent_cache.update(parent.get("cacheVariables", {}))
        merged.update(parent)
    merged.update(preset)
    if parent_cache or "cacheVariables" in preset:
        merged["cacheVariables"] = {**parent_cache, **preset.get("cacheVariables", {})}
    return merged


def _binary_dir(root: Path, configure_preset: str, merged: dict[str, Any]) -> Path:
    template = merged.get("binaryDir", "${sourceDir}/build/${presetName}")
    resolved = (
        template.replace("${sourceDir}", str(root))
        .replace("${presetName}", configure_preset)
    )
    return Path(resolved)


def _resolve_field(preset_name: str, field: str, root: Path) -> str:
    configure_presets = _load_preset_files(root)
    by_name = {p["name"]: p for p in configure_presets if "name" in p}

    build_presets = _load_build_presets(root)
    build_by_name = {p["name"]: p for p in build_presets if "name" in p}

    configure_name = preset_name
    if preset_name in build_by_name:
        configure_name = build_by_name[preset_name]["configurePreset"]
    elif preset_name not in by_name:
        raise SystemExit(f"unknown preset: {preset_name}")

    merged = _merge_configure_preset(by_name, configure_name)
    if field == "configurePreset":
        return configure_name
    if field == "binaryDir":
        return str(_binary_dir(root, configure_name, merged))
    raise SystemExit(f"unknown field: {field}")


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: 1q_preset_query.py <build-preset|configure-preset> <field>", file=sys.stderr)
        print("  field: configurePreset | binaryDir", file=sys.stderr)
        return 2

    preset_name = sys.argv[1]
    field = sys.argv[2]
    root = Path(os.environ.get("ONEQ_ROOT", ".")).resolve()

    try:
        print(_resolve_field(preset_name, field, root))
    except SystemExit as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
