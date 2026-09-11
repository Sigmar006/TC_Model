#!/usr/bin/env python3
"""Build guard: refuse cmake --build when configure/reconfigure is required."""

from __future__ import annotations

import importlib.util
import os
import re
import sys
from pathlib import Path

_LIB_DIR = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("1q_preset_query", _LIB_DIR / "1q_preset_query.py")
if _spec is None or _spec.loader is None:
    raise RuntimeError("failed to load 1q_preset_query.py")
_preset = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_preset)


def _path_candidates(raw: str, root: Path) -> list[Path]:
    text = raw.strip().replace("\\", "/")
    if not text:
        return []
    out: list[Path] = [Path(text)]
    match = re.match(r"^([A-Za-z]):/(.*)$", text)
    if match:
        drive, rest = match.group(1).lower(), match.group(2)
        out.append(Path(f"/mnt/{drive}/{rest}"))
    out.append(root / text)
    if match:
        out.append(root / rest)
    dedup: list[Path] = []
    seen: set[str] = set()
    for item in out:
        key = str(item)
        if key not in seen:
            seen.add(key)
            dedup.append(item)
    return dedup


def _resolve_existing(raw: str, root: Path) -> Path | None:
    for candidate in _path_candidates(raw, root):
        try:
            if candidate.is_file():
                return candidate
        except OSError:
            continue
    return None


def _cmake_generator(binary_dir: Path) -> str:
    cache = binary_dir / "CMakeCache.txt"
    try:
        for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("CMAKE_GENERATOR:INTERNAL="):
                return line.split("=", 1)[1].strip()
    except OSError:
        pass
    return ""


def cmake_stamp_stale(binary_dir: Path, root: Path) -> str | None:
    """仅在生成器产出 generate.stamp 时判定陈旧（VS / Unix Makefiles）。

    Ninja 树没有 CMakeFiles/generate.stamp——Ninja 在 build 时自行扫描依赖并
    触发 regen，因此"无 stamp"不等于"未配置"（未配置已由 CMakeCache.txt 检查
    覆盖）。此处放行，避免 macOS 主线 preset 被误拦。
    """
    if _cmake_generator(binary_dir).startswith("Ninja"):
        return None

    stamp = binary_dir / "CMakeFiles" / "generate.stamp"
    depend = binary_dir / "CMakeFiles" / "generate.stamp.depend"
    if not stamp.is_file():
        return "<missing generate.stamp>"
    if not depend.is_file():
        return "<missing generate.stamp.depend>"

    stamp_mtime = stamp.stat().st_mtime
    for line in depend.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        resolved = _resolve_existing(line, root)
        if resolved is None:
            continue
        try:
            if resolved.stat().st_mtime > stamp_mtime:
                return str(resolved)
        except OSError:
            continue
    return None


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: 1q_build_guard.py <build-preset>", file=sys.stderr)
        return 2

    build_preset = sys.argv[1].strip()
    root = Path(os.environ.get("ONEQ_ROOT", ".")).resolve()

    try:
        configure_preset = _preset._resolve_field(build_preset, "configurePreset", root)
        binary_dir = Path(_preset._resolve_field(build_preset, "binaryDir", root))
    except SystemExit as exc:
        print(f"[1q.sh build] {exc}", file=sys.stderr)
        return 1

    cache = binary_dir / "CMakeCache.txt"
    if not cache.is_file():
        print(
            f"[1q.sh build] 构建树未配置：{binary_dir}\n"
            f"  请先对应当前 preset 显式 configure（禁止 build 时自动 reconfigure）：\n"
            f"    scripts/1q.sh configure {configure_preset}\n"
            f"  切换 preset 时必须 configure 该 preset 对应的 configure_preset，"
            f"不能混用另一棵 build 树。",
            file=sys.stderr,
        )
        return 1

    stale = cmake_stamp_stale(binary_dir, root)
    if stale is not None:
        print(
            f"[1q.sh build] CMake 输入已比 generate.stamp 新，拒绝隐式 reconfigure。\n"
            f"  过期依赖：{stale}\n"
            f"  请显式 configure（会触发一次全量重编，属预期）：\n"
            f"    scripts/1q.sh configure {configure_preset}\n"
            f"  日常只改业务 .cpp 时不应看到此提示；若频繁出现，检查是否误改 cmake/ 或重复 configure。",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
