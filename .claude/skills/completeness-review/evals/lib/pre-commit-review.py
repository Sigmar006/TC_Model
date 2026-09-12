#!/usr/bin/env python3
"""Analyze staged changes for review scope classification.

Used by git pre-commit hook and manual review decisions.
Reads git diff --cached and outputs a JSON tier classification:

  {"tier": "trivial", "cpp_files": 0, "cpp_lines": 0, "modules": []}
  {"tier": "minor",   "cpp_files": N, "cpp_lines": M, "modules": [...]}
  {"tier": "major",   "cpp_files": N, "cpp_lines": M, "modules": [...]}
  {"tier": "empty"}
  {"tier": "error", "error": "..."}

Tiers (review depth follows risk — core library vs example/doc layers):
  trivial — no C++ files changed (config/docs only)
  minor   — C++ changes confined to non-core layers (examples/ tests/ docs/
             tools/ cmake/): example-layer refactors, test-only edits, pure
             deletions do NOT trigger the full three-lane review; build +
             focused tests suffice. Also <3 core files and <50 core lines.
  major   — >=3 C++ files or >=50 lines touching the core library
             (src/ or include/1q/): algorithm/contract changes need the full
             /completeness-review flow.

Line counts come from `git diff --cached --numstat` (exact numbers; the
--stat '+' character display is column-compressed and unreliable).
"""

import json
import subprocess
import sys


def analyze():
    try:
        result = subprocess.run(
            ["git", "diff", "--cached", "--numstat"],
            capture_output=True, text=True, timeout=5
        )
        lines = result.stdout.strip().split("\n")
        if not lines or lines == [""]:
            return {"tier": "empty"}

        cpp_patterns = (".cpp", ".h", ".hpp", ".cc", ".cxx", ".c", ".cmake")
        cpp_files = 0
        cpp_insertions = 0
        cpp_deletions = 0
        core_cpp_files = 0
        core_cpp_lines = 0
        total_files = 0
        modules = set()

        for line in lines:
            parts = line.split("\t")
            if len(parts) < 3:
                continue
            adds, dels, path = parts[0], parts[1], parts[2]
            # 重命名行（numstat 的路径列形如 "old => new"）：按新路径分类。
            if " => " in path:
                path = path.split(" => ")[-1]
            total_files += 1
            try:
                add_count = int(adds)
                del_count = int(dels)
            except ValueError:
                add_count = del_count = 0  # 二进制/非常规行：仅统计文件数

            is_core = path.startswith("src/") or path.startswith("include/1q/")
            if path.endswith(cpp_patterns):
                cpp_files += 1
                cpp_insertions += add_count
                cpp_deletions += del_count
                if is_core:
                    core_cpp_files += 1
                    core_cpp_lines += add_count + del_count

            if path.startswith("src/"):
                path_parts = path.split("/")
                if len(path_parts) >= 3:
                    modules.add(path_parts[1])
            elif path.startswith("include/1q/"):
                path_parts = path.split("/")
                if len(path_parts) >= 3:
                    modules.add(path_parts[2])
            elif path.startswith("tests/"):
                path_parts = path.split("/")
                if len(path_parts) >= 3:
                    modules.add(path_parts[2])

        cpp_total = cpp_insertions + cpp_deletions

        if cpp_files == 0:
            return {"tier": "trivial", "cpp_files": 0, "cpp_lines": 0,
                    "total_files": total_files, "modules": sorted(modules)}
        elif core_cpp_files >= 3 or core_cpp_lines >= 50:
            return {"tier": "major", "cpp_files": cpp_files, "cpp_lines": cpp_total,
                    "total_files": total_files, "modules": sorted(modules)}
        else:
            return {"tier": "minor", "cpp_files": cpp_files, "cpp_lines": cpp_total,
                    "total_files": total_files, "modules": sorted(modules)}

    except Exception as e:
        return {"tier": "error", "error": str(e)}


if __name__ == "__main__":
    print(json.dumps(analyze()))
