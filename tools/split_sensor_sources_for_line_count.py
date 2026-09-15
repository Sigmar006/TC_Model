#!/usr/bin/env python3
"""Create a source snapshot split for SBIRS infrared and LRDR/RIR line counts."""

from __future__ import annotations

import csv
import re
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "code_line_statistics_split"

INFRARED_DIRS = [
    Path("include/1q/sbirs_sensor"),
    Path("src/sbirs_sensor"),
]

LRDR_DIRS = [
    Path("include/1q/remote_identification_radar"),
    Path("src/remote_identification_radar"),
]

SHARED_PREFIXES = [
    "1q/coordinate/",
    "1q/electromagnetics/",
    "1q/environment/",
    "1q/foundation/",
    "1q/replay/",
    "common/",
]

INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+)"')


def source_files_under(paths: list[Path]) -> set[Path]:
    files: set[Path] = set()
    for directory in paths:
        absolute = ROOT / directory
        if not absolute.exists():
            continue
        for pattern in ("*.h", "*.cpp"):
            for path in absolute.rglob(pattern):
                files.add(path.relative_to(ROOT))
    return files


def resolve_include(include_name: str) -> Path | None:
    candidates: list[Path] = []
    if include_name.startswith("1q/"):
        candidates.append(Path("include") / include_name)
    else:
        candidates.append(Path("src") / include_name)
        candidates.append(Path("include/1q") / include_name)

    for candidate in candidates:
        if (ROOT / candidate).exists() and candidate.suffix in {".h", ".cpp"}:
            return candidate
    return None


def direct_shared_includes(files: set[Path]) -> set[Path]:
    includes: set[Path] = set()
    for relative in files:
        absolute = ROOT / relative
        try:
            for line in absolute.read_text(encoding="utf-8-sig").splitlines():
                match = INCLUDE_RE.match(line)
                if not match:
                    continue
                include_name = match.group(1)
                if not any(include_name.startswith(prefix) for prefix in SHARED_PREFIXES):
                    continue
                resolved = resolve_include(include_name)
                if resolved is not None:
                    includes.add(resolved)
        except UnicodeDecodeError:
            for line in absolute.read_text(encoding="gb18030").splitlines():
                match = INCLUDE_RE.match(line)
                if match:
                    resolved = resolve_include(match.group(1))
                    if resolved is not None:
                        includes.add(resolved)
    return includes


def add_companions(files: set[Path]) -> set[Path]:
    expanded = set(files)
    for relative in list(files):
        if relative.suffix == ".h":
            companion = relative.with_suffix(".cpp")
        elif relative.suffix == ".cpp":
            companion = relative.with_suffix(".h")
        else:
            continue
        if (ROOT / companion).exists():
            expanded.add(companion)
    return expanded


def copy_files(category: str, files: set[Path]) -> None:
    for relative in sorted(files):
        destination = OUTPUT / category / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, destination)


def count_lines(relative_to_output: Path) -> tuple[int, int]:
    text = relative_to_output.read_text(encoding="utf-8-sig", errors="replace")
    lines = text.splitlines()
    nonblank = sum(1 for line in lines if line.strip())
    return len(lines), nonblank


def write_reports(categories: dict[str, set[Path]]) -> None:
    rows: list[dict[str, str | int]] = []
    summary: list[tuple[str, int, int, int, int]] = []
    for category, files in categories.items():
        total_lines = 0
        total_nonblank = 0
        for relative in sorted(files):
            copied = OUTPUT / category / relative
            lines, nonblank = count_lines(copied)
            total_lines += lines
            total_nonblank += nonblank
            rows.append(
                {
                    "category": category,
                    "path": str(relative).replace("\\", "/"),
                    "extension": relative.suffix,
                    "lines": lines,
                    "nonblank_lines": nonblank,
                }
            )
        summary.append((category, len(files), total_lines, total_nonblank, sum(1 for f in files if f.suffix == ".h")))

    with (OUTPUT / "line_count_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["category", "path", "extension", "lines", "nonblank_lines"])
        writer.writeheader()
        writer.writerows(rows)

    with (OUTPUT / "MANIFEST.md").open("w", encoding="utf-8") as handle:
        handle.write("# Code Line Statistics Split\n\n")
        handle.write("Scope: copied `.h` and `.cpp` files only. Original project files were not moved.\n\n")
        handle.write("- `infrared_sensor`: SBIRS infrared sensor API and implementation files.\n")
        handle.write("- `lrdr_radar`: LRDR acceptance radar code as represented by the RIR module in this repo.\n")
        handle.write("- `shared`: direct shared source/header dependencies included by both SBIRS and RIR.\n\n")
        handle.write("| category | files | .h files | .cpp files | lines | nonblank lines |\n")
        handle.write("| --- | ---: | ---: | ---: | ---: | ---: |\n")
        for category, file_count, lines, nonblank, header_count in summary:
            cpp_count = file_count - header_count
            handle.write(f"| {category} | {file_count} | {header_count} | {cpp_count} | {lines} | {nonblank} |\n")


def main() -> int:
    if OUTPUT.exists():
        raise SystemExit(f"output already exists: {OUTPUT}")

    infrared = source_files_under(INFRARED_DIRS)
    lrdr = source_files_under(LRDR_DIRS)
    shared = add_companions(direct_shared_includes(infrared) & direct_shared_includes(lrdr))

    categories = {
        "infrared_sensor": infrared,
        "lrdr_radar": lrdr,
        "shared": shared,
    }

    OUTPUT.mkdir()
    for category, files in categories.items():
        copy_files(category, files)
    write_reports(categories)

    for category, files in categories.items():
        headers = sum(1 for file in files if file.suffix == ".h")
        print(f"{category}: {len(files)} files ({headers} .h, {len(files) - headers} .cpp)")
    print(f"wrote {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
