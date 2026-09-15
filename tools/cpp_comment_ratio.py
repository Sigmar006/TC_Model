#!/usr/bin/env python3
"""Report C/C++ comment ratios using CLOC-like physical-line classification."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Counts:
    code: int = 0
    comment: int = 0
    blank: int = 0

    def add(self, other: "Counts") -> None:
        self.code += other.code
        self.comment += other.comment
        self.blank += other.blank

    @property
    def ratio(self) -> float:
        denominator = self.code + self.comment
        return self.comment / denominator if denominator else 0.0


def classify(path: Path) -> Counts:
    counts = Counts()
    in_block_comment = False
    for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
        has_code = False
        has_comment = False
        in_string: str | None = None
        escaped = False
        index = 0
        while index < len(line):
            current = line[index]
            following = line[index + 1] if index + 1 < len(line) else ""
            if in_block_comment:
                has_comment = True
                if current == "*" and following == "/":
                    in_block_comment = False
                    index += 2
                else:
                    index += 1
                continue
            if in_string is not None:
                has_code = True
                if escaped:
                    escaped = False
                elif current == "\\":
                    escaped = True
                elif current == in_string:
                    in_string = None
                index += 1
                continue
            if current == "/" and following == "/":
                has_comment = True
                break
            if current == "/" and following == "*":
                has_comment = True
                in_block_comment = True
                index += 2
                continue
            if current in {'"', "'"}:
                has_code = True
                in_string = current
            elif not current.isspace():
                has_code = True
            index += 1

        if has_code:
            counts.code += 1
        elif has_comment:
            counts.comment += 1
        else:
            counts.blank += 1
    return counts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("directories", nargs="+", type=Path)
    parser.add_argument("--lowest", type=int, default=0)
    parser.add_argument("--csv", type=Path, dest="csv_path")
    args = parser.parse_args()

    summary_rows: list[dict[str, str | int]] = []
    for directory in args.directories:
        files = sorted(
            path
            for path in directory.rglob("*")
            if path.is_file() and path.suffix.lower() in {".h", ".cpp"}
        )
        total = Counts()
        by_file: list[tuple[float, Path, Counts]] = []
        for path in files:
            counts = classify(path)
            total.add(counts)
            by_file.append((counts.ratio, path, counts))
        print(
            f"{directory}: files={len(files)} code={total.code} comment={total.comment} "
            f"blank={total.blank} comment_ratio={total.ratio * 100.0:.2f}%"
        )
        summary_rows.append(
            {
                "directory": str(directory).replace("\\", "/"),
                "files": len(files),
                "code_lines": total.code,
                "comment_lines": total.comment,
                "blank_lines": total.blank,
                "comment_ratio_percent": f"{total.ratio * 100.0:.2f}",
            }
        )
        for _, path, counts in sorted(by_file)[: args.lowest]:
            print(
                f"  {counts.ratio * 100.0:6.2f}%  code={counts.code:5d} "
                f"comment={counts.comment:5d}  {path}"
            )
    if args.csv_path is not None:
        args.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with args.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0]))
            writer.writeheader()
            writer.writerows(summary_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
