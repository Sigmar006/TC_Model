#!/usr/bin/env python3
"""守卫 C/C++ 字符串字面量的编译期字符集：字面量内不得出现 ANSI 代码页表示不了的字符。

背景（2026-09-28 实测）：交付库面向不传 `/utf-8` 的 VS2015 消费方，cl.exe 按系统
ANSI 代码页（中文 Windows = GBK/cp936）生成窄字符串字面量。字面量里 GBK 码页没有的
字符会被 MSVC **静默转写成 `?`** 编进库——源文件正确、编译不报错，只有落盘日志看得出
（`目标RCS=0.100m²` → `目标RCS=0.100m?`）；验收读数不受影响，但证据文件对不上甲方
判据文本。规避写法是纯 ASCII（`m^2`、`ft^2`、`m/s^2`），或显式 `u8"..."`。

范围与判定：
  - 扫 git 追踪的 C/C++ 源文件，外加未忽略的未跟踪文件（新增但尚未提交的源文件同样
    要守）；扩展名集合与 scripts/utf8_bom.py 一致，避免两套口径。
  - 用小型词法状态机逐字符判定，只查**字符串/字符字面量内部**：注释与文档里的非 GBK
    字符无害（源文件带 BOM，cl.exe 按 UTF-8 解码），不报。
  - `u8"..."` / `L"..."` / `u"..."` / `U"..."` 前缀字面量跳过：它们由编译器显式转成
    UTF-8/UTF-16，不受 ANSI 代码页影响。
  - 判定基准是 cp936（GBK）编解码往返，不依赖运行本脚本的机器代码页，macOS/Linux 的
    CI 上结论与中文 Windows 编译机一致。

退出码：0 = 全部字面量为 ANSI 安全字符；1 = 有违例。
"""

import argparse
import subprocess
import sys
from pathlib import Path

# 与 scripts/utf8_bom.py 的 SOURCE_EXTENSIONS、.editorconfig 的 utf-8-bom 段同范围
SOURCE_EXTENSIONS = (
    ".h", ".hh", ".hpp", ".hxx", ".ipp", ".inl",
    ".c", ".cc", ".cpp", ".cxx",
)

# 编译机 ANSI 代码页（中文 Windows 简体中文 = 936）
ANSI_CODEC = "cp936"

# 仓库根目录（本脚本位于 scripts/ 下）
REPO_ROOT = Path(__file__).resolve().parent.parent

# 跳过这些字面量前缀：编译器已显式转码，与 ANSI 代码页无关
UNCHECKED_PREFIXES = ("u8", "u", "U", "L")

UTF8_BOM = "\ufeff"


def is_source_file(path: str) -> bool:
    return path.endswith(SOURCE_EXTENSIONS)


def list_source_files() -> list:
    """追踪文件 + 未忽略的未跟踪文件（相对路径，/ 分隔，去重排序）。"""
    names = set()
    for extra in (["--others", "--exclude-standard"], []):
        output = subprocess.check_output(
            ["git", "-C", str(REPO_ROOT), "ls-files", "-z"] + extra, text=False
        )
        names.update(name for name in output.decode("utf-8").split("\0") if name)
    return sorted(name for name in names if is_source_file(name))


def ansi_safe(ch: str) -> bool:
    """该字符能否由 ANSI 代码页（GBK）表示；不能则会被 MSVC 转写成 `?`。"""
    try:
        ch.encode(ANSI_CODEC)
        return True
    except UnicodeEncodeError:
        return False


class LiteralScanner:
    """C/C++ 词法状态机：只收集字面量内部的字符（行号 1 起，列号 1 起）。

    刻意不追求完整预处理语义——按 `//`、`/* */`、`"..."`、`'...'`、`R"delim(...)delim"`
    切分即可覆盖仓库实际写法；误判方向偏保守（宁可漏进注释，不把注释当字面量报）。
    """

    def __init__(self, text: str):
        self.text = text
        self.index = 0
        self.line = 1
        self.col = 1
        self.hits = []  # (line, col, char, literal_preview)

    def _advance(self, count: int = 1) -> None:
        for _ in range(count):
            if self.index >= len(self.text):
                return
            if self.text[self.index] == "\n":
                self.line += 1
                self.col = 1
            else:
                self.col += 1
            self.index += 1

    def _peek(self, offset: int = 0) -> str:
        pos = self.index + offset
        return self.text[pos] if pos < len(self.text) else ""

    def scan(self) -> list:
        while self.index < len(self.text):
            ch = self._peek()
            if ch == "/" and self._peek(1) == "/":
                self._skip_line_comment()
            elif ch == "/" and self._peek(1) == "*":
                self._skip_block_comment()
            elif ch == '"':
                self._read_literal('"')
            elif ch == "'":
                self._read_literal("'")
            elif ch in "RruUL" and self._literal_prefix_length() >= 0:
                self._read_prefixed_literal()
            else:
                self._advance()
        return self.hits

    def _skip_line_comment(self) -> None:
        while self.index < len(self.text) and self._peek() != "\n":
            self._advance()

    def _skip_block_comment(self) -> None:
        self._advance(2)
        while self.index < len(self.text):
            if self._peek() == "*" and self._peek(1) == "/":
                self._advance(2)
                return
            self._advance()

    def _literal_prefix_length(self) -> int:
        """返回紧随其后的字面量前缀长度（如 `u8` 为 2）；不是字面量前缀则返回 -1。"""
        for prefix in ("u8", "u", "U", "L", "R", "u8R", "uR", "UR", "LR"):
            if self.text.startswith(prefix, self.index) and self._peek(len(prefix)) == '"':
                return len(prefix)
        return -1

    def _read_prefixed_literal(self) -> None:
        prefix_len = self._literal_prefix_length()
        if prefix_len < 0:
            self._advance()
            return
        prefix = self.text[self.index:self.index + prefix_len]
        self._advance(prefix_len)
        if "R" in prefix:
            self._read_raw_literal()
            return
        # u8 / u / U / L：编译器显式转码，与 ANSI 代码页无关，不检查
        self._read_literal('"', unchecked=True)

    def _read_raw_literal(self) -> None:
        delimiter_end = self.text.find("(", self.index + 1)
        if delimiter_end < 0:
            self._advance()
            return
        delimiter = self.text[self.index + 1:delimiter_end]
        closer = ")" + delimiter + '"'
        start_line, start_col = self.line, self.col
        self._advance(delimiter_end - self.index + 1)
        body_start = self.index
        end = self.text.find(closer, self.index)
        if end < 0:
            end = len(self.text)
        self._collect(self.text[body_start:end], start_line, start_col)
        self._advance(end - self.index + len(closer))

    def _read_literal(self, quote: str, unchecked: bool = False) -> None:
        start_line, start_col = self.line, self.col
        self._advance()  # 开引号
        body = []
        while self.index < len(self.text):
            ch = self._peek()
            if ch == "\\":  # 转义序列（含续行）按两字符吞掉
                body.append(ch)
                self._advance()
                if self.index < len(self.text):
                    body.append(self._peek())
                    self._advance()
                continue
            if ch == quote:
                self._advance()
                break
            if ch == "\n":  # 未闭合字面量：止于行尾，避免吞掉整份文件
                break
            body.append(ch)
            self._advance()
        if not unchecked:
            self._collect("".join(body), start_line, start_col)

    def _collect(self, body: str, start_line: int, start_col: int) -> None:
        line, col = start_line, start_col + 1  # 开引号之后
        for ch in body:
            if ch == "\n":
                line, col = line + 1, 1
                continue
            if ord(ch) >= 128 and not ansi_safe(ch):
                self.hits.append((line, col, ch, body.strip()[:80]))
            col += 1


def check_file(rel_path: str) -> list:
    data = (REPO_ROOT / rel_path).read_bytes()
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        # 非合法 UTF-8 由 scripts/utf8_bom.py check 负责报错，这里不重复报
        return []
    if text.startswith(UTF8_BOM):
        text = text[len(UTF8_BOM):]
    return [(rel_path,) + hit for hit in LiteralScanner(text).scan()]


def cmd_check(args) -> int:
    files = list_source_files()
    violations = []
    for name in files:
        violations.extend(check_file(name))

    if violations:
        for name, line, col, ch, preview in violations:
            print(
                f"error: {name}:{line}:{col}: 字符串字面量含 U+{ord(ch):04X} {ch!r}"
                f"（{ANSI_CODEC} 无法表示，MSVC 会编成 '?'）",
                file=sys.stderr,
            )
            print(f"       字面量: {preview}", file=sys.stderr)
        print(
            f"check 失败: {len(violations)} 处违例；改用纯 ASCII（如 m^2 / ft^2 / m/s^2），"
            f"或显式 u8\"...\"",
            file=sys.stderr,
        )
        return 1
    print(f"check 通过: {len(files)} 个 C/C++ 源文件的字符串字面量均为 {ANSI_CODEC} 可表示字符")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="扫描全部 C/C++ 源文件的字符串字面量（CI 守卫）")
    args = parser.parse_args(argv)
    return cmd_check(args)


if __name__ == "__main__":
    # Windows 控制台代码页可能不是 UTF-8，避免打印中文/符号时抛异常
    try:
        sys.stdout.reconfigure(errors="replace")
        sys.stderr.reconfigure(errors="replace")
    except AttributeError:
        pass
    sys.exit(main())
