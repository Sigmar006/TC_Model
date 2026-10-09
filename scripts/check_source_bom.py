#!/usr/bin/env python3
"""源码编码守卫：C/C++ 源文件必须携带 Unicode BOM。

背景（仓库既有立场，见 tests/contract/ReadSourceLines.cmake）：
VS2015 集成端按系统代码页误读无 BOM 文件，中文注释会错位截断。本次开发
实际踩到该问题——编辑工具重写文件会静默丢掉 BOM，症状是编译器在中文注释
之后的代码处报出完全不相干的「未声明的标识符」，排查成本很高。

接受的 BOM：UTF-8（EF BB BF）、UTF-16 LE（FF FE）、UTF-16 BE（FE FF）。
仓库绝大多数源文件是 UTF-8 + BOM；`cmake/compat/oneq_vs2013_compat.h`
是既有的 UTF-16 LE 文件，同样带 BOM，属合法形态。

本守卫是确定性的（不依赖编译器行为）：
    python scripts/check_source_bom.py
"""

import pathlib
import sys

BOMS = (b'\xef\xbb\xbf', b'\xff\xfe', b'\xfe\xff')
SUFFIXES = ('.h', '.hpp', '.cpp', '.cc', '.cxx')
# third_party/vendor 代码不适用本仓库的 BOM 立场（它们自带编码约定）。
SKIP_DIRS = {'build', '.git', '.cache', 'node_modules', 'third_party', 'vendor', 'external'}


def main() -> int:
    root = pathlib.Path(__file__).resolve().parent.parent
    missing = []
    for p in root.rglob('*'):
        if not p.is_file() or p.suffix not in SUFFIXES:
            continue
        if any(part in SKIP_DIRS for part in p.parts):
            continue
        head = p.read_bytes()[:3]
        if not any(head.startswith(bom) for bom in BOMS):
            missing.append(p.relative_to(root))

    if not missing:
        print('BOM 检查通过：全部 C/C++ 源文件均携带 Unicode BOM')
        return 0
    print(f'BOM 检查失败：{len(missing)} 个文件缺少 Unicode BOM（VS2015 集成端会误读中文注释）')
    for name in missing:
        print(f'  {name}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
