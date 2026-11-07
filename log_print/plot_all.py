# -*- coding: utf-8 -*-
r"""一键绘图入口。

用法
----
    .\.venv\Scripts\python.exe plot_all.py                 # 生成全部 54 张图
    .\.venv\Scripts\python.exe plot_all.py --only 1,2,3    # 只画指定编号
    .\.venv\Scripts\python.exe plot_all.py --refresh       # 忽略缓存重新解析日志
    .\.venv\Scripts\python.exe plot_all.py --list          # 列出图目录
    .\.venv\Scripts\python.exe plot_all.py --doctor        # 环境/数据自检

日志目录默认是本文件旁的 log\，可用 --log-dir 或环境变量 LOG_PRINT_LOG_DIR 覆盖；
输出目录同理（--out-dir / LOG_PRINT_OUT_DIR）。程序内不写死任何绝对路径。
"""
from __future__ import annotations

import argparse
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

# 输出重定向到文件时，编码容纳不了的字符直接替换，不要因 UnicodeEncodeError 中断出图。
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(errors="replace")
    except Exception:
        pass

from viz import common, extract                      # noqa: E402
from viz import (figs_antenna, figs_geo, figs_ir, figs_ir_pro, figs_radar,
                 figs_radar_pro, figs_sat_pro, figs_signal)  # noqa: E402

MODULES = [figs_antenna, figs_radar, figs_ir, figs_signal, figs_radar_pro,
           figs_ir_pro, figs_sat_pro, figs_geo]

# 出图真正需要的数据文件。Rir_scan_pattern.csv 不在其列：图 3 的搜索驻留自带
# seq/az/el，波位表只作为换数据时的一致性核对基准（extract.scan_pattern()）。
DATA_FILES = [f for f, _ in extract.SCANNERS.values()] + [
    "RadarAntennaGainPatterndata.csv",
]


def registry():
    rows = []
    for m in MODULES:
        rows.extend(m.FIGURES)
    rows.sort(key=lambda r: int(r[0]))
    return rows


def doctor() -> int:
    """安装/换数据后的一键自检：依赖、字体、日志、目录权限。"""
    print("== 环境自检 ==")
    print(f"  Python   : {sys.version.split()[0]}  ({sys.executable})")
    print(f"  程序目录 : {common.APP}")
    print(f"  日志目录 : {common.LOG_DIR}")
    print(f"  输出目录 : {common.OUT_DIR}")
    print(f"  缓存目录 : {common.CACHE_DIR}")

    bad = 0
    try:
        import matplotlib
        import numpy
        import pandas
        import scipy
        print(f"  依赖     : numpy {numpy.__version__} / pandas {pandas.__version__} "
              f"/ matplotlib {matplotlib.__version__} / scipy {scipy.__version__}")
    except Exception as exc:                       # noqa: BLE001
        print(f"  依赖     : 导入失败 -> {exc}")
        bad += 1

    font = common.setup_style()
    if common.HAS_CJK_FONT:
        print(f"  中文字体 : {font}")
    else:
        print(f"  中文字体 : 未找到中文字体（已退化为 {font}），图上的中文会显示为方框")
        bad += 1

    print("  数据文件 :")
    missing = 0
    for fname in DATA_FILES:
        try:
            p = extract.resolve_log(fname)
        except FileNotFoundError:
            print(f"    [缺失] {fname}")
            missing += 1
            continue
        mb = p.stat().st_size / 1024 / 1024
        enc = extract.detect_encoding(p)
        note = "" if p.parent == Path(common.LOG_DIR) else f"  <- {p.parent}"
        print(f"    [OK]   {fname:<34} {mb:8.1f} MB  {enc}{note}")
    bad += missing

    if not common.writable(common.OUT_DIR):
        print(f"  输出目录 : 不可写 -> {common.OUT_DIR}")
        bad += 1
    if not common.writable(common.CACHE_DIR):
        print(f"  缓存目录 : 不可写 -> {common.CACHE_DIR}")
        bad += 1

    print("== 自检通过 ==" if not bad else f"== 自检发现 {bad} 项问题 ==")
    return 0 if not bad else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="雷达/红外仿真日志一键绘图")
    ap.add_argument("--only", default="", help="只画指定编号，如 1,3,7 或 1-5")
    ap.add_argument("--refresh", action="store_true", help="忽略解析缓存")
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--list", action="store_true", help="仅列出图目录")
    ap.add_argument("--doctor", action="store_true", help="环境与数据自检后退出")
    ap.add_argument("--log-dir", default="", help="日志目录（默认本目录下 log\\）")
    ap.add_argument("--out-dir", default="", help="出图目录（默认本目录下 figures\\）")
    ap.add_argument("--cache-dir", default="", help="解析缓存目录（默认 viz\\.cache\\）")
    args = ap.parse_args(argv)

    common.set_paths(log_dir=args.log_dir, out_dir=args.out_dir, cache_dir=args.cache_dir)

    if args.doctor:
        return doctor()

    font = common.setup_style()
    if not common.HAS_CJK_FONT:
        print("警告：未找到中文字体，图上的中文会显示为方框；请安装 Microsoft YaHei / SimHei 后重试。")

    rows = registry()

    if args.list:
        for num, slug, title, _ in rows:
            print(f"  {num:>3s}  {title:<28s}  {slug}.png")
        print(f"\n共 {len(rows)} 张。")
        return 0

    want = None
    if args.only.strip():
        want = set()
        for tok in args.only.replace("，", ",").split(","):
            tok = tok.strip()
            if not tok:
                continue
            if "-" in tok:
                a, b = tok.split("-", 1)
                want.update(range(int(a), int(b) + 1))
            else:
                want.add(int(tok))

    todo = [r for r in rows if want is None or int(r[0]) in want]
    if not todo:
        print("没有匹配的图编号。")
        return 2

    if args.refresh:
        cache = common.CACHE_DIR
        if cache.exists():
            for p in cache.glob("*.pkl"):
                p.unlink()
        print("已清除解析缓存。")

    print(f"日志目录：{common.LOG_DIR}  (字体 {font})")
    print(f"准备生成 {len(todo)} 张图 → {common.OUT_DIR}")
    t_all = time.time()
    ok, bad = [], []
    for num, slug, title, fn in todo:
        t0 = time.time()
        try:
            data = {
                "radar_output": extract.load("radar_output", verbose=True),
                "radar_ext": extract.load("radar_ext", verbose=True),
                "radar_int": extract.load("radar_int", verbose=True),
                "ir_int": extract.load("ir_int", verbose=True),
                "ir_ext": extract.load("ir_ext", verbose=True),
                "sat": extract.load("sat", verbose=True),
            }
            fig = fn(data)
            path = common.save(fig, slug, dpi=args.dpi)
            size = path.stat().st_size / 1024
            print(f"  [{num}] {title:<28s} {time.time() - t0:5.1f}s  {size:7.0f} KB")
            ok.append(path)
        except Exception:
            print(f"  [{num}] {title:<28s} 失败")
            traceback.print_exc()
            bad.append(num)

    dt = time.time() - t_all
    print(f"\n完成 {len(ok)} 张，失败 {len(bad)} 张，总耗时 {dt:.1f}s")
    if bad:
        print("失败编号：" + ", ".join(bad))
        return 1
    print(f"输出目录：{common.OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
