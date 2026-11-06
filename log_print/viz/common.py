# -*- coding: utf-8 -*-
"""公共样式、路径与坐标换算工具。"""
from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator, AutoMinorLocator

# ---------------------------------------------------------------- 路径
# 默认全部相对本程序定位，整包拷到别处（含无盘符的映射盘/长路径）也能跑；
# 内网日志目录不同时用 --log-dir 或环境变量覆盖，不写死任何绝对路径。
ROOT = Path(__file__).resolve().parent          # .../log_print/viz
APP = ROOT.parent                               # .../log_print


def _env_path(*names: str) -> Path | None:
    for n in names:
        v = os.environ.get(n, "").strip().strip('"')
        if v:
            return Path(v).expanduser().resolve()
    return None


def writable(d: Path) -> bool:
    """目录可用＝能创建、能写文件。"""
    try:
        d.mkdir(parents=True, exist_ok=True)
        probe = d / f".probe_{os.getpid()}"
        probe.write_bytes(b"")
        probe.unlink()
        return True
    except OSError:
        return False


def _default_cache() -> Path:
    """解析缓存默认放程序目录；程序目录只读（刻盘/受控目录）时退化到临时目录。"""
    want = _env_path("LOG_PRINT_CACHE_DIR") or (ROOT / ".cache")
    if writable(want):
        return want
    fallback = Path(tempfile.gettempdir()) / "log_print_cache"
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


LOG_DIR = _env_path("LOG_PRINT_LOG_DIR") or (APP / "log")
OUT_DIR = _env_path("LOG_PRINT_OUT_DIR") or (APP / "figures")
CACHE_DIR = _default_cache()


def set_paths(*, log_dir=None, out_dir=None, cache_dir=None) -> None:
    """命令行覆盖路径；必须在读日志/存图之前调用。"""
    global LOG_DIR, OUT_DIR, CACHE_DIR
    if log_dir:
        LOG_DIR = Path(log_dir).expanduser().resolve()
    if out_dir:
        OUT_DIR = Path(out_dir).expanduser().resolve()
    if cache_dir:
        CACHE_DIR = Path(cache_dir).expanduser().resolve()

# ---------------------------------------------------------------- 配色
C = {
    "truth":   "#1f4e9c",   # 真值
    "meas":    "#e07b39",   # 量测
    "filter":  "#2e8b57",   # 滤波
    "pred":    "#8e44ad",   # 预测
    "warn":    "#c0392b",   # 告警 / 失效
    "ok":      "#2e8b57",
    "neutral": "#7f8c8d",
    "grid":    "#d9dde2",
    "band":    "#eaf2f8",
    "accent1": "#2c7fb8",
    "accent2": "#d95f0e",
    "accent3": "#31a354",
    "accent4": "#756bb1",
    "accent5": "#bcbd22",
}

CYCLE = ["#2c7fb8", "#d95f0e", "#31a354", "#756bb1", "#bcbd22",
         "#e377c2", "#17becf", "#8c564b", "#7f7f7f", "#c0392b"]

# ---------------------------------------------------------------- 时间窗
W_RADAR = (1240.0, 2870.0)      # 雷达跟踪 / 特征段
W_RADAR_LONG = (600.0, 3200.0)  # 含前置处理
W_IR = (0.0, 2950.0)            # 红外跟踪段
W_ALL = (0.0, 7000.0)           # 全时段

# 雷达站址（ENU 原点）
SITE_LAT = 64.2906
SITE_LON = -149.1903
SITE_H = 0.0


# ---------------------------------------------------------------- 字体
# 按可用性挑一个中文字体；内网机器若一个中文字体都没有，图上的中文会变成方框，
# 此时启动阶段给出明确提示，而不是到出图才发现。
CJK_FONTS = ("Microsoft YaHei", "Microsoft YaHei UI", "SimHei", "Noto Sans SC",
             "Source Han Sans SC", "WenQuanYi Zen Hei", "SimSun", "Arial Unicode MS")

FONT = "DejaVu Sans"
HAS_CJK_FONT = False


def resolve_font() -> tuple[str, bool]:
    """返回 (字体名, 是否找到中文字体)。"""
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in CJK_FONTS:
        if name in have:
            return name, True
    return "DejaVu Sans", False


# ---------------------------------------------------------------- 样式
def setup_style() -> str:
    global FONT, HAS_CJK_FONT
    FONT, HAS_CJK_FONT = resolve_font()
    plt.rcParams.update({
        "font.sans-serif": [FONT, "DejaVu Sans"],
        "font.family": "sans-serif",
        "axes.unicode_minus": False,
        "font.size": 10.0,
        "axes.titlesize": 12.0,
        "axes.titleweight": "bold",
        "axes.titlepad": 9.0,
        "axes.labelsize": 10.0,
        "axes.labelpad": 4.0,
        "axes.edgecolor": "#4a4f55",
        "axes.linewidth": 0.8,
        "axes.facecolor": "white",
        "axes.grid": True,
        "axes.axisbelow": True,
        "axes.prop_cycle": matplotlib.cycler(color=CYCLE),
        "grid.color": C["grid"],
        "grid.linewidth": 0.6,
        "grid.alpha": 0.9,
        "legend.frameon": False,
        "legend.fontsize": 9.0,
        "legend.handlelength": 2.2,
        "xtick.labelsize": 9.0,
        "ytick.labelsize": 9.0,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "xtick.major.size": 3.5,
        "ytick.major.size": 3.5,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.12,
    })
    return FONT


def style_axes(ax, *, minor: bool = True) -> None:
    """统一的坐标轴细节：四边细框，只保留左/下刻度。"""
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.spines["left"].set_color("#4a4f55")
    ax.spines["bottom"].set_color("#4a4f55")
    if minor:
        if ax.get_xscale() != "log":
            ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        if ax.get_yscale() != "log":
            ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.tick_params(which="minor", length=2.0, color="#9aa3ab")


def panel_label(ax, text: str, *, dx: float = -0.085, dy: float = 1.06) -> None:
    ax.text(dx, dy, text, transform=ax.transAxes, fontsize=11.5,
            fontweight="bold", va="bottom", ha="left", color="#2c3e50")


def title_block(fig, title: str, subtitle: str = "") -> None:
    fig.suptitle(title, fontsize=14.5, fontweight="bold", color="#1b2733",
                 x=0.5, y=0.975, ha="center")
    fig._has_sub = bool(subtitle)
    if subtitle:
        fig.text(0.5, 0.933, subtitle, fontsize=9.2, color="#5b6773",
                 ha="center", va="top")


def finish(fig) -> None:
    """统一留出标题区，然后收缩布局。"""
    if getattr(fig, "_finished", False):
        return
    top = 0.895 if getattr(fig, "_has_sub", False) else 0.935
    try:
        fig.tight_layout(rect=(0.008, 0.006, 0.992, top))
    except Exception:
        pass
    fig._finished = True


def save(fig, name: str, *, out_dir: Path | None = None, dpi: int = 200) -> Path:
    finish(fig)
    out = Path(out_dir) if out_dir else OUT_DIR
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{name}.png"
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path


def empty_fig(title: str, msg: str):
    fig, ax = plt.subplots(figsize=(8, 4.2), constrained_layout=True)
    ax.axis("off")
    ax.text(0.5, 0.55, title, ha="center", va="center", fontsize=13, fontweight="bold")
    ax.text(0.5, 0.40, msg, ha="center", va="center", fontsize=10.5, color=C["neutral"])
    return fig


# ---------------------------------------------------------------- 数值解析
_NUM = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"


def fnum(s):
    """把 '不适用' / 'miss' / 空串 等转成 nan。"""
    if s is None:
        return np.nan
    if isinstance(s, (int, float, np.floating, np.integer)):
        return float(s)
    s = s.strip()
    if not s or s in ("不适用", "miss", "none", "None", "-"):
        return np.nan
    try:
        return float(s)
    except ValueError:
        return np.nan


def parse_floats(s: str):
    """从 '(-1.0,2.0,3.0)' 或 '[a;b;c]' 中提取全部浮点数。"""
    if s is None:
        return np.array([])
    import re
    out = []
    for tok in re.split(r"[;,\s]+", s.strip().strip("[]()（）")):
        v = fnum(tok)
        if not math.isnan(v):
            out.append(v)
    return np.asarray(out, dtype=float)


def to_num(series):
    """把一列可能含 'miss'/'不适用' 的对象转成 float ndarray。"""
    return np.array([fnum(v) for v in series], dtype=float)


# ---------------------------------------------------------------- 坐标换算
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def enu_to_ecef(e, n, u, lat0=SITE_LAT, lon0=SITE_LON, h0=SITE_H):
    """站心 ENU → ECEF（WGS84）。输入可为数组。"""
    e = np.asarray(e, dtype=float)
    n = np.asarray(n, dtype=float)
    u = np.asarray(u, dtype=float)
    phi = math.radians(lat0)
    lam = math.radians(lon0)
    sp, cp = math.sin(phi), math.cos(phi)
    sl, cl = math.sin(lam), math.cos(lam)
    N = _A / math.sqrt(1.0 - _E2 * sp * sp)
    x0 = (N + h0) * cp * cl
    y0 = (N + h0) * cp * sl
    z0 = (N * (1.0 - _E2) + h0) * sp
    x = x0 - sl * e - sp * cl * n + cp * cl * u
    y = y0 + cl * e - sp * sl * n + cp * sl * u
    z = z0 + cp * n + sp * u
    return x, y, z


def ecef_to_lla(x, y, z):
    """ECEF → (lat°, lon°, h m)，迭代解算。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    z = np.asarray(z, dtype=float)
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - _E2))
    for _ in range(8):
        N = _A / np.sqrt(1.0 - _E2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1.0 - _E2 * N / (N + h)))
    N = _A / np.sqrt(1.0 - _E2 * np.sin(lat) ** 2)
    h = p / np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), h


def enu_to_lla(e, n, u):
    x, y, z = enu_to_ecef(e, n, u)
    return ecef_to_lla(x, y, z)


# ---------------------------------------------------------------- 杂项
def dt_window(ax, t0, t1):
    if t0 is not None:
        ax.set_xlim(t0, t1)


def shade(ax, t0, t1, color="#f2f5f7", alpha=0.8, label=None):
    span = ax.axvspan(t0, t1, color=color, alpha=alpha, lw=0, zorder=0, label=label)
    return span


def light_grid(ax):
    ax.grid(True, which="major", color=C["grid"], lw=0.6)
    ax.grid(True, which="minor", color=C["grid"], lw=0.35, alpha=0.6)
