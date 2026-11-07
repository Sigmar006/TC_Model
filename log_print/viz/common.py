# -*- coding: utf-8 -*-
"""公共样式、路径与坐标换算工具。"""
from __future__ import annotations

import functools
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


def finish(fig, *, bottom: float = 0.006) -> None:
    """统一留出标题区，然后收缩布局；bottom 给图脚注留白。"""
    if getattr(fig, "_finished", False):
        return
    top = 0.895 if getattr(fig, "_has_sub", False) else 0.935
    try:
        fig.tight_layout(rect=(0.008, bottom, 0.992, top))
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


# ---------------------------------------------------------------- dB 换算
# 图上的坐标一律用 dB 表达，不再出现 10^n 对数刻度：
#   功率 / 功率谱 / 比值 / 计数 / 时间间隔 / 方差  → 10·lg（dBW、dB、dB(m²)…）
#   长度 / 位置误差                              → 20·lg（dB，相对 1 m）
# 非正值在对数域无定义，直接返回 NaN，由 matplotlib 跳点、统计时自然被排除。
def db10(x):
    """线性功率（或比值/计数/间隔/方差）→ dB。"""
    a = np.asarray(x, dtype=float)
    out = np.full(a.shape, np.nan, dtype=float)
    ok = np.isfinite(a) & (a > 0.0)
    out[ok] = 10.0 * np.log10(a[ok])
    return out


def db20(x):
    """长度（或幅度）→ dB，参考 1 m。"""
    a = np.asarray(x, dtype=float)
    out = np.full(a.shape, np.nan, dtype=float)
    ok = np.isfinite(a) & (a > 0.0)
    out[ok] = 20.0 * np.log10(a[ok])
    return out


# ---------------------------------------------------------------- 观测窗裁剪
# 滤波/航迹只画到该实体最后一次观测：观测停止后的外推段不是量测事实，默认不画。
# 一键开关：plot_all.py --show-extrapolated（或 common.set_clip_to_obs(False)）。
CLIP_TO_OBS = True


def set_clip_to_obs(flag: bool) -> None:
    """统一开关：False 时保留滤波外推段（--show-extrapolated）。"""
    global CLIP_TO_OBS
    CLIP_TO_OBS = bool(flag)


def obs_span(t) -> tuple[float, float]:
    """观测时刻 → (首, 末)；没有有效时刻时返回 (nan, nan)。"""
    a = np.asarray(t, dtype=float)
    a = a[np.isfinite(a)]
    if not a.size:
        return float("nan"), float("nan")
    return float(a.min()), float(a.max())


def obs_mask(t, lo, hi):
    """按观测窗返回保留掩码；开关关闭或窗口无效时全保留。"""
    t = np.asarray(t, dtype=float)
    if not CLIP_TO_OBS or not (np.isfinite(lo) and np.isfinite(hi)):
        return np.ones(t.shape, dtype=bool)
    return (t >= lo) & (t <= hi)


def obs_clip(t, lo, hi):
    """返回 (保留掩码, 被剔除点数)；开关关闭时剔除数为 0。"""
    m = obs_mask(t, lo, hi)
    return m, int((~m).sum())


def obs_note(n_cut: int, hi: float = float("nan"), head: str = "滤波") -> str:
    """图内一句裁剪说明；关闭裁剪时明确写出，避免读者误以为没有外推段。"""
    if not CLIP_TO_OBS:
        return f"{head}含外推段（观测窗裁剪已关闭）"
    if n_cut:
        return f"{head}只画到观测窗末 {hi:.0f} s（剔除外推 {n_cut} 点）"
    return ""


def radar_obs_span(D, tid):
    """雷达目标观测窗：点迹输出（radar_ext.plot）或关联量测（meas_trk）。"""
    for mod, tab in (("radar_ext", "plot"), ("radar_int", "meas_trk")):
        df = (D.get(mod) or {}).get(tab)
        if df is None or len(df) == 0 or "tid" not in df.columns:
            continue
        m = to_num(df["tid"]) == float(tid)
        if m.any():
            return obs_span(to_num(df["t"])[m])
    return float("nan"), float("nan")


def ir_obs_span(D, *, tid=None, sat=None):
    """红外观测窗：ir_ext.ir_obs 中 (卫星, 目标) 的时刻范围。"""
    df = (D.get("ir_ext") or {}).get("ir_obs")
    if df is None or len(df) == 0 or "t" not in df.columns:
        return float("nan"), float("nan")
    m = np.ones(len(df), dtype=bool)
    if tid is not None and "tid" in df.columns:
        m &= to_num(df["tid"]) == float(tid)
    if sat is not None and "sat" in df.columns:
        m &= to_num(df["sat"]) == float(sat)
    return obs_span(to_num(df["t"])[m])


def all_obs_span(D):
    """全批观测并集：给没有实体键的滤波表（sat.ukf）兜底。"""
    spans = []
    for mod, tab in (("radar_ext", "plot"), ("ir_ext", "ir_obs")):
        df = (D.get(mod) or {}).get(tab)
        if df is not None and len(df) and "t" in df.columns:
            spans.append(obs_span(to_num(df["t"])))
    spans = [s for s in spans if np.isfinite(s[0])]
    if not spans:
        return float("nan"), float("nan")
    return min(s[0] for s in spans), max(s[1] for s in spans)


# ---------------------------------------------------------------- 缺表降级
def needs(*specs, title: str = ""):
    """装饰器：本图依赖的日志表缺失时改画一张说明卡，而不是抛 KeyError。

    specs 每项为 (模块键, 表名, 日志消息名)，模块键取 radar_output / radar_ext /
    radar_int / ir_int / ir_ext / sat。任一表缺失或为空即判定本批数据不足。
    """
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(D, *args, **kwargs):
            _scan_entity_universe(D)
            for mod, tab, msg in specs:
                df = (D.get(mod) or {}).get(tab)
                if df is None or len(df) == 0:
                    name = title or getattr(fn, "__name__", "图")
                    return empty_fig(name, f"本批日志无「{msg}」记录，本图从略")
            return fn(D, *args, **kwargs)
        return wrapper
    return deco


# ---------------------------------------------------------------- 分目标
# 跟踪与特征图一律按目标分开：一行（或一组曲线）只画一个目标，标题与图例
# 直接写出目标号，避免把不同目标的量测、误差、特征混在一起统计。
def by_target(df, col: str = "tid"):
    """按目标列分组，记录多的目标排在前面。

    返回 [(目标号, 子表)]；同一目标的多条航迹留在同一组里，
    由调用方决定是分行画还是用线型区分。数据里没有该列时退回单组 (None, df)。
    """
    if df is None or len(df) == 0 or col not in df.columns:
        return [(None, df)]
    vals = df[col]
    ids = sorted({int(v) for v in vals.dropna().unique()})
    groups = [(tid, df[vals == float(tid)]) for tid in ids]
    groups.sort(key=lambda kv: (-len(kv[1]), kv[0]))
    return groups


def target_title(tid, tracks=None) -> str:
    """统一的目标标签：'目标 109' 或 '目标 109（航迹 1、2）'。"""
    if tid is None:
        return "全部目标"
    if tracks is None or len(tracks) == 0:
        return f"目标 {int(tid)}"
    tr = "、".join(str(int(v)) for v in sorted({int(v) for v in tracks}))
    return f"目标 {int(tid)}（航迹 {tr}）"


# ---------------------------------------------------------------- 实体配色
# 同一目标 / 卫星在多张图里保持同一颜色，跨图对照时不用重新认图例。
LS_CYCLE = ["-", "--", "-.", ":", (0, (5, 1.4)), (0, (1, 1.1)),
            (0, (4, 1.2, 1, 1.2)), (0, (6, 1.4, 1, 1.4, 1, 1.4))]
MARKERS = ["o", "s", "^", "D", "v", "P", "X", "<", ">", "*"]
# 基本色板只有 10 个；本批实体号 99–109 共 11 个，补 3 个深色备位。
ENTITY_CYCLE = CYCLE + ["#34495e", "#7f4f24", "#00a6a6"]

_UNIVERSE: list[int] = []
_UNIVERSE_SCANNED = False


def _entity_ids(ids) -> list[int]:
    """规范化实体号：只保留可转整数的值，去重后升序。"""
    out: list[int] = []
    for v in ids:
        try:
            iv = int(v)
        except (TypeError, ValueError):
            continue
        if iv not in out:
            out.append(iv)
    out.sort()
    return out


def color_map(ids) -> dict[int, str]:
    """实体号 → 固定颜色（按整批数据的编号次序取色板）。"""
    ids = _entity_ids(ids)
    order = _order(ids)
    return {v: ENTITY_CYCLE[order.index(v) % len(ENTITY_CYCLE)] for v in ids}


def ls_map(ids) -> dict[int, object]:
    """实体号 → 线型（只在本图内区分，按本图编号升序取）。"""
    return {v: LS_CYCLE[i % len(LS_CYCLE)]
            for i, v in enumerate(_entity_ids(ids))}


def marker_map(ids) -> dict[int, str]:
    """实体号 → 标记（只在本图内区分，按本图编号升序取）。"""
    return {v: MARKERS[i % len(MARKERS)]
            for i, v in enumerate(_entity_ids(ids))}


def _scan_entity_universe(D) -> None:
    """首次出图时扫描整批数据，固定实体号到颜色的次序（只做一次）。"""
    global _UNIVERSE, _UNIVERSE_SCANNED
    if _UNIVERSE_SCANNED:
        return
    _UNIVERSE_SCANNED = True
    vals: set[int] = set()
    try:
        import pandas as pd
    except Exception:                                   # noqa: BLE001
        pd = None
    for tag, mod in (D or {}).items():
        if not isinstance(mod, dict):
            continue
        # key 只在卫星模块里是目标键；雷达模块的 meas_trk.key 是航迹序号（1–4），
        # 混进来会把真实实体号的配色整体顶偏。
        cols_want = ("tid", "sat", "key") if tag == "sat" else ("tid", "sat")
        for df in mod.values():
            cols = getattr(df, "columns", None)
            if cols is None:
                continue
            for col in cols_want:
                if col not in cols:
                    continue
                if pd is not None:
                    v = pd.to_numeric(df[col], errors="coerce").dropna()
                    vals.update(int(x) for x in v.unique())
                else:
                    a = to_num(df[col])
                    vals.update(int(x) for x in a[np.isfinite(a)])
    if vals:
        _UNIVERSE = sorted(vals)


def _order(ids) -> list[int]:
    """实体号在该批数据中的固定次序；未见过的号追加在末尾。"""
    ids = _entity_ids(ids)
    if not _UNIVERSE:
        return ids
    return list(_UNIVERSE) + [v for v in ids if v not in _UNIVERSE]


def align_same_rows(df, ref, cols=("t", "cyc"), carry=("tid", "track")):
    """把 ref 的实体列按行序搬到 df 上（df 缺目标号时用）。

    仅当两表行数相同、且 cols 指定键列逐行一致时才复制，返回带 carry 列的
    新表；条件不满足返回 None，调用方应退回整批统计并在图内注明“无目标号”。
    """
    if df is None or ref is None or len(df) != len(ref) or len(df) == 0:
        return None
    keys = [c for c in cols if c in df.columns and c in ref.columns]
    if not keys:
        return None
    a = df.reset_index(drop=True)
    b = ref.reset_index(drop=True)
    for c in keys:
        x = to_num(a[c])
        y = to_num(b[c])
        if not np.array_equal(np.isnan(x), np.isnan(y)):
            return None
        m = ~np.isnan(x)
        if not np.allclose(x[m], y[m], rtol=0.0, atol=1e-9):
            return None
    out = df.copy()
    for c in carry:
        if c in b.columns:
            out[c] = b[c].to_numpy()
    return out


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
