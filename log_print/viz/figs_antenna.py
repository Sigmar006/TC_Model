# -*- coding: utf-8 -*-
"""图 1–3：天线方向图与波束扫描调度。"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
from scipy.interpolate import RectBivariateSpline

from . import extract
from .common import C, db10, empty_fig, style_axes, title_block

SIDE = ["top", "right"]


# ---------------------------------------------------------------- 公共
def _pattern():
    """返回 az∈[-180,180)、el∈[-90,90]、增益 [n_az, n_el]。"""
    az, el, g = extract.antenna_pattern()
    az = az.astype(float)
    el = el.astype(float)
    g = g.astype(float)
    if az.max() > 180.0:                      # 0..360 → -180..180
        az = (az + 180.0) % 360.0 - 180.0
        order = np.argsort(az, kind="stable")
        az = az[order]
        g = g[order, :]
        keep = np.concatenate(([True], np.diff(az) > 1e-9))   # 去掉 0/360 重复点
        az = az[keep]
        g = g[keep, :]
    return az, el, g


def _sub(az, el, g, a_lo, a_hi, e_lo, e_hi):
    ia = np.where((az >= a_lo) & (az <= a_hi))[0]
    ie = np.where((el >= e_lo) & (el <= e_hi))[0]
    return az[ia], el[ie], g[np.ix_(ia, ie)]


def _hpbw(axis, cut, drop=3.0):
    """-3 dB 宽度（线性插值到门限）。"""
    peak = float(np.nanmax(cut))
    thr = peak - drop
    idx = np.where(cut >= thr)[0]
    if idx.size == 0:
        return np.nan
    i0, i1 = int(idx[0]), int(idx[-1])
    x0 = axis[i0]
    if i0 > 0 and cut[i0] != cut[i0 - 1]:
        f = (thr - cut[i0 - 1]) / (cut[i0] - cut[i0 - 1])
        x0 = axis[i0 - 1] + f * (axis[i0] - axis[i0 - 1])
    x1 = axis[i1]
    if i1 < cut.size - 1 and cut[i1] != cut[i1 + 1]:
        f = (thr - cut[i1]) / (cut[i1 + 1] - cut[i1])
        x1 = axis[i1] + f * (axis[i1 + 1] - axis[i1])
    return abs(x1 - x0)


# ---------------------------------------------------------------- 图 1
def fig01(D):
    az, el, g = _pattern()
    peak = float(np.nanmax(g))

    fig = plt.figure(figsize=(13.4, 5.8))
    title_block(fig, "图 1  天线增益方向图（三维）",
                f"峰值增益 {peak:.2f} dBi；RadarAntennaGainPatterndata.csv，"
                f"{az.size}×{el.size} 网格（0.1°）")

    # ---------------- (a) 主瓣（轻度平滑后插值到 0.02°）
    a1, e1, g1 = _sub(az, el, g, -1.8, 1.8, -1.8, 1.8)
    from scipy.ndimage import gaussian_filter
    g1 = gaussian_filter(g1, sigma=2.0, mode="nearest")
    spl = RectBivariateSpline(a1, e1, g1, kx=3, ky=3, s=0)
    a_f = np.linspace(a1[0], a1[-1], 181)
    e_f = np.linspace(e1[0], e1[-1], 181)
    G_f = spl(a_f, e_f)
    A, E = np.meshgrid(a_f, e_f, indexing="ij")

    ax = fig.add_subplot(1, 2, 1, projection="3d")
    surf = ax.plot_surface(A, E, G_f, cmap="viridis", rstride=2, cstride=2,
                           linewidth=0, antialiased=True, alpha=0.98)
    ax.set_xlabel("方位偏角 (°)", labelpad=4)
    ax.set_ylabel("俯仰偏角 (°)", labelpad=4)
    ax.set_zlabel("增益 (dBi)", labelpad=4)
    ax.set_title("(a) 主瓣立体形貌（±1.8°）", fontsize=10.5, pad=0)
    ax.view_init(elev=26, azim=-125)
    ax.tick_params(labelsize=7)
    cb = fig.colorbar(surf, ax=ax, shrink=0.55, pad=0.10)
    cb.set_label("增益 (dBi)", fontsize=8)
    cb.ax.tick_params(labelsize=7)

    # ---------------- (b) 全空域（球坐标半径 = 归一化增益）
    S = 10
    na = (az.size // S) * S
    ne = (el.size // S) * S
    g2 = g[:na, :ne].reshape(na // S, S, ne // S, S).max(axis=(1, 3))
    az2 = np.radians(az[:na].reshape(-1, S).mean(axis=1))
    el2 = np.radians(el[:ne].reshape(-1, S).mean(axis=1))
    AA, EE = np.meshgrid(az2, el2, indexing="ij")
    r = np.sqrt(np.clip(g2, 0.0, None) / peak)
    X = r * np.cos(EE) * np.cos(AA)
    Y = r * np.cos(EE) * np.sin(AA)
    Z = r * np.sin(EE)
    norm = Normalize(vmin=float(np.nanmin(g2)), vmax=peak)
    colors = cm.viridis(norm(g2))
    X = np.vstack([X, X[:1, :]])            # 闭合方位接缝
    Y = np.vstack([Y, Y[:1, :]])
    Z = np.vstack([Z, Z[:1, :]])
    colors = np.vstack([colors, colors[:1, :]])

    ax = fig.add_subplot(1, 2, 2, projection="3d")
    ax.plot_surface(X, Y, Z, facecolors=colors, rstride=1, cstride=1,
                    linewidth=0, antialiased=False, shade=False)
    ax.set_axis_off()
    ax.set_title("(b) 全空域波瓣（半径 ∝ √增益）", fontsize=10.5, pad=0)
    ax.view_init(elev=24, azim=-60)
    ax.set_box_aspect((1, 1, 0.75), zoom=1.22)
    sm = cm.ScalarMappable(norm=norm, cmap="viridis")
    cb = fig.colorbar(sm, ax=ax, shrink=0.55, pad=0.02)
    cb.set_label("增益 (dBi)", fontsize=8)
    cb.ax.tick_params(labelsize=7)

    ax.text2D(0.02, 0.02, f"波束峰值 {peak:.1f} dBi，副瓣/背瓣约 8–17 dBi",
              transform=ax.transAxes, fontsize=8.2, color="#5b6773")
    return fig


# ---------------------------------------------------------------- 图 2
def fig02(D):
    az, el, g = _pattern()
    peak = float(np.nanmax(g))
    i_az0 = int(np.argmin(np.abs(az)))
    i_el0 = int(np.argmin(np.abs(el)))
    cut_az = g[:, i_el0]
    cut_el = g[i_az0, :]

    fig, axes = plt.subplots(2, 2, figsize=(12.6, 7.8))
    title_block(fig, "图 2  天线方向图二维剖面与主瓣宽度",
                f"峰值 {peak:.2f} dBi；方位剖面取俯仰 0°，俯仰剖面取方位 0°")

    ax = axes[0, 0]
    ax.plot(az, cut_az, color=C["accent1"], lw=1.0)
    ax.axhline(peak - 3, color=C["warn"], lw=0.9, ls="--", label="-3 dB")
    ax.axhline(peak - 10, color=C["accent4"], lw=0.9, ls=":", label="-10 dB")
    ax.set_xlabel("方位偏角 (°)")
    ax.set_ylabel("增益 (dBi)")
    ax.set_title("(a) 方位剖面（全周）", fontsize=10.5)
    ax.set_xlim(-180, 180)
    ax.legend(loc="upper right", ncol=2, fontsize=8)
    style_axes(ax)

    ax = axes[0, 1]
    ax.plot(el, cut_el, color=C["accent2"], lw=1.0)
    ax.axhline(peak - 3, color=C["warn"], lw=0.9, ls="--", label="-3 dB")
    ax.axhline(peak - 10, color=C["accent4"], lw=0.9, ls=":", label="-10 dB")
    ax.set_xlabel("俯仰偏角 (°)")
    ax.set_ylabel("增益 (dBi)")
    ax.set_title("(b) 俯仰剖面（全周）", fontsize=10.5)
    ax.set_xlim(-90, 90)
    ax.legend(loc="upper right", ncol=2, fontsize=8)
    style_axes(ax)

    # (c) 主瓣等高线
    a1, e1, g1 = _sub(az, el, g, -2.0, 2.0, -2.0, 2.0)
    A, E = np.meshgrid(a1, e1, indexing="ij")
    ax = axes[1, 0]
    cf = ax.contourf(A, E, g1, levels=16, cmap="viridis")
    lv = [peak - 20, peak - 10, peak - 6, peak - 3]
    cs = ax.contour(A, E, g1, levels=lv,
                    colors=["#c0392b", "#e07b39", "#f6d365", "#ffffff"],
                    linewidths=[1.0, 1.0, 1.0, 1.8])
    ax.clabel(cs, fmt=lambda v: f"{v - peak:.0f} dB", fontsize=7.5)
    cb = fig.colorbar(cf, ax=ax, shrink=0.92, pad=0.02)
    cb.set_label("增益 (dBi)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("方位偏角 (°)")
    ax.set_ylabel("俯仰偏角 (°)")
    ax.set_title("(c) 主瓣等高线（相对峰值）", fontsize=10.5)
    style_axes(ax)

    # (d) 归一化剖面 + HPBW
    hp_az = _hpbw(az, cut_az)
    hp_el = _hpbw(el, cut_el)
    ax = axes[1, 1]
    m = np.abs(az) <= 2.5
    ax.plot(az[m], cut_az[m] - peak, color=C["accent1"], lw=1.4, label="方位剖面")
    m = np.abs(el) <= 2.5
    ax.plot(el[m], cut_el[m] - peak, color=C["accent2"], lw=1.4, label="俯仰剖面")
    ax.axhline(-3, color=C["warn"], lw=1.0, ls="--")
    ax.axvline(-hp_az / 2, color=C["neutral"], lw=0.8, ls=":")
    ax.axvline(hp_az / 2, color=C["neutral"], lw=0.8, ls=":")
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-30, 1)
    ax.set_xlabel("偏角 (°)")
    ax.set_ylabel("相对增益 (dB)")
    ax.set_title("(d) 主瓣归一化剖面", fontsize=10.5)
    ax.annotate(f"-3 dB 波束宽度\n方位 {hp_az:.2f}°  /  俯仰 {hp_el:.2f}°",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=9,
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 3
def fig03(D):
    d = extract.load("radar_output", verbose=False)
    beam = d.get("beam")
    if beam is None or len(beam) == 0:
        return empty_fig("图 3 波束扫描与驻留调度",
                         "本批日志无「波束扫描功能测试」记录，本图从略")
    bs = d.get("beam_sum")

    fig, axes = plt.subplots(1, 3, figsize=(15.2, 4.9))
    title_block(fig, "图 3  波束扫描与驻留调度",
                f"共 {len(beam)} 条驻留记录：搜索 {int((beam['kind'] == '搜索').sum())}，"
                f"跟随驻留 {int((beam['kind'] == '驻留').sum())}，"
                f"指定驻留 {int((beam['kind'] == '指定驻留').sum())}")

    ax = axes[0]
    # 驻留密度用 dB 表示（相对 1 次）：色标仍是“越密越亮”，但刻度可读。
    hb = ax.hexbin(beam["az"], beam["el"], gridsize=(72, 32), cmap="magma",
                   mincnt=1)
    cnt_db = db10(np.maximum(np.asarray(hb.get_array(), dtype=float), 1.0))
    hb.set_array(cnt_db)
    hb.set_clim(float(np.nanmin(cnt_db)), float(np.nanmax(cnt_db)))
    cb = fig.colorbar(hb, ax=ax, pad=0.02)
    cb.set_label("驻留次数 (dB，相对 1 次)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("波束方位 (°)")
    ax.set_ylabel("波束俯仰 (°)")
    ax.set_title("(a) 空域驻留密度", fontsize=10.5)
    style_axes(ax)

    ax = axes[1]
    for kind, col in (("搜索", C["accent1"]), ("驻留", C["accent3"]),
                      ("指定驻留", C["accent2"])):
        s = beam[beam["kind"] == kind].groupby("cyc").size()
        ax.plot(s.index, s.values, lw=0.9, color=col, label=kind)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("每周期驻留数")
    ax.set_title("(b) 每周期驻留数构成", fontsize=10.5)
    ax.set_xlim(0, 3200)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[2]
    srch = beam[beam["kind"] == "搜索"].dropna(subset=["seq"])
    sc = ax.scatter(srch["t"], srch["seq"], s=0.5, c=srch["el"],
                    cmap="viridis", alpha=0.30, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("俯仰 (°)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("波位序号")
    ax.set_title("(c) 搜索波位推进（方位优先）", fontsize=10.5)
    ax.set_xlim(0, 2400)
    style_axes(ax)

    ax2 = ax.inset_axes([0.03, 0.62, 0.38, 0.33])
    if bs is not None and len(bs):
        bs = bs.sort_values("t")
        ax2.plot(bs["t"], bs["n"], color=C["neutral"], lw=0.8)
    ax2.set_xlim(0, 2400)
    ax2.set_title("周期搜索驻留数", fontsize=7, pad=1.5)
    ax2.tick_params(labelsize=6)
    style_axes(ax2, minor=False)
    return fig


FIGURES = [
    ("01", "fig01_antenna_3d", "图 1 天线增益方向图（三维）", fig01),
    ("02", "fig02_antenna_cuts", "图 2 天线方向图二维剖面", fig02),
    ("03", "fig03_beam_scan", "图 3 波束扫描与驻留调度", fig03),
]
