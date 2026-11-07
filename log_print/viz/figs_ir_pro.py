# -*- coding: utf-8 -*-
"""图 36–39：红外辐射链路、大气透过率、跟踪一致性与宽场候选。

坐标口径
--------
* 信噪比、接收功率、能量一律换算成 dB 后线性绘制（10·lg），不使用对数坐标轴；
* 链路各阶段的记录量用 dB 表示，柱端标原始条数；
* NIS 与 χ²(2) 门限直接比较，超门比例按滚动窗口统计。

实体口径
--------
* 颜色＝卫星、标记 / 线型＝目标，与图 14 的约定一致，跨图同名实体同色；
* 每条曲线、每个直方分组只属于一个（卫星，目标）组合，不把多颗卫星混成一条。
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from scipy.stats import chi2

from .common import (C, color_map, db10, ls_map, marker_map, needs,
                     style_axes, target_title, title_block)


def _rng_txt(v, unit="", nd=2):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if not v.size:
        return "未记录"
    return f"{v.min():.{nd}f}–{v.max():.{nd}f}{unit}"


def _safe_db(x, floor=1e-30):
    return db10(np.clip(np.asarray(x, dtype=float), floor, None))


def _pairs(df):
    """按（卫星，目标）拆表；缺实体号时退回整批。"""
    if df is None or "sat" not in df.columns or "tid" not in df.columns:
        return [(None, None, df)]
    out = []
    for (s, t), g in df.groupby(["sat", "tid"], sort=True, dropna=True):
        out.append((int(s), int(t), g))
    return out


def _legend_sat(ax, sats, scmap, loc="best", ncol=1, fs=7.0, title=None):
    hs = [Line2D([], [], ls="none", marker="o", ms=5, color=scmap[s],
                 label=f"卫星 {s}") for s in sats]
    return ax.legend(handles=hs, loc=loc, ncol=ncol, fontsize=fs, title=title,
                     title_fontsize=fs, handletextpad=0.3, labelspacing=0.3)


def _pair_legend(ax, sats, tids, scmap, lmap, loc="upper center", ncol=2,
                 fs=6.2, extra=()):
    """直方图/曲线用：颜色＝卫星，线型＝目标；两组手柄合成一个图例。"""
    hs = [Line2D([], [], color=scmap[s], lw=1.8, label=f"卫星 {s}")
          for s in sats]
    hs += [Line2D([], [], color=C["neutral"], lw=1.4, ls=lmap[t],
                  label=f"目标 {t}") for t in tids]
    hs += list(extra)
    return ax.legend(handles=hs, loc=loc, ncol=ncol, fontsize=fs,
                     frameon=True, framealpha=0.92, edgecolor="#c8d0d8",
                     handlelength=1.8, handletextpad=0.35,
                     columnspacing=0.8, labelspacing=0.25)


# ---------------------------------------------------------------- 图 36
@needs(("ir_int", "ir_rad", "红外辐射噪声与检测判决交互接口测试"),
       title="图 36 红外辐射 SNR–距离与检测门限")
def fig36(D):
    rad = D["ir_int"]["ir_rad"]
    pairs = _pairs(rad)
    sats = sorted({s for s, _, _ in pairs})
    tids = sorted({t for _, t, _ in pairs})
    scmap = color_map(sats)
    tmark = marker_map(tids)
    lmap = ls_map(tids)
    snr_db = _safe_db(rad["snr"].to_numpy(float))
    rng = rad["rng"].to_numpy(float) / 1e3
    tau = rad["tau"].to_numpy(float)
    p_rx = _safe_db(rad["p_rx"].to_numpy(float))
    thr_lin = float(np.nanmedian(rad["thr"].to_numpy(float)))
    thr_db = float(db10(thr_lin))
    fov_ok = rad["fov_res"].astype(str).eq("通过").to_numpy()
    admit_ok = rad["admit"].astype(str).eq("通过").to_numpy()
    snr_ok = rad["snr_res"].astype(str).eq("通过").to_numpy()

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 36  红外辐射 SNR–距离、接收功率与检测门限（按卫星/目标）",
                f"辐射记录 {len(rad)} 条：卫星 {'、'.join(str(s) for s in sats)}，"
                f"目标 {'、'.join(str(t) for t in tids)}；"
                f"SNR {_rng_txt(snr_db, ' dB', 1)}；门限 {thr_lin:.0f}（≈{thr_db:.1f} dB）；"
                f"视场门通过 {int(fov_ok.sum())} 条；颜色＝卫星、标记＝目标")

    ax = axes[0, 0]
    for s, t, g in pairs:
        ax.scatter(g["rng"] / 1e3, _safe_db(g["snr"]), s=3, marker=tmark[t],
                   color=scmap[s], alpha=0.22, linewidths=0)
    ax.axhline(thr_db, color=C["warn"], ls="--", lw=1.1,
               label=f"SNR 门限 {thr_db:.1f} dB")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("SNR (dB)")
    ax.set_title("(a) SNR–距离密度与门限", fontsize=10.5)
    ax.annotate(f"最小余量 {float(np.min(snr_db - thr_db)):.1f} dB\n"
                f"中位 {float(np.median(snr_db - thr_db)):.1f} dB",
                xy=(0.03, 0.30), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    hs = [Line2D([], [], ls="none", marker="o", ms=5, color=scmap[s],
                 label=f"卫星 {s}") for s in sats]
    hs += [Line2D([], [], ls="none", marker=tmark[t], ms=5, color=C["neutral"],
                  label=f"目标 {t}") for t in tids]
    ax.legend(handles=hs, loc="center right", ncol=1, fontsize=6.2,
              frameon=True, framealpha=0.92, edgecolor="#c8d0d8",
              handletextpad=0.3, labelspacing=0.25)
    style_axes(ax)

    ax = axes[0, 1]
    for s, t, g in pairs:
        ax.scatter(g["rng"] / 1e3, _safe_db(g["p_rx"]), s=4, marker=tmark[t],
                   color=scmap[s], alpha=0.22, linewidths=0)
    for s in sats:
        m = rad["sat"].to_numpy(float) == s
        if not m.any():
            continue
        # 1/R² 参考线锚定该卫星 τ 最高的最近样本：τ 更低时叠加了大气衰减。
        idx = np.where(m & (tau >= 0.98))[0]
        if idx.size == 0:
            idx = np.where(m)[0]
        i0 = int(idx[np.argmin(rng[idx])])
        xs = np.linspace(float(np.min(rng[m])), float(np.max(rng[m])), 60)
        ax.plot(xs, p_rx[i0] - 20.0 * np.log10(xs / rng[i0]),
                color=scmap[s], ls="--", lw=1.0)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("接收功率 (dBW)")
    ax.set_title("(b) 接收功率–距离（虚线＝各卫星 1/R² 参考）", fontsize=10.5)
    _legend_sat(ax, sats, scmap, loc="lower left", fs=7.0)
    style_axes(ax)

    ax = axes[1, 0]
    edges = np.histogram_bin_edges(snr_db, bins=44)
    for s, t, g in pairs:
        v = _safe_db(g["snr"])
        ax.hist(v, bins=edges, histtype="step", lw=1.2, color=scmap[s],
                ls=lmap[t])
    ax.axvline(thr_db, color=C["warn"], ls="--", lw=1.1)
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("辐射记录数")
    ax.set_title("(c) SNR 分布与门限（颜色＝卫星，线型＝目标）", fontsize=10.5)
    ax.annotate(f"SNR 门通过 {int(snr_ok.sum())}/{len(snr_ok)}\n"
                f"视场门通过 {int(fov_ok.sum())}/{len(fov_ok)}\n"
                f"综合准入 {int(admit_ok.sum())}/{len(admit_ok)}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    _pair_legend(ax, sats, tids, scmap, lmap, loc="center left", ncol=2,
                 fs=6.0,
                 extra=[Line2D([], [], color=C["warn"], ls="--", lw=1.1,
                               label=f"门限 {thr_db:.1f} dB")])
    style_axes(ax)

    ax = axes[1, 1]
    margin = snr_db - thr_db
    edges = np.histogram_bin_edges(margin, bins=44)
    for s, t, g in pairs:
        v = _safe_db(g["snr"]) - thr_db
        ax.hist(v, bins=edges, histtype="step", lw=1.2, color=scmap[s],
                ls=lmap[t])
    ax.set_xlabel("SNR 余量（SNR − 门限） (dB)")
    ax.set_ylabel("辐射记录数")
    ax.set_title("(d) 检测余量分布（颜色＝卫星，线型＝目标）", fontsize=10.5)
    ax.annotate(f"余量 {_rng_txt(margin, ' dB', 1)}\n"
                f"低于门限 {int((margin < 0).sum())} 条",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    _pair_legend(ax, sats, tids, scmap, lmap, loc="center left", ncol=2,
                 fs=6.0)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 37
@needs(("ir_int", "ir_rad", "红外辐射噪声与检测判决交互接口测试"),
       title="图 37 大气透过率与辐射传输")
def fig37(D):
    rad = D["ir_int"]["ir_rad"]
    pairs = _pairs(rad)
    sats = sorted({s for s, _, _ in pairs})
    tids = sorted({t for _, t, _ in pairs})
    scmap = color_map(sats)
    tmark = marker_map(tids)
    lmap = ls_map(tids)
    rng = rad["rng"].to_numpy(float) / 1e3
    tau = rad["tau"].to_numpy(float)
    snr_db = _safe_db(rad["snr"].to_numpy(float))
    p_rx = _safe_db(rad["p_rx"].to_numpy(float))
    e_sig = _safe_db(rad["e_sig"].to_numpy(float))

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 37  大气透过率、接收功率与能量预算（按卫星/目标）",
                f"斜距 {_rng_txt(rng, ' km', 0)}，τ {_rng_txt(tau, '', 3)}；"
                f"信号能量 {_rng_txt(e_sig, ' dBJ', 1)}；"
                f"颜色＝卫星、标记＝目标")

    ax = axes[0, 0]
    for s, t, g in pairs:
        ax.scatter(g["rng"] / 1e3, g["tau"], s=3, marker=tmark[t],
                   color=scmap[s], alpha=0.18, linewidths=0)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("大气透过率 τ")
    ax.set_title("(a) τ–距离密度（颜色＝卫星，标记＝目标）", fontsize=10.5)
    _legend_sat(ax, sats, scmap, loc="lower left", fs=7.0)
    style_axes(ax)

    ax = axes[0, 1]
    edges = np.histogram_bin_edges(tau, bins=44)
    for s, t, g in pairs:
        ax.hist(g["tau"], bins=edges, histtype="step", lw=1.2, color=scmap[s],
                ls=lmap[t])
        ax.axvline(float(np.median(g["tau"])), color=scmap[s], ls="--", lw=0.8)
    q10, q50, q90 = np.percentile(tau, [10, 50, 90])
    ax.axvline(q50, color=C["warn"], ls="-", lw=1.0)
    ax.set_xlabel("大气透过率 τ")
    ax.set_ylabel("辐射记录数")
    ax.set_title(f"(b) τ 分布（颜色＝卫星，线型＝目标；全体 10% {q10:.3f} / "
                 f"90% {q90:.3f}）", fontsize=9.4)
    _pair_legend(ax, sats, tids, scmap, lmap, loc="center left", ncol=1,
                 fs=5.8,
                 extra=[Line2D([], [], color=C["warn"], ls="-", lw=1.0,
                               label=f"全体中位 {q50:.3f}")])
    style_axes(ax)

    ax = axes[1, 0]
    for s, t, g in pairs:
        ax.scatter(g["rng"] / 1e3, _safe_db(g["p_rx"]), s=3, marker=tmark[t],
                   color=scmap[s], alpha=0.18, linewidths=0)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("接收功率 (dBW)")
    ax.set_title("(c) 接收功率–距离（颜色＝卫星，标记＝目标）", fontsize=10.5)
    _legend_sat(ax, sats, scmap, loc="upper right", fs=7.0)
    style_axes(ax)

    ax = axes[1, 1]
    for s, t, g in pairs:
        ax.scatter(g["tau"], _safe_db(g["snr"]), s=3, marker=tmark[t],
                   color=scmap[s], alpha=0.18, linewidths=0)
    ax.set_xlabel("大气透过率 τ")
    ax.set_ylabel("SNR (dB)")
    ax.set_title("(d) SNR 与 τ（颜色＝卫星，标记＝目标）", fontsize=10.5)
    _legend_sat(ax, sats, scmap, loc="lower right", fs=7.0)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 38
@needs(("ir_int", "ir_trk", "红外量测与跟踪滤波交互接口测试"),
       title="图 38 红外跟踪 NIS 一致性")
def fig38(D):
    trk = D["ir_int"]["ir_trk"].sort_values("t")
    pairs = _pairs(trk)
    sats = sorted({s for s, _, _ in pairs})
    tids = sorted({t for _, t, _ in pairs})
    scmap = color_map(sats)
    tmark = marker_map(tids)
    lmap = ls_map(tids)
    nis = trk["nis"].to_numpy(float)
    over = trk["nis_res"].astype(str).str.startswith("超门").to_numpy()
    g95 = float(chi2.ppf(0.95, 2))
    g99 = float(chi2.ppf(0.99, 2))

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 38  红外跟踪 NIS 与新息一致性（按卫星/目标）",
                f"跟踪记录 {len(trk)} 条（卫星 {'、'.join(str(s) for s in sats)}；"
                f"目标 {'、'.join(str(t) for t in tids)}），NIS {_rng_txt(nis, '', 1)}；"
                f"χ²(2) 95% 门限 {g95:.2f}、99% 门限 {g99:.2f}，"
                f"超门 {int(over.sum())} 条；颜色＝卫星、标记/线型＝目标")

    ax = axes[0, 0]
    n_zero = int((~np.isfinite(db10(nis))).sum())
    for s, t, g in pairs:
        x = g["t"].to_numpy(float)
        y = db10(g["nis"].to_numpy(float))
        fin = np.isfinite(y)
        ax.plot(x[fin], y[fin], ls="none", marker=tmark[t], ms=1.8,
                color=scmap[s], alpha=0.45,
                label=f"卫星 {s} · {target_title(t)}")
    ax.axhline(float(db10(g95)), color=C["ok"], ls="--", lw=0.9,
               label="95% 门限")
    ax.axhline(float(db10(g99)), color=C["accent4"], ls=":", lw=1.0,
               label="99% 门限")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("NIS (dB，10·lg)")
    ax.set_title("(a) NIS 时序与门限", fontsize=10.5)
    ax.annotate(f"NIS=0 共 {n_zero} 条（10·lg 无定义，未画）\n"
                f"超 99% 门限 {int((nis > g99).sum())} 条",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.0,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=6.2, ncol=2, framealpha=0.9)
    style_axes(ax)

    ax = axes[0, 1]
    edges = np.linspace(0.0, 20.0, 45)
    binsz = float(edges[1] - edges[0])
    for s, t, g in pairs:
        v = g["nis"].to_numpy(float)
        hi = v[v <= 20]
        # 用该组合的全样本归一化：截到 0～20 只为看清主峰，不改变密度分母。
        w = np.ones(hi.size) / (len(v) * binsz)
        ax.hist(hi, bins=edges, weights=w, histtype="step", lw=1.3,
                color=scmap[s], ls=lmap[t], label=f"卫星 {s} · {target_title(t)}")
    xs = np.linspace(0.01, 20, 400)
    ax.plot(xs, chi2.pdf(xs, 2), color=C["warn"], lw=1.6,
            label="χ²(2) 理论")
    ax.axvline(g95, color=C["ok"], ls="--", lw=0.9)
    ax.axvline(g99, color=C["accent4"], ls=":", lw=1.0)
    ax.set_xlabel("NIS")
    ax.set_ylabel("概率密度")
    ax.set_ylim(0, 0.56)
    ax.set_title("(b) NIS 分布与 χ²(2) 理论（按卫星/目标）", fontsize=10.5)
    ax.annotate(f"中位 {float(np.median(nis)):.2f}（理论 1.39）\n"
                f"NIS>20 共 {int((nis > 20).sum())} 条（已计入归一化）\n"
                f"超 95% 门限 {float(over.mean())*100:.1f}%",
                xy=(0.42, 0.94), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="center right", fontsize=6.2)
    style_axes(ax)

    ax = axes[1, 0]
    for s, t, g in pairs:
        o = g["nis_res"].astype(str).str.startswith("超门").to_numpy()
        win = max(20, len(o) // 20)
        rate = np.convolve(o.astype(float), np.ones(win) / win, mode="valid")
        ax.plot(g["t"].to_numpy(float)[win - 1:], rate * 100.0, lw=1.2,
                color=scmap[s], ls=lmap[t],
                label=f"卫星 {s} · {target_title(t)}")
    ax.axhline(5.0, color=C["warn"], ls="--", lw=1.0, label="理论 5%")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("超门比例 (%)")
    ax.set_title("(c) 滚动超门率（各组合按自身长度开窗）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=6.2)
    style_axes(ax)

    ax = axes[1, 1]
    for s, t, g in pairs:
        v = g["nis"].to_numpy(float)
        pos = v[v > 0]
        if pos.size < 3:
            continue
        ss = np.sort(pos)
        p = (np.arange(1, ss.size + 1) - 0.5) / ss.size
        ax.plot(db10(chi2.ppf(p, 2)), db10(ss), ls="none", marker=tmark[t],
                ms=2.0, color=scmap[s], alpha=0.5,
                label=f"卫星 {s} · {target_title(t)}")
    lo = float(db10(max(chi2.ppf(0.5 / 1000.0, 2), 1e-3)))
    hi = float(db10(chi2.ppf(0.9995, 2)))
    ax.plot([lo, hi], [lo, hi], color=C["warn"], ls="--", lw=1.0, label="1:1")
    ax.set_xlabel("χ²(2) 理论分位 (dB)")
    ax.set_ylabel("NIS 实测分位 (dB)")
    ax.set_title("(d) Q–Q 一致性（NIS>0）", fontsize=10.5)
    ax.annotate(f"NIS=0 的 {int((nis <= 0).sum())} 条（10·lg 无定义）\n"
                "已从 Q–Q 样本中剔除",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=6.2)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 39
@needs(("ir_int", "ir_trk", "红外量测与跟踪滤波交互接口测试"),
       ("ir_int", "ir_cand", "红外宽场候选与窄场调度交互接口测试"),
       ("ir_int", "ir_geom", "红外场景几何与视场门控交互接口测试"),
       title="图 39 红外视线角、视场门与候选")
def fig39(D):
    trk = D["ir_int"]["ir_trk"]
    cand = D["ir_int"]["ir_cand"]
    geom = D["ir_int"]["ir_geom"]

    tgrp = _pairs(trk)
    cgrp = _pairs(cand)
    # 几何门控覆盖 5 颗卫星，标题与配色必须把它们都算进来，否则面板 (b)(c)
    # 里出现的卫星在图例/标题中没有交代。
    gsats_all = sorted(int(v) for v in geom["sat"].dropna().unique())
    sats = sorted({s for s, _, _ in tgrp + cgrp} | set(gsats_all))
    tids = sorted({t for _, t, _ in tgrp + cgrp})
    scmap = color_map(sats)
    tmark = marker_map(tids)
    csat = cand["sat"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 39  红外跟踪视线角、视场门与宽场候选（按卫星/目标）",
                f"跟踪 {len(trk)} 条、候选 {len(cand)} 条；"
                f"几何门控记录 {len(geom)} 条；卫星 {'、'.join(str(s) for s in sats)}，"
                f"目标 {'、'.join(str(t) for t in tids)}；颜色＝卫星、标记＝目标")

    ax = axes[0, 0]
    for s, t, g in tgrp:
        ax.scatter(g["az"], g["el"], s=3, marker=tmark[t], color=scmap[s],
                   alpha=0.2, linewidths=0)
    for s, t, g in cgrp:
        ax.scatter(g["az"], g["el"], s=34, marker=tmark[t], facecolors="none",
                   edgecolors=scmap[s], linewidths=1.2, zorder=3)
    ax.set_xlabel("方位 (°)")
    ax.set_ylabel("俯仰 (°)")
    ax.set_title("(a) 跟踪视线角分布与候选位置（空心＝候选）", fontsize=10.5)
    tsats = sorted({s for s, _, _ in tgrp})
    hs = [Line2D([], [], ls="none", marker="o", ms=5, color=scmap[s],
                 label=f"卫星 {s}") for s in tsats]
    hs += [Line2D([], [], ls="none", marker=tmark[t], ms=5, color=C["neutral"],
                  label=f"目标 {t}") for t in tids]
    ax.legend(handles=hs, loc="best", ncol=2, fontsize=6.2,
              handletextpad=0.3, labelspacing=0.3)
    style_axes(ax)

    ax = axes[0, 1]
    wf = geom[geom["stage"].astype(str).eq("WFOV")]
    res = wf["fov_res"].astype(str)
    gsats = sorted(int(v) for v in wf["sat"].dropna().unique())
    tot = [int((wf["sat"] == s).sum()) for s in gsats]
    pas = [int(((wf["sat"] == s) & res.eq("通过")).sum()) for s in gsats]
    rate = [100.0 * p_ / t_ if t_ else 0.0 for p_, t_ in zip(pas, tot)]
    x = np.arange(len(gsats))
    for xi, s in zip(x, gsats):
        ax.bar(xi, rate[xi], width=0.58, color=scmap.get(s, C["accent3"]),
               alpha=0.9, label=f"卫星 {s}")
    ax.set_xticks(x)
    ax.set_xticklabels([f"卫星 {int(s)}" for s in gsats], fontsize=8)
    ax.set_ylabel("视场门通过率 (%)")
    ax.set_ylim(0, max(rate + [0.0]) * 1.45 if max(rate) > 0 else 1.0)
    ax.set_title("(b) 各卫星视场门通过率", fontsize=10.5)
    for xi, p_, t_, r_ in zip(x, pas, tot, rate):
        # 通过数为 0 时柱高为 0，必须直接用文字标出计数，避免误读为漏画。
        ax.annotate(f"{p_}/{t_}\n{r_:.2f}%", xy=(xi, r_), xytext=(0, 3),
                    textcoords="offset points", ha="center", fontsize=6.8,
                    va="bottom",
                    color=scmap.get(gsats[xi], C["accent3"]) if p_ else C["warn"])
    ax.annotate(f"合计 {sum(pas)}/{sum(tot)} 条通过",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=7.0, ncol=2)
    style_axes(ax)

    ax = axes[1, 0]
    occ = geom[geom["stage"].astype(str).eq("地球遮挡")]
    ogrp = _pairs(occ)
    osats = sorted({s for s, _, _ in ogrp if s is not None})
    otids = sorted({t for _, t, _ in ogrp if t is not None})
    lmap = ls_map(sorted(set(tids) | set(otids)))
    allm = occ["margin"].to_numpy(float) / 1e3
    allm = allm[np.isfinite(allm)]
    edges = np.histogram_bin_edges(allm, bins=44) if allm.size else 44
    for s, t, g in ogrp:
        m = g["margin"].to_numpy(float) / 1e3
        m = m[np.isfinite(m)]
        if not m.size:
            continue
        ax.hist(m, bins=edges, histtype="step", lw=1.2,
                color=scmap.get(s, C["accent4"]), ls=lmap.get(t, "-"))
    ax.set_xlabel("遮挡余量 (km)")
    ax.set_ylabel("记录数")
    ax.set_title("(c) 地球遮挡余量分布（颜色＝卫星，线型＝目标）", fontsize=10.5)
    ax.annotate(f"{len(allm)} 条遮挡记录\n"
                f"余量 {_rng_txt(allm, ' km', 0)}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    _pair_legend(ax, osats, otids, scmap, lmap, loc="center left", ncol=2,
                 fs=5.8)
    style_axes(ax)

    ax = axes[1, 1]
    for s, t, g in cgrp:
        ax.scatter(g["tmeas"], _safe_db(g["snr"]), s=34, marker=tmark[t],
                   color=scmap[s], alpha=0.9, edgecolors="white", linewidths=0.4,
                   label=f"卫星 {s} · {target_title(t)}")
    ax.set_xlabel("候选量测时刻 (s)")
    ax.set_ylabel("候选 SNR (dB)")
    ax.set_title("(d) 候选 SNR 与时刻（颜色＝卫星，标记＝目标）", fontsize=10.5)
    ax.annotate(f"连续命中均为 {int(cand['hits'].min())}\n"
                f"候选状态：{cand['state'].iloc[0] if len(cand) else '无'}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=6.6)
    style_axes(ax)
    return fig


FIGURES = [
    ("36", "fig36_ir_snr_range", "图 36 红外辐射 SNR–距离", fig36),
    ("37", "fig37_ir_tau_budget", "图 37 大气透过率与辐射传输", fig37),
    ("38", "fig38_ir_nis", "图 38 红外跟踪 NIS 一致性", fig38),
    ("39", "fig39_ir_coverage_cand", "图 39 红外视线角与候选", fig39),
]
