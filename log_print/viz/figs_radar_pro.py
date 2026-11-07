# -*- coding: utf-8 -*-
"""图 26–35：面向期刊版式的雷达检测、信号处理、跟踪与资源管理图。

这些图与 4–11、16–25 并存，不替换原图；差别在于版式按论文常见图型组织：
检测性能曲线族、距离–多普勒量测平面、杂波分布与 CFAR、MTI/MTD 增益、
J/S 与压制比、量测误差与 CRLB 标度、真值–航迹对比、资源占用、天线方向图、
HRRP 距离像。所有文字与坐标窗仍由数据判定，不写死本批数值。
"""
from __future__ import annotations

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from scipy.stats import ncx2

from . import common, extract
from .common import (C, CYCLE, align_same_rows, by_target, color_map, db10, db20,
                     marker_map, needs, obs_mask, radar_obs_span, style_axes,
                     target_title, title_block)

C_LIGHT = 299792458.0


# ---------------------------------------------------------------- 公共
def _bin(x, y, nbins=36, minn=12):
    """按 x 等宽分箱，返回箱中心与中位/10–90% 分位。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if x.size < 2:
        e = np.array([])
        return e, e, e, e
    edges = np.linspace(x.min(), x.max(), nbins + 1)
    idx = np.digitize(x, edges)
    cx, med, lo, hi = [], [], [], []
    for k in range(1, len(edges)):
        m = idx == k
        if m.sum() >= minn:
            cx.append(0.5 * (edges[k - 1] + edges[k]))
            med.append(float(np.median(y[m])))
            lo.append(float(np.percentile(y[m], 10)))
            hi.append(float(np.percentile(y[m], 90)))
    return (np.asarray(cx), np.asarray(med), np.asarray(lo), np.asarray(hi))


def _rsig(v):
    """稳健标准差（1.4826·MAD），避免野值把 σ 拉爆。"""
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if v.size < 3:
        return np.nan
    med = float(np.median(v))
    return float(1.4826 * np.median(np.abs(v - med)))


def _rng_txt(v, unit="", nd=2):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if not v.size:
        return "未记录"
    return f"{v.min():.{nd}f}–{v.max():.{nd}f}{unit}"


def _fit_pfa(snr_db, pd_obs):
    """在 Swerling I 曲线上拟合等效虚警率：Pd = Pfa^(1/(1+SNR))。"""
    snr_db = np.asarray(snr_db, dtype=float)
    pd_obs = np.asarray(pd_obs, dtype=float)
    ok = np.isfinite(snr_db) & np.isfinite(pd_obs)
    snr_db, pd_obs = snr_db[ok], pd_obs[ok]
    if snr_db.size < 20:
        return np.nan
    lin = 10.0 ** (snr_db / 10.0)
    best = (np.inf, np.nan)
    for g in np.linspace(0.2, 40.0, 400):
        pred = np.exp(-g / (1.0 + lin))
        err = float(np.mean((pred - pd_obs) ** 2))
        if err < best[0]:
            best = (err, g)
    return float(np.exp(-best[1]))


def _swelling_i(snr_db, pfa):
    return np.exp(np.log(pfa) / (1.0 + 10.0 ** (np.asarray(snr_db) / 10.0)))


def _nonfluct(snr_db, pfa):
    gamma = -np.log(pfa)
    lin = 10.0 ** (np.asarray(snr_db) / 10.0)
    return ncx2.sf(2.0 * gamma, 2, 2.0 * lin)


# ---------------------------------------------------------------- 图 26
@needs(("radar_ext", "plot", "雷达对外目标信息输出接口测试-点迹量测"),
       ("radar_int", "det", "雷达信号处理与检测判决交互接口测试"),
       ("radar_output", "cfar", "恒虚警检测功能测试"),
       title="图 26 检测性能曲线族")
def fig26(D):
    p = D["radar_ext"]["plot"]
    det = D["radar_int"]["det"].sort_values("t")
    cfar = D["radar_output"]["cfar"].sort_values("t")

    groups = by_target(p)
    ids = [t for t, _ in groups]
    cmap = color_map(ids)
    mmap = marker_map(ids)
    snr_all = p["snr"].to_numpy(float)
    pd_all = p["pd"].to_numpy(float)
    thr = float(det["thr"].median())

    pfa_txt = []
    pfa_all = _fit_pfa(snr_all, pd_all)
    for tid, g in groups:
        pf = _fit_pfa(g["snr"].to_numpy(float), g["pd"].to_numpy(float))
        pfa_txt.append(f"{target_title(tid)} " +
                       (f"$P_{{fa}}$={pf:.2e}" if np.isfinite(pf) else "样本不足"))
    xs_all = np.linspace(float(np.nanmin(snr_all)), float(np.nanmax(snr_all)), 300)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 26  检测性能曲线族：$P_d$–SNR、$P_d$–R 与 90% 作用距离",
                f"点迹 {len(p)} 条（{'、'.join(f'{target_title(t)} {len(g)} 条' for t, g in groups)}），"
                f"SNR {_rng_txt(snr_all, ' dB')}，检测门限 {thr:.2f} dB；"
                f"逐目标拟合等效虚警率：{'；'.join(pfa_txt)}")

    ax = axes[0, 0]
    for tid, g in groups:
        snr = g["snr"].to_numpy(float)
        pdv = g["pd"].to_numpy(float)
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        small = len(g) < 20
        ax.scatter(snr, pdv, s=30 if small else 8, color=col, marker=mk,
                   alpha=0.9 if small else 0.45, linewidths=0.5 if small else 0,
                   edgecolors="white" if small else None,
                   zorder=3 if small else 2, label=f"{target_title(tid)} 点迹")
        cxs, med, lo, hi = _bin(snr, pdv, 40, 15)
        if cxs.size:
            ax.plot(cxs, med, color=col, lw=1.8,
                    label=f"{target_title(tid)} 分箱中位")
            ax.fill_between(cxs, lo, hi, color=col, alpha=0.12, lw=0)
        pf = _fit_pfa(snr, pdv)
        if np.isfinite(pf):
            xs = np.linspace(float(np.nanmin(snr)), float(np.nanmax(snr)), 200)
            ax.plot(xs, _swelling_i(xs, pf), color=col, lw=1.2, ls="--")
    if np.isfinite(pfa_all):
        ax.plot(xs_all, _nonfluct(xs_all, pfa_all), color=C["neutral"], lw=1.2,
                ls=":", label="非起伏目标（全体 $P_{fa}$）")
    ax.axvline(thr, color=C["ok"], lw=0.9, ls="-.", label=f"门限 {thr:.1f} dB")
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("检测概率 $P_d$")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("(a) $P_d$–SNR 与理论曲线（虚线＝逐目标拟合）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.2)
    style_axes(ax)

    ax = axes[0, 1]
    for tid, g in groups:
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        rng = g["rng"].to_numpy(float) / 1e3
        pdv = g["pd"].to_numpy(float)
        small = len(g) < 20
        ax.scatter(rng, pdv, s=30 if small else 7, color=col, marker=mk,
                   alpha=0.9 if small else 0.6, linewidths=0,
                   edgecolors="white" if small else None, zorder=3 if small else 2,
                   label=f"{target_title(tid)}（{len(g)} 条）")
        cxr, medr, lor, hir = _bin(rng, pdv, 36, 15)
        if cxr.size:
            ax.plot(cxr, medr, color=col, lw=1.6)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("检测概率 $P_d$")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("(b) $P_d$–距离（颜色＝目标）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 0]
    for tid, g in by_target(cfar):
        col = cmap.get(int(tid), C["accent3"]) if tid is not None else C["accent3"]
        r90 = g["r90"].to_numpy(float) / 1e3
        if len(g) > 1:
            ax.plot(g["t"], r90, color=col, lw=1.5, label=target_title(tid))
            ax.fill_between(g["t"], r90, color=col, alpha=0.12)
        else:
            ax.plot(g["t"], r90, ls="none", marker="D", ms=5, color=col,
                    label=f"{target_title(tid)}（{len(g)} 次）")
    ax.axhline(float(np.median(cfar["r90"])) / 1e3, color=C["warn"], ls="--",
               lw=1.0, label=f"全体中位 {float(np.median(cfar['r90']))/1e3:.0f} km")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("$P_d=90\\%$ 作用距离 (km)")
    ax.set_title("(c) CFAR 90% 检测距离（按目标）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 1]
    notes = []
    for tid, g in groups:
        col = cmap[int(tid)]
        margin = g["snr"].to_numpy(float) - thr
        ax.hist(margin, bins=48, histtype="step", lw=1.5, color=col,
                label=target_title(tid))
        ax.axvline(float(np.median(margin)), color=col, ls="--", lw=0.9)
        notes.append(f"{target_title(tid)}：中位 {float(np.median(margin)):.1f} dB，"
                     f"低于门限 {int((margin < 0).sum())}/{len(margin)} 条")
    ax.axvline(0.0, color=C["warn"], lw=1.1, ls="--", label="门限")
    ax.set_xlabel("SNR 余量（SNR − 门限） (dB)")
    ax.set_ylabel("点迹数")
    ax.set_title("(d) 检测余量分布（按目标叠加）", fontsize=10.5)
    ax.annotate("\n".join(notes), xy=(0.03, 0.94), xycoords="axes fraction",
                fontsize=7.8, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=7.6)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 27
@needs(("radar_int", "echo_sp", "雷达回波信号处理交互接口测试"),
       ("radar_ext", "plot", "雷达对外目标信息输出接口测试-点迹量测"),
       title="图 27 距离–多普勒量测平面")
def fig27(D):
    e = D["radar_int"]["echo_sp"]
    p = D["radar_ext"]["plot"]

    ge = by_target(e)
    gp = by_target(p)
    ids = [t for t, _ in ge + gp]
    cmap = color_map(ids)
    mmap = marker_map(ids)

    rng = e["rng"].to_numpy(float) / 1e3
    dop = e["dop_out"].to_numpy(float) / 1e3
    prf = e["prf"].to_numpy(float)
    delay = e["delay_out"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 27  距离–多普勒量测平面与距离标定（按目标）",
                f"回波 {len(e)} 条（{'、'.join(f'{target_title(t)} {len(g)} 条' for t, g in ge)}），"
                f"斜距 {_rng_txt(rng, ' km', 0)}，多普勒 {_rng_txt(dop, ' kHz', 1)}；"
                f"PRF {_rng_txt(prf, ' Hz', 1)}；颜色与标记按目标区分")

    ax = axes[0, 0]
    for tid, g in ge:
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        small = len(g) < 20
        ax.scatter(g["rng"] / 1e3, g["dop_out"] / 1e3, s=40 if small else 18,
                   color=col, marker=mk, alpha=0.95 if small else 0.85,
                   edgecolors="white", linewidths=0.4,
                   label=f"{target_title(tid)}（{len(g)} 条）")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("多普勒频率 (kHz)")
    ax.set_title("(a) 距离–多普勒点迹（颜色/标记＝目标）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.8)
    style_axes(ax)

    ax = axes[0, 1]
    for tid, g in gp:
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        small = len(g) < 20
        ax.scatter(g["rng"] / 1e3, g["vr"], s=40 if small else 7, color=col,
                   marker=mk, alpha=0.95 if small else 0.7, linewidths=0,
                   edgecolors="white" if small else None, zorder=3 if small else 2,
                   label=f"{target_title(tid)}（{len(g)} 点）")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("径向速度 $v_r$ (m/s)")
    ax.set_title("(b) 点迹距离–径向速度（颜色/标记＝目标）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.8)
    style_axes(ax)

    ax = axes[1, 0]
    edges = np.histogram_bin_edges(dop, bins=40)
    for tid, g in ge:
        col = cmap[int(tid)]
        d = g["dop_out"].to_numpy(float) / 1e3
        if len(d) >= 3:
            ax.hist(d, bins=edges, histtype="step", lw=1.5, color=col,
                    label=target_title(tid))
            ax.axvline(float(np.median(d)), color=col, ls="--", lw=0.9)
        else:
            ax.axvline(float(d[0]), color=col, ls=":", lw=1.4,
                       label=f"{target_title(tid)}（{len(d)} 条，单点）")
    ax.set_xlabel("多普勒频率 (kHz)")
    ax.set_ylabel("回波数")
    ax.set_title("(c) 多普勒分布（按目标叠加）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.8)
    style_axes(ax)

    ax = axes[1, 1]
    r_cal = C_LIGHT * delay / 2.0 / 1e3
    for tid, g in ge:
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        gd = g["delay_out"].to_numpy(float)
        rc = C_LIGHT * gd / 2.0 / 1e3
        gr = g["rng"].to_numpy(float) / 1e3
        small = len(g) < 20
        ax.scatter(gr, rc, s=40 if small else 10, color=col, marker=mk,
                   alpha=0.95 if small else 0.7, linewidths=0,
                   edgecolors="white" if small else None,
                   label=f"{target_title(tid)}（{len(g)} 条）")
    lim = [min(float(np.nanmin(rng)), float(np.nanmin(r_cal))),
           max(float(np.nanmax(rng)), float(np.nanmax(r_cal)))]
    ax.plot(lim, lim, color=C["neutral"], lw=0.9, ls="--", label="1:1")
    rel = np.abs(r_cal - rng) / np.clip(rng, 1e-9, None)
    ax.set_xlabel("日志斜距 (km)")
    ax.set_ylabel("$c\\,\\Delta t/2$ (km)")
    ax.set_title("(d) 时延–距离一致性", fontsize=10.5)
    ax.annotate(f"全体中位相对偏差 {float(np.median(rel))*100:.2f}%\n"
                f"最大 {float(np.max(rel))*100:.2f}%",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=7.6)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 28
@needs(("radar_int", "det", "雷达信号处理与检测判决交互接口测试"),
       ("radar_output", "noise", "噪声与杂波计算功能测试"),
       title="图 28 噪声分布与 CFAR 门限")
def fig28(D):
    det = D["radar_int"]["det"].sort_values("t")
    noise = D["radar_output"]["noise"]

    groups = by_target(det)
    ids = [t for t, _ in groups]
    cmap = color_map(ids)
    pn_all = det["p_noise"].to_numpy(float)
    pn_all = pn_all[np.isfinite(pn_all) & (pn_all > 0)]
    pn_db_all = db10(pn_all)
    thr = float(det["thr"].median())
    snr_all = det["snr_out"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 28  噪声功率分布、CFAR 门限与多普勒通道噪声底（按目标）",
                f"检测记录 {len(det)} 条（{'、'.join(f'{target_title(t)} {len(g)} 条' for t, g in groups)}），"
                f"噪声样本 {len(pn_all)} 个，噪声中位 {float(np.median(pn_db_all)):.1f} dBW，"
                f"SNR 门限 {thr:.2f} dB")

    ax = axes[0, 0]
    for tid, g in groups:
        col = cmap[int(tid)]
        pn = g["p_noise"].to_numpy(float)
        pn = pn[np.isfinite(pn) & (pn > 0)]
        if pn.size == 0:
            continue
        if pn.size < 2:
            ax.plot([float(db10(pn[0]))], [1.0], ls="none", marker="o", ms=6,
                    color=col, label=f"{target_title(tid)}（1 个样本）")
            continue
        s = np.sort(pn)
        f = np.arange(1, s.size + 1) / s.size
        ax.plot(db10(s), f, drawstyle="steps-post", color=col, lw=1.4,
                label=f"{target_title(tid)}（{s.size} 个）")
        mean_p = float(np.mean(s))
        grid = np.linspace(float(db10(s[0])), float(db10(s[-1])), 200)
        ax.plot(grid, 1.0 - np.exp(-(10.0 ** (grid / 10.0)) / mean_p),
                color=col, lw=1.0, ls="--")
    ax.set_xlabel("噪声功率 (dBW)")
    ax.set_ylabel("累积概率")
    ax.set_title("(a) 噪声功率 CDF（实线实测，虚线指数拟合）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.6)
    style_axes(ax)

    ax = axes[0, 1]
    edges = db10(np.histogram_bin_edges(pn_all, bins=44))
    for tid, g in groups:
        col = cmap[int(tid)]
        pn = g["p_noise"].to_numpy(float)
        pn = pn[np.isfinite(pn) & (pn > 0)]
        if pn.size >= 3:
            ax.hist(db10(pn), bins=edges, density=True, histtype="step",
                    lw=1.4, color=col, label=target_title(tid))
        else:
            ax.axvline(float(db10(pn[0])), color=col, ls=":", lw=1.5,
                       label=f"{target_title(tid)}（单点）")
        if pn.size >= 20:
            mean_p = float(np.mean(pn))
            y = np.linspace(float(db10(pn.min())), float(db10(pn.max())), 200)
            p_lin = 10.0 ** (y / 10.0)
            pdf_db = (np.log(10.0) / 10.0 * p_lin / mean_p
                      * np.exp(-p_lin / mean_p))
            ax.plot(y, pdf_db, color=col, lw=1.0, ls="--")
    ax.set_xlabel("噪声功率 (dBW)")
    ax.set_ylabel("概率密度")
    ax.set_title("(b) 噪声功率 PDF（按目标）", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 0]
    sedges = np.histogram_bin_edges(snr_all, bins=40)
    notes = []
    for tid, g in groups:
        col = cmap[int(tid)]
        snr = g["snr_out"].to_numpy(float)
        if snr.size >= 3:
            ax.hist(snr, bins=sedges, histtype="step", lw=1.5, color=col,
                    label=f"{target_title(tid)}（{len(snr)} 条）")
            ax.axvline(float(np.median(snr)), color=col, ls="--", lw=0.9)
        else:
            ax.axvline(float(snr[0]), color=col, ls=":", lw=1.5,
                       label=f"{target_title(tid)}（单点）")
        notes.append(f"{target_title(tid)}：低于门限 {int((snr < thr).sum())}/{len(snr)} 条，"
                     f"中位 {float(np.median(snr)):.1f} dB")
    ax.axvline(thr, color=C["warn"], ls="--", lw=1.1,
               label=f"SNR 门限 {thr:.1f} dB")
    ax.set_xlabel("检测 SNR (dB)")
    ax.set_ylabel("检测记录数")
    ax.set_title("(c) 检测 SNR 分布与门限（按目标叠加）", fontsize=10.5)
    ax.annotate("\n".join(notes), xy=(0.03, 0.82), xycoords="axes fraction",
                fontsize=7.8, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=7.4)
    style_axes(ax)

    ax = axes[1, 1]

    def _chmat(g):
        vals = []
        for srow in g["ch"].astype(str):
            try:
                vals.append([float(x) for x in srow.split(",") if x.strip()])
            except ValueError:
                continue
        ncol = min((len(v) for v in vals), default=0)
        if not ncol:
            return np.empty((0, 0))
        return np.asarray([v[:ncol] for v in vals], dtype=float)

    ngrp = by_target(noise)
    ncs = []
    for _, g in ngrp:
        m = _chmat(g)
        if m.size:
            ncs.append(int(m.shape[1]))
    ncol = min(ncs) if ncs else 0
    n = max(len(ngrp), 1)
    bw = 0.7 / n
    handles = []
    spreads = []
    for j, (tid, g) in enumerate(ngrp):
        mat = _chmat(g)
        if not mat.size:
            continue
        spreads.append(float(np.nanmax(db10(mat)) - np.nanmin(db10(mat))))
        col = cmap.get(int(tid), CYCLE[j]) if tid is not None else CYCLE[j]
        off = (j - (n - 1) / 2.0) * bw
        pos = np.arange(1, ncol + 1) + off
        if len(mat) >= 5:
            data = [db10(mat[:, k]) for k in range(ncol)]
            bp = ax.boxplot(data, positions=pos, widths=bw * 0.9,
                            showfliers=False, patch_artist=True)
            for box in bp["boxes"]:
                box.set_facecolor(col)
                box.set_alpha(0.65)
                box.set_edgecolor(col)
            for med in bp["medians"]:
                med.set_color("#1b2733")
                med.set_linewidth(1.0)
            handles.append(Patch(facecolor=col, alpha=0.65,
                                 label=f"{target_title(tid)}（{len(mat)} 帧）"))
        else:
            h, = ax.plot(pos, db10(mat[0]), ls="none", marker="D", ms=5,
                         color=col, label=f"{target_title(tid)}（{len(mat)} 帧）")
            handles.append(h)
    ax.set_xticks(range(1, ncol + 1))
    ax.set_xticklabels([f"CH{k+1}" for k in range(ncol)], fontsize=8)
    ax.set_xlabel("多普勒通道")
    ax.set_ylabel("噪声功率 (dBW)")
    ax.set_title("(d) MTD 各通道噪声底（按目标分组）", fontsize=10.5)
    if handles:
        ax.legend(handles=handles, loc="lower right", fontsize=7.6)
    else:
        ax.legend(loc="lower right", fontsize=7.6)
    if spreads and max(spreads) < 0.05:
        ax.annotate(f"各目标各通道噪声底恒定（极差 < 0.05 dB），\n"
                    f"箱体高度为 0，退化为中位实线",
                    xy=(0.03, 0.06), xycoords="axes fraction", fontsize=7.8,
                    va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                           fc="#f7f9fb", ec="#c8d0d8"))
    ax.grid(axis="x", visible=False)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 29
@needs(("radar_output", "gain", "雷达信号处理链增益计算功能测试"),
       ("radar_output", "jam_gain", "干扰抑制增益计算功能测试"),
       title="图 29 MTI/MTD 处理链增益")
def fig29(D):
    g = D["radar_output"]["gain"]
    jg = D["radar_output"]["jam_gain"]

    ggrp = by_target(g)
    jgrp = by_target(jg)
    ids = [t for t, _ in ggrp + jgrp]
    cmap = color_map(ids)
    mmap = marker_map(ids)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 29  脉压 / MTI / MTD 逐级增益与干扰抑制（按目标）",
                f"处理记录 {len(g)} 条（{'、'.join(f'{target_title(t)} {len(gg)} 条' for t, gg in ggrp)}），"
                f"链路 {g['chain'].iloc[0] if len(g) else '未记录'}；"
                f"颜色与标记按目标区分")

    ax = axes[0, 0]
    names = ["脉压", "MTI", "MTD"]
    n = max(len(ggrp), 1)
    bw = 0.66 / n
    for j, (tid, gg) in enumerate(ggrp):
        off = (j - (n - 1) / 2.0) * bw
        data = [gg["pc"].to_numpy(float), gg["mti"].to_numpy(float),
                gg["mtd"].to_numpy(float)]
        meds = [float(np.median(d)) for d in data]
        los = [float(np.percentile(d, 10)) for d in data]
        his = [float(np.percentile(d, 90)) for d in data]
        col = cmap[int(tid)]
        x = np.arange(len(names)) + off
        ax.bar(x, meds, width=bw * 0.9, color=col, alpha=0.85,
               label=f"{target_title(tid)}（{len(gg)} 条）")
        ax.errorbar(x, meds, yerr=[np.asarray(meds) - np.asarray(los),
                                   np.asarray(his) - np.asarray(meds)],
                    fmt="none", ecolor="#2c3e50", capsize=3, lw=0.9)
        ax.plot(x, meds, ls="none", marker=mmap[int(tid)], ms=4, color=col)
    ax.set_xticks(np.arange(len(names)))
    ax.set_xticklabels(names)
    ax.set_ylabel("增益 (dB)")
    ax.set_title("(a) 各级增益中位与 10–90% 分位（柱＝目标）", fontsize=10.5)
    ax.grid(axis="x", visible=False)
    ax.legend(loc="best", fontsize=7.8)
    style_axes(ax)

    ax = axes[0, 1]
    for tid, gg in ggrp:
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        pb = db10(gg["p_before"].to_numpy(float))
        pa = db10(gg["p_after"].to_numpy(float))
        small = len(gg) < 20
        ax.scatter(pb, pa, s=45 if small else 12, color=col, marker=mk,
                   alpha=0.95 if small else 0.65, linewidths=0,
                   edgecolors="white" if small else None,
                   label=target_title(tid))
    pb_all = db10(g["p_before"].to_numpy(float))
    pa_all = db10(g["p_after"].to_numpy(float))
    lim = [min(float(np.nanmin(pb_all)), float(np.nanmin(pa_all))),
           max(float(np.nanmax(pb_all)), float(np.nanmax(pa_all)))]
    ax.plot(lim, lim, color=C["neutral"], ls="--", lw=0.9, label="1:1")
    ax.set_xlabel("处理前功率 (dBW)")
    ax.set_ylabel("处理后功率 (dBW)")
    ax.set_title("(b) 处理前后功率（颜色＝目标）", fontsize=10.5)
    ax.annotate(f"全体实测增益中位 {float(np.nanmedian(pa_all-pb_all)):.1f} dB\n"
                f"{_rng_txt(pa_all-pb_all, ' dB', 1)}",
                xy=(0.97, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", ha="right", bbox=dict(boxstyle="round,pad=0.35",
                                                fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 0]
    tot_all = np.concatenate([gg["pc"].to_numpy(float) + gg["mti"].to_numpy(float)
                              + gg["mtd"].to_numpy(float) for _, gg in ggrp])
    edges = np.histogram_bin_edges(tot_all, bins=36)
    for tid, gg in ggrp:
        col = cmap[int(tid)]
        total = (gg["pc"].to_numpy(float) + gg["mti"].to_numpy(float)
                 + gg["mtd"].to_numpy(float))
        if total.size >= 3:
            ax.hist(total, bins=edges, histtype="step", lw=1.5, color=col,
                    label=f"{target_title(tid)}（{len(total)} 条）")
            ax.axvline(float(np.median(total)), color=col, ls="--", lw=0.9)
        else:
            ax.axvline(float(total[0]), color=col, ls=":", lw=1.5,
                       label=f"{target_title(tid)}（单点）")
    ax.set_xlabel("链路总增益 (dB)")
    ax.set_ylabel("处理记录数")
    ax.set_title("(c) 总增益分布（按目标叠加）", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 1]
    sup_all = np.concatenate([gg["suppr"].to_numpy(float) for _, gg in jgrp])
    edges = np.histogram_bin_edges(sup_all, bins=36)
    for tid, gg in jgrp:
        col = cmap[int(tid)]
        s = gg["suppr"].to_numpy(float)
        if s.size >= 3:
            ax.hist(s, bins=edges, histtype="step", lw=1.5, color=col,
                    label=f"{target_title(tid)}（{len(s)} 条）")
            ax.axvline(float(np.median(s)), color=col, ls="--", lw=0.9)
        else:
            ax.axvline(float(s[0]), color=col, ls=":", lw=1.5,
                       label=f"{target_title(tid)}（单点）")
    ax.set_xlabel("干扰抑制比 (dB)")
    ax.set_ylabel("处理记录数")
    ax.set_title("(d) MTI 后干扰抑制比（按目标叠加）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.6)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 30
@needs(("radar_output", "echo", "回波功率计算功能测试"),
       ("radar_output", "jam_power", "干扰功率计算功能测试"),
       ("radar_output", "jam_gain", "干扰抑制增益计算功能测试"),
       title="图 30 J/S 与干扰压制")
def fig30(D):
    e = D["radar_output"]["echo"]
    jp = D["radar_output"]["jam_power"]
    jg = D["radar_output"]["jam_gain"]

    egrp = by_target(e)
    jgrp = by_target(jg)
    ids = [t for t, _ in egrp + jgrp]
    cmap = color_map(ids)
    mmap = marker_map(ids)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 30  干信比、压制比与干扰源功率（按目标）",
                f"回波 {len(e)} 条（{'、'.join(f'{target_title(t)} {len(g)} 条' for t, g in egrp)}）；"
                f"干扰源 {jp['src'].nunique()} 个（源号与目标号一致）；颜色与标记按目标区分")

    ax = axes[0, 0]
    for tid, g in egrp:
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        js = db10(g["p_jam"].to_numpy(float)
                  / np.clip(g["p_echo"].to_numpy(float), 1e-30, None))
        rng = g["rng"].to_numpy(float) / 1e3
        small = len(g) < 20
        ax.scatter(rng, js, s=45 if small else 16, color=col, marker=mk,
                   alpha=0.95 if small else 0.8, linewidths=0,
                   edgecolors="white" if small else None,
                   label=f"{target_title(tid)}（{len(g)} 条）")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("J/S (dB)")
    ax.set_title("(a) 干信比随距离（颜色＝目标）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.8)
    style_axes(ax)

    ax = axes[0, 1]
    for tid, g in egrp:
        col = cmap[int(tid)]
        js = db10(g["p_jam"].to_numpy(float)
                  / np.clip(g["p_echo"].to_numpy(float), 1e-30, None))
        t = g["t"].to_numpy(float)
        if len(g) == 1:
            ax.plot(t, js, ls="none", marker=mmap[int(tid)], ms=5, color=col,
                    label=target_title(tid))
        else:
            ax.plot(t, js, lw=0.8, color=col, alpha=0.8,
                    label=target_title(tid))
    js_all = db10(e["p_jam"].to_numpy(float)
                  / np.clip(e["p_echo"].to_numpy(float), 1e-30, None))
    ax.axhline(float(np.median(js_all)), color=C["warn"], ls="--", lw=1.0,
               label=f"全体中位 {float(np.median(js_all)):.1f} dB")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("J/S (dB)")
    ax.set_title("(b) 干信比时序（按目标）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 0]
    for tid, g in jgrp:
        col = cmap[int(tid)]
        s = g["suppr"].to_numpy(float)
        t = g["t"].to_numpy(float)
        if len(g) == 1:
            ax.plot(t, s, ls="none", marker=mmap[int(tid)], ms=5, color=col,
                    label=target_title(tid))
        else:
            ax.plot(t, s, lw=0.9, color=col, label=target_title(tid))
    ax.axhline(float(np.median(jg["suppr"])), color=C["warn"], ls="--", lw=1.0,
               label=f"全体中位 {float(np.median(jg['suppr'])):.1f} dB")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("压制比 (dB)")
    ax.set_title("(c) MTI 压制比时序（按目标）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 1]
    srcs = sorted(int(v) for v in jp["src"].dropna().unique())
    scmap = color_map(srcs)
    data = [db10(jp.loc[jp["src"] == s, "power"].to_numpy(float)) for s in srcs]
    vp = ax.violinplot(data, showmedians=True, showextrema=False, widths=0.72)
    for body, s in zip(vp["bodies"], srcs):
        body.set_facecolor(scmap[s])
        body.set_edgecolor(scmap[s])
        body.set_alpha(0.6)
    vp["cmedians"].set_color([scmap[s] for s in srcs])
    vp["cmedians"].set_linewidth(1.4)
    for k, s in enumerate(srcs):
        n = int((jp["src"] == s).sum())
        ax.text(k + 1, float(np.nanmax(data[k])), f"{n} 条", ha="center",
                va="bottom", fontsize=7.4, color=scmap[s])
    ax.set_xticks(range(1, len(srcs) + 1))
    ax.set_xticklabels([f"目标 {s}\n（干扰源）" for s in srcs], fontsize=8)
    ax.set_ylabel("干扰功率 (dBW)")
    ax.set_title("(d) 干扰源功率分布（颜色＝目标）", fontsize=10.5)
    ax.grid(axis="x", visible=False)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 31
@needs(("radar_output", "meas", "对目标的角度和距离进行测量功能测试"),
       title="图 31 量测误差与 CRLB 标度")
def fig31(D):
    m = D["radar_output"]["meas"].sort_values("t", kind="stable")
    kf = (D.get("radar_output") or {}).get("kf")
    aligned = align_same_rows(m, kf) if kf is not None else None
    if aligned is not None:
        m = aligned
        src_note = "量测表无目标号，已按 kf 表同序（(t, cyc) 逐行一致）恢复"
    else:
        src_note = "量测表无目标号且无法与 kf 同序对齐，按整批统计（不拆分目标）"
    groups = by_target(m)
    ids = [t for t, _ in groups]
    cmap = color_map(ids)
    mmap = marker_map(ids)

    dr_all = m["dr"].to_numpy(float)
    rng_all = m["rng"].to_numpy(float) / 1e3

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 31  距离/角度量测误差与距离标度（按目标）",
                f"量测 {len(m)} 点（{'、'.join(f'{target_title(t)} {len(g)} 点' for t, g in groups)}），"
                f"斜距 {_rng_txt(rng_all, ' km', 0)}；{src_note}；"
                f"误差用稳健统计（中位/95% 分位），野值不参与 σ 估计")

    ax = axes[0, 0]
    for tid, g in groups:
        col = cmap[int(tid)]
        t = g["t"].to_numpy(float)
        dr = g["dr"].to_numpy(float)
        ax.plot(t, dr / 1e3, lw=0.5, color=col, alpha=0.55,
                label=f"{target_title(tid)}（{len(g)} 点）")
        ax.plot(t, pd.Series(np.abs(dr) / 1e3).rolling(45, min_periods=5).max(),
                lw=1.3, color=col)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("斜距误差 (km)")
    ax.set_title("(a) 斜距误差时序（细线逐周期，粗线 45 s 滑动峰值）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.6)
    style_axes(ax)

    ax = axes[0, 1]
    allang = np.concatenate([np.abs(g["daz"].to_numpy(float))
                             for _, g in groups]
                            + [np.abs(g["del"].to_numpy(float))
                               for _, g in groups])
    lo = float(np.nanpercentile(allang[allang > 0], 0.5)) if (allang > 0).any() else 0.0
    hi = float(np.nanpercentile(allang, 99.5))
    bins = np.linspace(lo if hi > lo else 0.0, hi if hi > 0 else 1.0, 40)
    notes = []
    for tid, g in groups:
        col = cmap[int(tid)]
        daz = g["daz"].to_numpy(float)
        dele = g["del"].to_numpy(float)
        ax.hist(daz, bins=bins, histtype="step", lw=1.4, color=col,
                label=f"{target_title(tid)} 方位")
        ax.hist(dele, bins=bins, histtype="step", lw=1.2, color=col, ls="--",
                label=f"{target_title(tid)} 俯仰")
        notes.append(f"{target_title(tid)}：方位 95% |e| "
                     f"{float(np.percentile(np.abs(daz),95)):.2f}°、俯仰 "
                     f"{float(np.percentile(np.abs(dele),95)):.2f}°")
    ax.set_xlabel("角度误差 (°)")
    ax.set_ylabel("样本数")
    ax.set_title("(b) 角度误差分布（实线方位 / 虚线俯仰）", fontsize=10.5)
    ax.annotate("\n".join(notes), xy=(0.97, 0.05), xycoords="axes fraction",
                fontsize=7.6, va="bottom", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=6.8)
    style_axes(ax)

    ax = axes[1, 0]
    nz_all = np.abs(dr_all) > 0
    for tid, g in groups:
        col = cmap[int(tid)]
        mk = mmap[int(tid)]
        rng = g["rng"].to_numpy(float) / 1e3
        absdr = np.abs(g["dr"].to_numpy(float))
        nz = absdr > 0
        ax.scatter(rng[nz], db20(absdr[nz]), s=5, color=col, marker=mk,
                   alpha=0.35, linewidths=0, label=target_title(tid))
        cx, med, lo, hi = _bin(rng[nz], db20(absdr[nz]), 14, 40)
        if cx.size >= 2:
            ax.plot(cx, med, marker=mk, ms=3.6, lw=1.3, color=col)
    cx, med, lo, hi = _bin(rng_all[nz_all], db20(np.abs(dr_all[nz_all])), 18, 40)
    good = (cx > 0) & np.isfinite(med)
    if good.sum() >= 2:
        base = int(np.argmax(good))
        ref = med[base] + 40.0 * (np.log10(cx) - np.log10(cx[base]))
        ax.plot(cx, ref, ls=":", lw=1.3, color=C["ok"],
                label="CRLB $R^2$ 参考（40 dB/十倍程）")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("|斜距误差| (dB re 1 m)")
    ax.set_title("(c) 误差–距离密度与分位包络（按目标）", fontsize=10.5)
    ax.annotate(f"零误差 {int((~nz_all).sum())}/{len(dr_all)} 条（20·lg 无定义，未画）\n"
                f"|e|>50 km 共 {int((np.abs(dr_all) > 5e4).sum())} 条",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=7.2)
    style_axes(ax)

    ax = axes[1, 1]
    for tid, g in groups:
        col = cmap[int(tid)]
        absdr = np.abs(g["dr"].to_numpy(float))
        nz = absdr > 0
        s = np.sort(absdr[nz])
        frac = (int((~nz).sum()) + np.arange(1, s.size + 1)) / len(absdr)
        if s.size:
            ax.plot(db20(s), frac, lw=1.6, color=col,
                    label=f"{target_title(tid)}（零误差 {float((~nz).mean())*100:.0f}%）")
        q50 = float(np.percentile(absdr, 50))
        ax.axvline(float(db20(max(q50, 1e-3))), color=col, ls="--", lw=0.9)
    ax.set_xlabel("|斜距误差| (dB re 1 m)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title("(d) 误差累积分布（按目标）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.4)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 32
@needs(("radar_ext", "scene", "目标场景接口测试"),
       ("radar_ext", "track_out", "雷达对外目标信息输出接口测试-航迹识别"),
       title="图 32 真值–航迹对比")
def fig32(D):
    scene = D["radar_ext"]["scene"]
    to = D["radar_ext"]["track_out"]

    groups = by_target(to)
    prepared = []
    n_excl = 0
    for tid, g in groups:
        truth = scene[scene["tid"] == tid].sort_values("t") if "tid" in scene else scene
        if len(truth) < 2:
            continue
        tt = truth["t"].to_numpy(float)
        lo, hi = radar_obs_span(D, tid)
        if not (np.isfinite(lo) and np.isfinite(hi)):
            lo, hi = float(tt.min()), float(tt.max())
        for trk in sorted(int(v) for v in g["track"].unique()):
            full = g[g["track"] == trk].sort_values("t")
            tf_all = full["t"].to_numpy(float)
            keep = ((tf_all >= tt.min()) & (tf_all <= tt.max())
                    & obs_mask(tf_all, lo, hi))
            n_excl += int((~keep).sum())
            if keep.sum() < 2:
                continue
            f = full[keep]
            tf = tf_all[keep]
            e3 = np.sqrt(
                (np.interp(tf, tt, truth["e0"].to_numpy(float)) - f["e0"].to_numpy(float)) ** 2
                + (np.interp(tf, tt, truth["e1"].to_numpy(float)) - f["e1"].to_numpy(float)) ** 2
                + (np.interp(tf, tt, truth["e2"].to_numpy(float)) - f["e2"].to_numpy(float)) ** 2)
            prepared.append(dict(tid=tid, trk=trk, f=f, truth=truth, e3=e3))
    if not prepared:
        return None

    tid_list = []
    for p in prepared:
        if p["tid"] not in tid_list:
            tid_list.append(p["tid"])
    cmap = color_map(tid_list)
    mmap = marker_map(tid_list)
    lss = ["--", ":", "-."]

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    clip_txt = ("仅统计观测窗内的航迹帧，观测停止后的外推段不画"
                if common.CLIP_TO_OBS else
                "观测窗裁剪已关闭，航迹含外推段（仍限真值时段）")
    title_block(fig, "图 32  真值–航迹对比与三维定位误差（按目标）",
                f"{len(groups)} 个目标（{'、'.join(target_title(t) for t in tid_list)}）、"
                f"{len(prepared)} 条航迹；{clip_txt}（剔除 {n_excl} 帧）；"
                f"不同目标按颜色区分")

    ax = axes[0, 0]
    for tid in tid_list:
        tr = next(p["truth"] for p in prepared if p["tid"] == tid)
        ax.plot(tr["e0"] / 1e3, tr["e1"] / 1e3, color=cmap[tid],
                lw=1.6, label=f"真值 {target_title(tid)}")
        for j, p in enumerate([q for q in prepared if q["tid"] == tid]):
            f = p["f"]
            ax.plot(f["e0"] / 1e3, f["e1"] / 1e3, color=cmap[tid],
                    lw=1.0, ls=lss[j % len(lss)],
                    label=f"{target_title(tid)} 航迹 {int(p['trk'])}")
    ax.set_xlabel("东向 (km)")
    ax.set_ylabel("北向 (km)")
    ax.set_title("(a) 地面投影", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.0, ncol=2)
    style_axes(ax)

    ax = axes[0, 1]
    for tid in tid_list:
        tr = next(p["truth"] for p in prepared if p["tid"] == tid)
        r = np.sqrt(tr["e0"] ** 2 + tr["e1"] ** 2 + tr["e2"] ** 2) / 1e3
        ax.plot(r, tr["e2"] / 1e3, color=cmap[tid], lw=1.4, ls="-")
        for j, p in enumerate([q for q in prepared if q["tid"] == tid]):
            f = p["f"]
            rf = np.sqrt(f["e0"] ** 2 + f["e1"] ** 2 + f["e2"] ** 2) / 1e3
            ax.plot(rf, f["e2"] / 1e3, color=cmap[tid], lw=1.0,
                    ls=lss[j % len(lss)])
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("高度 (km)")
    ax.set_title("(b) 距离–高度剖面（颜色同 (a)）", fontsize=10.5)
    style_axes(ax)

    ax = axes[1, 0]
    for tid in tid_list:
        for j, p in enumerate([q for q in prepared if q["tid"] == tid]):
            ax.plot(p["f"]["t"], db20(p["e3"]), color=cmap[tid], lw=1.2,
                    ls=lss[j % len(lss)],
                    label=f"{target_title(tid)} / 航迹 {int(p['trk'])}")
    ax.axhline(0.0, color=C["neutral"], ls=":", lw=0.9, label="0 dB = 1 m")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("三维定位误差 (dB re 1 m)")
    ax.set_title("(c) 误差时序（观测窗内）", fontsize=10.5)
    ax.annotate(f"零误差 {int(sum((p['e3'] <= 0).sum() for p in prepared))} 帧"
                "（20·lg 无定义，未画）",
                xy=(0.03, 0.04), xycoords="axes fraction", fontsize=8.0,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper left", fontsize=7.2, framealpha=0.92)
    style_axes(ax)

    ax = axes[1, 1]
    all_e = np.concatenate([p["e3"] for p in prepared])
    lev = db20(all_e)
    okv = np.isfinite(lev)
    edges = np.histogram_bin_edges(lev[okv], bins=40) if okv.any() else 40
    for tid in tid_list:
        e = np.concatenate([p["e3"] for p in prepared if p["tid"] == tid])
        lv = db20(e)
        ok = np.isfinite(lv)
        if ok.sum() >= 3:
            ax.hist(lv[ok], bins=edges, histtype="step", lw=1.4, color=cmap[tid],
                    label=f"{target_title(tid)}（{len(e)} 帧）")
            ax.axvline(float(db20(max(np.median(e), 1e-3))), color=cmap[tid],
                       ls="--", lw=0.9)
        else:
            ax.axvline(float(db20(max(np.median(e), 1e-3))), color=cmap[tid],
                       ls=":", lw=1.5,
                       label=f"{target_title(tid)}（{len(e)} 帧）")
    ax.set_xlabel("三维定位误差 (dB re 1 m)")
    ax.set_ylabel("航迹帧数")
    ax.set_title("(d) 误差分布（按目标叠加）", fontsize=10.5)
    ax.annotate(f"全体零误差 {int((all_e <= 0).sum())} 帧未入直方图\n"
                f"样本 {int(okv.sum())}/{all_e.size} 帧",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.0,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=7.8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 33
@needs(("radar_ext", "res", "雷达资源管理接口测试"),
       ("radar_output", "sched", "雷达调度功能测试"),
       title="图 33 雷达资源占用与任务构成")
def fig33(D):
    res = D["radar_ext"]["res"]
    sch = D["radar_output"]["sched"]

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    used = res["used"].to_numpy(float)
    free = res["free"].to_numpy(float)
    pool = used + free
    title_block(fig, "图 33  雷达资源占用、队列与任务构成",
                f"资源记录 {len(res)} 条，调度记录 {len(sch)} 条；"
                f"资源池 {float(np.median(pool)):.0f} 单元，"
                f"最大占用 {float(np.max(used)):.0f} 单元；"
                f"资源与调度为雷达级汇总，逐条记录不带目标号")

    ax = axes[0, 0]
    cols = ["n_search", "n_desig", "n_ext", "n_lib", "n_conf"]
    labels = ["搜索", "确认", "跟踪", "制导", "识别"]
    data = [sch[c].to_numpy(float) for c in cols]
    ax.stackplot(sch["t"], *data, labels=labels,
                 colors=[CYCLE[k] for k in range(len(cols))], alpha=0.85)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("每周期任务数")
    ax.set_title("(a) 任务构成", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.4, ncol=3)
    style_axes(ax)

    ax = axes[0, 1]
    ax.fill_between(res["t"], used, step="post", color=C["accent2"], alpha=0.75,
                    label="占用")
    ax.plot(res["t"], used, drawstyle="steps-post", lw=0.8,
            color=C["accent2"])
    ax.set_ylim(0, max(1.0, float(np.max(used)) * 1.35))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("占用单元数")
    ax.set_title("(b) 资源池占用（纵轴按实际占用量缩放）", fontsize=10.5)
    ax.annotate(f"空闲 {float(np.median(free)):.0f}/{float(np.median(pool)):.0f} 单元\n"
                f"占用率峰值 {float(np.max(used))/float(np.median(pool))*100:.1f}%",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    util = used / np.clip(used + free, 1e-9, None)
    ax.plot(res["t"], util * 100.0, lw=0.9, color=C["accent3"])
    ax.axhline(float(np.median(util)) * 100.0, color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {float(np.median(util))*100:.1f}%")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("占用率 (%)")
    ax.set_title("(c) 资源占用率", fontsize=10.5)
    ax.legend(loc="best", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    qn = res["queue"].astype(str).apply(lambda s: 0 if s.strip() in ("", "miss")
                                        else len([x for x in s.split(";") if x.strip()]))
    ax.hist(qn.to_numpy(float), bins=np.arange(-0.5, qn.max() + 1.5, 1.0),
            color=C["accent4"], alpha=0.8, edgecolor="white", linewidth=0.4)
    ax.set_xlabel("排队任务数")
    ax.set_ylabel("记录数")
    ax.set_title("(d) 调度队列长度", fontsize=10.5)
    ax.annotate(f"最大 {int(qn.max())}，非零 {int((qn>0).sum())}/{len(qn)} 条",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.4,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 34
@needs(("radar_output", "beam", "波束扫描功能测试"),
       title="图 34 天线方向图与波束宽度")
def fig34(D):
    az, el, g = extract.antenna_pattern()
    az = np.asarray(az, float)
    el = np.asarray(el, float)
    g = np.asarray(g, float)
    beam = D["radar_output"]["beam"]

    # 视轴在 0°；0..360 折叠到 -180..180 并去重。
    # 方位 |az|>90° 的整行为常数（截断底，不随俯仰变化），不参与切面统计。
    azw = (az + 180.0) % 360.0 - 180.0
    order = np.argsort(azw, kind="stable")
    azw = azw[order]
    gw = g[order, :]
    keep = np.concatenate(([True], np.diff(azw) > 1e-9))
    azw, gw = azw[keep], gw[keep, :]
    row_ptp = np.nanmax(gw, axis=1) - np.nanmin(gw, axis=1)
    valid = row_ptp > 1e-9
    az_lo, az_hi = float(azw[valid].min()), float(azw[valid].max())

    gv = np.where(valid[:, None], gw, np.nan)
    ia, ie = np.unravel_index(int(np.nanargmax(gv)), gv.shape)
    peak = float(gv[ia, ie])
    a0, e0 = float(azw[ia]), float(el[ie])
    cut_az = gv[:, ie]
    cut_el = gw[ia, :]

    def _hpbw(axis, cut, pk, drop=3.0):
        """主瓣连续段上的 -3 dB 宽度（线性插值到门限）。"""
        lvl = float(cut[pk]) - drop
        above = np.isfinite(cut) & (cut >= lvl)
        if not above[pk]:
            return np.nan, None
        lo = pk
        while lo > 0 and above[lo - 1]:
            lo -= 1
        hi = pk
        while hi < cut.size - 1 and above[hi + 1]:
            hi += 1
        x0, x1 = float(axis[lo]), float(axis[hi])
        if lo > 0 and np.isfinite(cut[lo - 1]) and cut[lo] != cut[lo - 1]:
            f = (lvl - cut[lo - 1]) / (cut[lo] - cut[lo - 1])
            x0 = axis[lo - 1] + f * (axis[lo] - axis[lo - 1])
        if hi < cut.size - 1 and np.isfinite(cut[hi + 1]) and cut[hi + 1] != cut[hi]:
            f = (lvl - cut[hi]) / (cut[hi + 1] - cut[hi])
            x1 = axis[hi] + f * (axis[hi + 1] - axis[hi])
        return abs(x1 - x0), (lo, hi)

    hp_az, seg_az = _hpbw(azw, cut_az, ia)
    hp_el, seg_el = _hpbw(el, cut_el, ie)

    def _sll(cut, seg):
        """主瓣（含 2 点余量）之外的最高电平。"""
        if seg is None:
            return np.nan
        lo, hi = seg
        m = np.isfinite(cut).copy()
        m[max(0, lo - 2):hi + 3] = False
        return float(np.nanmax(cut[m])) if m.any() else np.nan

    sll_az = _sll(cut_az, seg_az)
    sll_el = _sll(cut_el, seg_el)

    def _env(axis, cut, step=1.0):
        """按 step 分箱取峰值包络，抑制 0.1° 采样造成的密集条纹。"""
        lo = float(np.floor(np.nanmin(axis)))
        hi = float(np.ceil(np.nanmax(axis)))
        edges = np.arange(lo, hi + step, step)
        idx = np.clip(np.digitize(axis, edges) - 1, 0, edges.size - 2)
        cx, cy = [], []
        for k in range(edges.size - 1):
            m = (idx == k) & np.isfinite(cut)
            if m.any():
                cx.append(0.5 * (edges[k] + edges[k + 1]))
                cy.append(float(np.nanmax(cut[m])))
        return np.asarray(cx), np.asarray(cy)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 34  天线方向图、主瓣宽度与副瓣电平",
                f"峰值 {peak:.2f} dBi @ ({a0:.1f}°, {e0:.1f}°)；"
                f"3 dB 波束宽度 方位 {hp_az:.2f}° / 俯仰 {hp_el:.2f}°；"
                f"宽角切面按 1° 分箱取峰值包络；"
                f"方向图为雷达级特性，不按目标拆分")

    ax = axes[0, 0]
    ka = max(1, int(round(0.5 / max(abs(azw[1] - azw[0]), 1e-9))))
    ke = max(1, int(round(0.5 / max(abs(el[1] - el[0]), 1e-9))))
    na = (gv.shape[0] // ka) * ka
    ne = (gv.shape[1] // ke) * ke
    gm = gv[:na, :ne].reshape(na // ka, ka, ne // ke, ke).max(axis=(1, 3))
    az_m = azw[:na].reshape(-1, ka).mean(axis=1)
    el_m = el[:ne].reshape(-1, ke).mean(axis=1)
    pc = ax.pcolormesh(az_m, el_m, gm.T, cmap="viridis", shading="auto")
    cb = fig.colorbar(pc, ax=ax, pad=0.02)
    cb.set_label("增益 (dBi)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlim(az_lo - 2, az_hi + 2)
    ax.set_xlabel("方位偏角 (°)")
    ax.set_ylabel("俯仰偏角 (°)")
    ax.set_title("(a) 有效扇区方向图（0.5° 峰值包络）", fontsize=10.5)
    style_axes(ax, minor=False)

    ax = axes[0, 1]
    ax.plot(azw, cut_az, ".", ms=1.2, color=C["accent1"], alpha=0.25)
    ex, ey = _env(azw, cut_az, 1.0)
    ax.plot(ex, ey, color=C["accent1"], lw=1.4, label="1° 峰值包络")
    ax.axhline(peak - 3, color=C["warn"], ls="--", lw=0.9, label="-3 dB")
    if np.isfinite(sll_az):
        ax.axhline(sll_az, color=C["accent4"], ls=":", lw=1.0,
                   label=f"最高副瓣 {sll_az:.1f} dBi")
    ax.set_xlim(az_lo - 1, az_hi + 1)
    ax.set_xlabel("方位偏角 (°)")
    ax.set_ylabel("增益 (dBi)")
    ax.set_title(f"(b) 方位切面（俯仰 {e0:.1f}°）", fontsize=10.5)
    ax.annotate(f"3 dB 宽度 {hp_az:.2f}°\n副瓣电平 {sll_az-peak:+.1f} dB",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.2,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    ax.plot(el, cut_el, ".", ms=1.2, color=C["accent2"], alpha=0.25)
    ex, ey = _env(el, cut_el, 1.0)
    ax.plot(ex, ey, color=C["accent2"], lw=1.4, label="1° 峰值包络")
    ax.axhline(peak - 3, color=C["warn"], ls="--", lw=0.9, label="-3 dB")
    if np.isfinite(sll_el):
        ax.axhline(sll_el, color=C["accent4"], ls=":", lw=1.0,
                   label=f"最高副瓣 {sll_el:.1f} dBi")
    ax.set_xlabel("俯仰偏角 (°)")
    ax.set_ylabel("增益 (dBi)")
    ax.set_title(f"(c) 俯仰切面（方位 {a0:.1f}°）", fontsize=10.5)
    ax.annotate(f"3 dB 宽度 {hp_el:.2f}°\n副瓣电平 {sll_el-peak:+.1f} dB",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.2,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    zoom = 5.0
    ma = np.abs(azw - a0) <= zoom
    me = np.abs(el - e0) <= zoom
    ax.plot(azw[ma] - a0, cut_az[ma] - peak, color=C["accent1"], lw=1.4,
            label="方位切面")
    ax.plot(el[me] - e0, cut_el[me] - peak, color=C["accent2"], lw=1.4,
            label="俯仰切面")
    for lv in (-3.0, -10.0, -20.0):
        ax.axhline(lv, color=C["neutral"], ls=":", lw=0.8)
        ax.text(zoom * 0.96, lv + 0.8, f"{lv:.0f} dB", ha="right", fontsize=7.2,
                color=C["neutral"])
    ax.set_xlim(-zoom, zoom)
    ax.set_ylim(-30, 2)
    ax.set_xlabel("相对视轴偏角 (°)")
    ax.set_ylabel("相对增益 (dB)")
    ax.set_title(f"(d) 主瓣归一化剖面（±{zoom:.0f}°）", fontsize=10.5)
    ax.annotate(f"扫描驻留 {len(beam)} 条\n"
                f"方位/俯仰 HPBW {hp_az:.2f}° / {hp_el:.2f}°",
                xy=(0.03, 0.92), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 35
@needs(("radar_output", "hrrp", "目标一维距离像计算功能测试"),
       title="图 35 HRRP 距离像")
def fig35(D):
    h = D["radar_output"]["hrrp"].sort_values("t")

    def _parse(s):
        out = []
        for tok in str(s).split(";"):
            tok = tok.strip().strip("()")
            if not tok:
                continue
            parts = tok.split(",")
            if len(parts) != 2:
                continue
            try:
                out.append((float(parts[0].replace("m", "")),
                            float(parts[1].replace("dBsm", ""))))
            except ValueError:
                continue
        return out

    has_tid = "tid" in h.columns
    has_trk = "track" in h.columns
    rows = []
    for _, r in h.iterrows():
        tid = int(r["tid"]) if has_tid else None
        trk = int(r["track"]) if has_trk else None
        for pos, amp in _parse(r["centers"]):
            rows.append((float(r["t"]), pos, amp,
                         int(r.get("npeaks", 0) or 0), tid, trk))
    if not rows:
        return None
    tab = pd.DataFrame(rows, columns=["t", "pos", "amp", "npeaks", "tid", "track"])
    if has_tid:
        groups = by_target(h)
    else:
        groups = [(None, h)]
    ids = [t for t, _ in groups]
    cmap = color_map(ids)
    mmap = marker_map(ids)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    n_single = int((h["npeaks"].to_numpy(float) == 1).sum())
    n_double = int((h["npeaks"].to_numpy(float) == 2).sum())
    head = "、".join(f"{target_title(t)} {len(g)} 帧" for t, g in groups) if has_tid else "未记录目标号"
    title_block(fig, "图 35  HRRP 距离像：散射中心位置、幅度与间距",
                f"距离像 {len(h)} 帧（{head}），散射中心 {len(tab)} 个；"
                f"单峰 {n_single} 帧 / 双峰 {n_double} 帧；颜色与标记按目标区分")

    ax = axes[0, 0]
    amp = tab["amp"].to_numpy(float)
    lo, hi = float(np.nanmin(amp)), float(np.nanmax(amp))
    size = 10.0 + 26.0 * (amp - lo) / max(hi - lo, 1e-9)
    for tid in ids:
        sub = tab if tid is None else tab[tab["tid"] == tid]
        if not len(sub):
            continue
        col = cmap[int(tid)] if tid is not None else C["accent1"]
        mk = mmap.get(int(tid), "o") if tid is not None else "o"
        ax.scatter(sub["t"], sub["pos"], s=size[sub.index], color=col, marker=mk,
                   alpha=0.7, linewidths=0,
                   label=target_title(tid) if tid is not None else "全部距离像")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("散射中心位置 (m)")
    ax.set_title("(a) 距离像随时间的散射中心（点大小∝幅度）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.8, markerscale=1.2)
    style_axes(ax)

    ax = axes[0, 1]
    ups = np.unique(np.round(tab["pos"].to_numpy(float), 3))
    n = max(len(ids), 1)
    bw = 0.5 / n
    for j, tid in enumerate(ids):
        sub = tab if tid is None else tab[tab["tid"] == tid]
        if not len(sub):
            continue
        col = cmap[int(tid)] if tid is not None else C["accent1"]
        off = (j - (n - 1) / 2.0) * bw
        cy = [float(np.median(sub.loc[np.isclose(sub["pos"], p, atol=1e-3), "amp"]))
              for p in ups]
        ax.bar(np.arange(len(ups)) + off, cy, width=bw * 0.9, color=col,
               alpha=0.85,
               label=f"{target_title(tid)}（{len(sub)} 个中心）" if tid is not None
               else "全部距离像")
    ax.set_xticks(range(len(ups)))
    ax.set_xticklabels([f"{p:.2f}" for p in ups])
    ax.set_xlabel("距离 (m)")
    ax.set_ylabel("中位幅度 (dBsm)")
    ax.set_title("(b) 平均距离像（按散射中心位置 × 目标）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.8)
    style_axes(ax)

    ax = axes[1, 0]
    lss = ["-", "--", "-.", ":"]
    for tid in ids:
        sub = tab if tid is None else tab[tab["tid"] == tid]
        if not len(sub):
            continue
        col = cmap[int(tid)] if tid is not None else C["accent1"]
        for k, p in enumerate(sorted(sub["pos"].unique())):
            s = sub[sub["pos"] == p].sort_values("t")
            ax.plot(s["t"], s["amp"], lw=0.9, color=col,
                    ls=lss[k % len(lss)],
                    label=(f"{target_title(tid)} · {p:.2f} m"
                           if tid is not None else f"{p:.2f} m"))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("幅度 (dBsm)")
    ax.set_title("(c) 各散射中心幅度时序（颜色＝目标，线型＝位置）", fontsize=10.5)
    ax.legend(loc="best", fontsize=6.8, ncol=2, title="目标 · 位置")
    style_axes(ax)

    ax = axes[1, 1]
    lenses = h["length"].to_numpy(float)
    edges = np.histogram_bin_edges(lenses, bins=np.arange(np.nanmin(lenses) - 0.25,
                                                          np.nanmax(lenses) + 0.35,
                                                          0.25))
    for k, (tid, g) in enumerate(groups):
        col = cmap[int(tid)] if tid is not None else C["accent3"]
        ln = g["length"].to_numpy(float)
        ax.hist(ln, bins=edges, histtype="step", lw=1.5, color=col,
                label=f"{target_title(tid)}（{len(ln)} 帧）")
        npk = g["npeaks"].to_numpy(float)
        ax.annotate(f"{target_title(tid)}：单峰 {int((npk == 1).sum())} / "
                    f"双峰 {int((npk == 2).sum())} 帧",
                    xy=(0.03, 0.94 - 0.07 * k),
                    xycoords="axes fraction", fontsize=7.6, va="top")
    ax.set_xlabel("距离像长度 (m)")
    ax.set_ylabel("帧数")
    ax.set_title("(d) 距离像长度分布（按目标叠加）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.6)
    style_axes(ax)
    return fig


FIGURES = [
    ("26", "fig26_pd_snr_family", "图 26 检测性能曲线族", fig26),
    ("27", "fig27_range_doppler", "图 27 距离–多普勒量测平面", fig27),
    ("28", "fig28_noise_cfar", "图 28 噪声分布与 CFAR 门限", fig28),
    ("29", "fig29_mti_mtd_gain", "图 29 MTI/MTD 处理链增益", fig29),
    ("30", "fig30_js_suppression", "图 30 J/S 与干扰压制", fig30),
    ("31", "fig31_meas_crlb", "图 31 量测误差与 CRLB 标度", fig31),
    ("32", "fig32_truth_track", "图 32 真值–航迹对比", fig32),
    ("33", "fig33_resource_mgmt", "图 33 雷达资源占用与任务构成", fig33),
    ("34", "fig34_antenna_scan", "图 34 天线方向图与波束宽度", fig34),
    ("35", "fig35_hrrp_profile", "图 35 HRRP 距离像", fig35),
]
