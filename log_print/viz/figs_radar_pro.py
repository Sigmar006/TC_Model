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
from scipy.stats import ncx2

from . import extract
from .common import (C, CYCLE, by_target, db10, db20, needs, style_axes,
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

    snr = p["snr"].to_numpy(float)
    pdv = p["pd"].to_numpy(float)
    rng = p["rng"].to_numpy(float) / 1e3
    thr = float(det["thr"].median())

    cxs, med, lo, hi = _bin(snr, pdv, 40, 15)
    pfa = _fit_pfa(snr, pdv)
    xs = np.linspace(float(np.nanmin(snr)), float(np.nanmax(snr)), 300)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 26  检测性能曲线族：$P_d$–SNR、$P_d$–R 与 90% 作用距离",
                f"点迹 {len(p)} 条，SNR {_rng_txt(snr, ' dB')}，"
                f"检测门限 {thr:.2f} dB；"
                + (f"拟合等效 $P_{{fa}}$ = {pfa:.2e}" if np.isfinite(pfa)
                   else "样本不足，未拟合 $P_{fa}$"))

    ax = axes[0, 0]
    ax.scatter(snr, pdv, s=6, color=C["neutral"], alpha=0.35, linewidths=0,
               label="逐点迹")
    if cxs.size:
        ax.plot(cxs, med, color=C["accent1"], lw=1.8, label="分箱中位")
        ax.fill_between(cxs, lo, hi, color=C["accent1"], alpha=0.16, lw=0,
                        label="10–90% 分位")
    if np.isfinite(pfa):
        ax.plot(xs, _swelling_i(xs, pfa), color=C["warn"], lw=1.5, ls="--",
                label="Swerling I/II 拟合")
        ax.plot(xs, _nonfluct(xs, pfa), color=C["accent4"], lw=1.5, ls=":",
                label="非起伏目标（同 $P_{fa}$）")
    ax.axvline(thr, color=C["ok"], lw=0.9, ls="-.", label=f"门限 {thr:.1f} dB")
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("检测概率 $P_d$")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("(a) $P_d$–SNR 与理论曲线", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.6)
    style_axes(ax)

    ax = axes[0, 1]
    sc = ax.scatter(rng, pdv, c=snr, s=7, cmap="viridis", alpha=0.75,
                    linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("SNR (dB)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    cxr, medr, lor, hir = _bin(rng, pdv, 36, 15)
    if cxr.size:
        ax.plot(cxr, medr, color=C["warn"], lw=1.6, label="分箱中位")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("检测概率 $P_d$")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("(b) $P_d$–距离（颜色为 SNR）", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    ax.plot(cfar["t"], cfar["r90"] / 1e3, color=C["accent3"], lw=1.5)
    ax.fill_between(cfar["t"], cfar["r90"] / 1e3, color=C["accent3"], alpha=0.12)
    ax.axhline(float(np.median(cfar["r90"])) / 1e3, color=C["warn"], ls="--",
               lw=1.0, label=f"中位 {float(np.median(cfar['r90']))/1e3:.0f} km")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("$P_d=90\\%$ 作用距离 (km)")
    ax.set_title("(c) CFAR 90% 检测距离", fontsize=10.5)
    ax.legend(loc="best", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    margin = snr - thr
    ax.hist(margin, bins=48, color=C["accent1"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.axvline(0.0, color=C["warn"], lw=1.1, ls="--", label="门限")
    ax.set_xlabel("SNR 余量（SNR − 门限） (dB)")
    ax.set_ylabel("点迹数")
    ax.set_title("(d) 检测余量分布", fontsize=10.5)
    ax.annotate(f"中位 {float(np.median(margin)):.1f} dB\n"
                f"低于门限 {int((margin < 0).sum())}/{len(margin)} 条",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.4,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 27
@needs(("radar_int", "echo_sp", "雷达回波信号处理交互接口测试"),
       ("radar_ext", "plot", "雷达对外目标信息输出接口测试-点迹量测"),
       title="图 27 距离–多普勒量测平面")
def fig27(D):
    e = D["radar_int"]["echo_sp"]
    p = D["radar_ext"]["plot"]

    rng = e["rng"].to_numpy(float) / 1e3
    dop = e["dop_out"].to_numpy(float) / 1e3
    prf = e["prf"].to_numpy(float)
    pout = db10(e["p_out"].to_numpy(float))
    delay = e["delay_out"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 27  距离–多普勒量测平面与距离标定",
                f"回波 {len(e)} 条，斜距 {_rng_txt(rng, ' km', 0)}，"
                f"多普勒 {_rng_txt(dop, ' kHz', 1)}；"
                f"PRF 记录值 {_rng_txt(prf, ' Hz', 1)}")

    ax = axes[0, 0]
    sc = ax.scatter(rng, dop, c=pout, s=18, cmap="magma", alpha=0.9,
                    edgecolors="white", linewidths=0.3)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("回波功率 (dBW)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("多普勒频率 (kHz)")
    ax.set_title("(a) 距离–多普勒点迹", fontsize=10.5)
    style_axes(ax)

    ax = axes[0, 1]
    vr = p["vr"].to_numpy(float)
    pdv = p["pd"].to_numpy(float)
    sc = ax.scatter(p["rng"].to_numpy(float) / 1e3, vr, c=pdv, s=7,
                    cmap="viridis", alpha=0.75, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("$P_d$", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("径向速度 $v_r$ (m/s)")
    ax.set_title("(b) 点迹距离–径向速度", fontsize=10.5)
    style_axes(ax)

    ax = axes[1, 0]
    ax.hist(dop, bins=40, color=C["accent1"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.axvline(float(np.median(dop)), color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {float(np.median(dop)):.1f} kHz")
    ax.set_xlabel("多普勒频率 (kHz)")
    ax.set_ylabel("回波数")
    ax.set_title("(c) 多普勒分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    r_cal = C_LIGHT * delay / 2.0 / 1e3
    ax.scatter(rng, r_cal, s=10, color=C["accent2"], alpha=0.7, linewidths=0)
    lim = [min(float(np.nanmin(rng)), float(np.nanmin(r_cal))),
           max(float(np.nanmax(rng)), float(np.nanmax(r_cal)))]
    ax.plot(lim, lim, color=C["neutral"], lw=0.9, ls="--", label="1:1")
    rel = np.abs(r_cal - rng) / np.clip(rng, 1e-9, None)
    ax.set_xlabel("日志斜距 (km)")
    ax.set_ylabel("$c\\,\\Delta t/2$ (km)")
    ax.set_title("(d) 时延–距离一致性", fontsize=10.5)
    ax.annotate(f"中位相对偏差 {float(np.median(rel))*100:.2f}%\n"
                f"最大 {float(np.max(rel))*100:.2f}%",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.4,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 28
@needs(("radar_int", "det", "雷达信号处理与检测判决交互接口测试"),
       ("radar_output", "noise", "噪声与杂波计算功能测试"),
       title="图 28 噪声分布与 CFAR 门限")
def fig28(D):
    det = D["radar_int"]["det"].sort_values("t")
    noise = D["radar_output"]["noise"]

    pn = det["p_noise"].to_numpy(float)
    pn = pn[np.isfinite(pn) & (pn > 0)]
    pn_db = db10(pn)
    thr = float(det["thr"].median())

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 28  噪声功率分布、CFAR 门限与多普勒通道噪声底",
                f"检测记录 {len(det)} 条，噪声样本 {len(pn)} 个，"
                f"噪声中位 {float(np.median(pn_db)):.1f} dBW，SNR 门限 {thr:.2f} dB")

    ax = axes[0, 0]
    s = np.sort(pn)
    f = np.arange(1, s.size + 1) / s.size
    mean_p = float(np.mean(s))
    s_db = db10(s)
    grid = np.linspace(float(np.min(s_db)), float(np.max(s_db)), 300)
    ax.plot(s_db, f, drawstyle="steps-post", color=C["accent1"], lw=1.3,
            label="实测 CDF")
    ax.plot(grid, 1.0 - np.exp(-(10.0 ** (grid / 10.0)) / mean_p),
            color=C["warn"], lw=1.4, label="指数分布拟合")
    ax.set_xlabel("噪声功率 (dBW)")
    ax.set_ylabel("累积概率")
    ax.set_title("(a) 噪声功率 CDF", fontsize=10.5)
    ax.annotate(f"样本 {len(pn)} 个；中位 {float(np.median(pn_db)):.1f} dBW",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)

    ax = axes[0, 1]
    ax.hist(pn_db, bins=44, density=True, color=C["accent1"], alpha=0.8,
            edgecolor="white", linewidth=0.3)
    y = np.linspace(float(np.nanmin(pn_db)), float(np.nanmax(pn_db)), 300)
    p_lin = 10.0 ** (y / 10.0)
    pdf_db = np.log(10.0) / 10.0 * p_lin / mean_p * np.exp(-p_lin / mean_p)
    ax.plot(y, pdf_db, color=C["warn"], lw=1.4, label="指数分布拟合")
    ax.set_xlabel("噪声功率 (dBW)")
    ax.set_ylabel("概率密度")
    ax.set_title("(b) 噪声功率 PDF", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    snr = det["snr_out"].to_numpy(float)
    ax.hist(snr, bins=40, color=C["accent3"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.axvline(thr, color=C["warn"], ls="--", lw=1.1,
               label=f"SNR 门限 {thr:.1f} dB")
    ax.set_xlabel("检测 SNR (dB)")
    ax.set_ylabel("检测记录数")
    ax.set_title("(c) 检测 SNR 分布与门限", fontsize=10.5)
    ax.annotate(f"低于门限 {int((snr < thr).sum())}/{len(snr)} 条\n"
                f"中位 {float(np.median(snr)):.1f} dB",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    ch = noise["ch"].astype(str)
    vals = []
    for srow in ch:
        try:
            vals.append([float(x) for x in srow.split(",") if x.strip()])
        except ValueError:
            continue
    if vals:
        ncol = min(len(v) for v in vals)
        mat = np.asarray([v[:ncol] for v in vals], dtype=float)
        data = [db10(mat[:, k]) for k in range(ncol)]
        bp = ax.boxplot(data, showfliers=False, patch_artist=True, widths=0.62)
        for k, box in enumerate(bp["boxes"]):
            box.set_facecolor(CYCLE[k % len(CYCLE)])
            box.set_alpha(0.72)
            box.set_edgecolor("none")
        ax.set_xticks(range(1, ncol + 1))
        ax.set_xticklabels([f"CH{k+1}" for k in range(ncol)], fontsize=8)
        ax.set_xlabel("多普勒通道")
        ax.set_ylabel("噪声功率 (dBW)")
        ax.set_title("(d) MTD 各通道噪声底", fontsize=10.5)
        med_db = db10(np.nanmedian(mat, axis=0))
        spread = float(np.nanmax(med_db) - np.nanmin(med_db))
        ax.annotate(f"{len(mat)} 帧 × {ncol} 通道\n"
                    + ("各通道噪声底一致" if spread < 1e-6
                       else f"通道间极差 {spread:.1f} dB"),
                    xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.2,
                    va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                           fc="#f7f9fb", ec="#c8d0d8"))
    else:
        ax.text(0.5, 0.5, "无可解析的通道噪声", ha="center", va="center",
                transform=ax.transAxes, color=C["neutral"])
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

    pc = g["pc"].to_numpy(float)
    mti = g["mti"].to_numpy(float)
    mtd = g["mtd"].to_numpy(float)
    pb = db10(g["p_before"].to_numpy(float))
    pa = db10(g["p_after"].to_numpy(float))
    total = pc + mti + mtd
    suppr = jg["suppr"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 29  脉压 / MTI / MTD 逐级增益与干扰抑制",
                f"处理记录 {len(g)} 条，链路 {g['chain'].iloc[0] if len(g) else '未记录'}；"
                f"总增益中位 {float(np.median(total)):.1f} dB")

    ax = axes[0, 0]
    names = ["脉压", "MTI", "MTD"]
    data = [pc, mti, mtd]
    meds = [float(np.median(d)) for d in data]
    los = [float(np.percentile(d, 10)) for d in data]
    his = [float(np.percentile(d, 90)) for d in data]
    x = np.arange(len(names))
    ax.bar(x, meds, width=0.55, color=[CYCLE[k] for k in range(3)], alpha=0.82)
    ax.errorbar(x, meds, yerr=[np.asarray(meds) - np.asarray(los),
                               np.asarray(his) - np.asarray(meds)],
                fmt="none", ecolor="#2c3e50", capsize=4, lw=1.0)
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("增益 (dB)")
    ax.set_title("(a) 各级增益中位与 10–90% 分位", fontsize=10.5)
    ax.annotate(f"脉压 {meds[0]:.1f} dB\n"
                f"MTI {meds[1]:.1f} dB（{int((mti<0).sum())} 次为负）\n"
                f"MTD {meds[2]:.1f} dB",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.grid(axis="x", visible=False)
    style_axes(ax)

    ax = axes[0, 1]
    ax.scatter(pb, pa, s=12, color=C["accent1"], alpha=0.65, linewidths=0)
    lim = [min(float(np.nanmin(pb)), float(np.nanmin(pa))),
           max(float(np.nanmax(pb)), float(np.nanmax(pa)))]
    ax.plot(lim, lim, color=C["neutral"], ls="--", lw=0.9, label="1:1")
    ax.set_xlabel("处理前功率 (dBW)")
    ax.set_ylabel("处理后功率 (dBW)")
    ax.set_title("(b) 处理前后功率", fontsize=10.5)
    ax.annotate(f"实测增益中位 {float(np.nanmedian(pa-pb)):.1f} dB\n"
                f"{_rng_txt(pa-pb, ' dB', 1)}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.4,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    ax.hist(total, bins=36, color=C["accent3"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.axvline(float(np.median(total)), color=C["warn"], ls="--", lw=1.1,
               label=f"中位 {float(np.median(total)):.1f} dB")
    ax.set_xlabel("链路总增益 (dB)")
    ax.set_ylabel("处理记录数")
    ax.set_title("(c) 总增益分布", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    ax.hist(suppr, bins=36, color=C["accent2"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.axvline(float(np.median(suppr)), color=C["warn"], ls="--", lw=1.1,
               label=f"中位 {float(np.median(suppr)):.1f} dB")
    ax.set_xlabel("干扰抑制比 (dB)")
    ax.set_ylabel("处理记录数")
    ax.set_title("(d) MTI 后干扰抑制比", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
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

    js = db10(e["p_jam"].to_numpy(float) / np.clip(e["p_echo"].to_numpy(float),
                                                   1e-30, None))
    rng = e["rng"].to_numpy(float) / 1e3
    t = e["t"].to_numpy(float)
    suppr = jg["suppr"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 30  干信比、压制比与干扰源功率",
                f"回波 {len(e)} 条，J/S {_rng_txt(js, ' dB', 1)}；"
                f"干扰源 {jp['src'].nunique()} 个")

    ax = axes[0, 0]
    sc = ax.scatter(rng, js, c=t, s=16, cmap="viridis", alpha=0.85, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("仿真时间 (s)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("J/S (dB)")
    ax.set_title("(a) 干信比随距离", fontsize=10.5)
    style_axes(ax)

    ax = axes[0, 1]
    ax.plot(t, js, lw=0.8, color=C["accent2"], alpha=0.75)
    ax.axhline(float(np.median(js)), color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {float(np.median(js)):.1f} dB")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("J/S (dB)")
    ax.set_title("(b) 干信比时序", fontsize=10.5)
    ax.legend(loc="best", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    ax.plot(jg["t"].to_numpy(float), suppr, lw=0.9, color=C["accent3"])
    ax.axhline(float(np.median(suppr)), color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {float(np.median(suppr)):.1f} dB")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("压制比 (dB)")
    ax.set_title("(c) MTI 压制比时序", fontsize=10.5)
    ax.legend(loc="best", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    srcs = sorted(jp["src"].dropna().unique())
    data = [db10(jp.loc[jp["src"] == s, "power"].to_numpy(float)) for s in srcs]
    ax.violinplot(data, showmedians=True, showextrema=False, widths=0.72)
    ax.set_xticks(range(1, len(srcs) + 1))
    ax.set_xticklabels([f"源 {int(s)}" for s in srcs])
    ax.set_ylabel("干扰功率 (dBW)")
    ax.set_title("(d) 干扰源功率分布", fontsize=10.5)
    ax.grid(axis="x", visible=False)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 31
@needs(("radar_output", "meas", "对目标的角度和距离进行测量功能测试"),
       title="图 31 量测误差与 CRLB 标度")
def fig31(D):
    m = D["radar_output"]["meas"].sort_values("t")
    dr = m["dr"].to_numpy(float)
    daz = m["daz"].to_numpy(float)
    dele = m["del"].to_numpy(float)
    rng = m["rng"].to_numpy(float) / 1e3
    t = m["t"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 31  距离/角度量测误差与距离标度",
                f"量测 {len(m)} 点，斜距 {_rng_txt(rng, ' km', 0)}；"
                f"误差用稳健统计（中位/95% 分位），野值不参与 σ 估计")

    ax = axes[0, 0]
    ax.plot(t, dr / 1e3, lw=0.5, color=C["accent1"], alpha=0.55)
    ax.plot(t, pd.Series(np.abs(dr) / 1e3).rolling(45, min_periods=5).max(),
            lw=1.3, color=C["warn"], label="45 s 滑动峰值")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("斜距误差 (km)")
    ax.set_title("(a) 斜距误差时序", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[0, 1]
    bins = np.linspace(float(np.nanpercentile(daz, 0.5)),
                       float(np.nanpercentile(daz, 99.5)), 40)
    ax.hist(daz, bins=bins, color=C["accent2"], alpha=0.62,
            edgecolor="white", linewidth=0.3, label="方位")
    bins = np.linspace(float(np.nanpercentile(dele, 0.5)),
                       float(np.nanpercentile(dele, 99.5)), 40)
    ax.hist(dele, bins=bins, color=C["accent4"], alpha=0.55,
            edgecolor="white", linewidth=0.3, label="俯仰")
    ax.set_xlabel("角度误差 (°)")
    ax.set_ylabel("样本数")
    ax.set_title("(b) 角度误差分布", fontsize=10.5)
    ax.annotate(f"方位 95% |e| {float(np.percentile(np.abs(daz),95)):.2f}°\n"
                f"俯仰 95% |e| {float(np.percentile(np.abs(dele),95)):.2f}°",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.4,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    absdr = np.abs(dr)
    nz = absdr > 0
    e_db = db20(absdr[nz])
    hb = ax.hexbin(rng[nz], e_db, gridsize=(40, 28),
                   cmap="viridis", mincnt=1)
    cb = fig.colorbar(hb, ax=ax, pad=0.02)
    cb.set_label("样本数", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    cx, med, lo, hi = _bin(rng[nz], e_db, 18, 25)
    if cx.size >= 2:
        ax.plot(cx, med, "o", ms=4.5, color=C["warn"], label="分箱中位")
        ax.plot(cx, hi, "x", ms=4.0, mew=1.0, color=C["accent4"],
                label="90% 分位")
        good = (cx > 0) & np.isfinite(med)
        if good.sum() >= 2:
            base = int(np.argmax(good))
            # CRLB 距离误差 ∝ R²：换算成 20·lg 后应为 40 dB/十倍程。
            ref = med[base] + 40.0 * (np.log10(cx) - np.log10(cx[base]))
            ax.plot(cx, ref, ls=":", lw=1.2, color=C["ok"],
                    label="CRLB $R^2$ 参考（40 dB/十倍程）")
            k = float(np.polyfit(np.log10(cx[good]), med[good], 1)[0])
            ax.annotate(f"零误差 {int((~nz).sum())}/{len(absdr)} 条（20·lg 无定义，未画）\n"
                        f"|e|>50 km 共 {int((absdr > 5e4).sum())} 条\n"
                        f"分箱中位斜率 {k:.0f} dB/十倍程（CRLB 参考 40）",
                        xy=(0.03, 0.94), xycoords="axes fraction",
                        fontsize=8.0, va="top",
                        bbox=dict(boxstyle="round,pad=0.35",
                                  fc="#f7f9fb", ec="#c8d0d8"))
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("|斜距误差| (dB re 1 m)")
    ax.set_title("(c) 误差–距离密度与分位包络", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.6, framealpha=0.92)
    style_axes(ax)

    ax = axes[1, 1]
    s = np.sort(absdr[nz])
    frac = (int((~nz).sum()) + np.arange(1, s.size + 1)) / len(absdr)
    ax.plot(db20(s), frac, lw=1.6, color=C["accent3"], label="非零误差 CDF")
    ax.axhline(float((~nz).mean()), color=C["neutral"], ls=":", lw=1.0,
               label=f"零误差累积 {float((~nz).mean())*100:.1f}%")
    q50 = float(np.percentile(absdr, 50))
    q95 = float(np.percentile(absdr, 95))
    ax.axvline(float(db20(max(q50, 1e-3))), color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {q50:.0f} m")
    ax.axvline(float(db20(max(q95, 1e-3))), color=C["accent4"], ls=":", lw=1.1,
               label=f"95% {q95/1e3:.1f} km")
    ax.set_xlabel("|斜距误差| (dB re 1 m)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title("(d) 误差累积分布", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.6)
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
        for trk in sorted(int(v) for v in g["track"].unique()):
            full = g[g["track"] == trk].sort_values("t")
            tf_all = full["t"].to_numpy(float)
            keep = (tf_all >= tt.min()) & (tf_all <= tt.max())
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

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 32  真值–航迹对比与三维定位误差",
                f"{len(groups)} 个目标、{len(prepared)} 条航迹；"
                f"仅统计真值时段内的航迹帧，超出段不外推"
                f"（剔除非共同时段 {n_excl} 帧）")

    ax = axes[0, 0]
    tid_list = []
    for p in prepared:
        if p["tid"] not in tid_list:
            tid_list.append(p["tid"])
    tid_color = {tid: CYCLE[k % len(CYCLE)] for k, tid in enumerate(tid_list)}
    lss = ["--", ":", "-."]
    for tid in tid_list:
        tr = next(p["truth"] for p in prepared if p["tid"] == tid)
        ax.plot(tr["e0"] / 1e3, tr["e1"] / 1e3, color=tid_color[tid],
                lw=1.6, label=f"真值 {target_title(tid)}")
        for j, p in enumerate([q for q in prepared if q["tid"] == tid]):
            f = p["f"]
            ax.plot(f["e0"] / 1e3, f["e1"] / 1e3, color=tid_color[tid],
                    lw=1.0, ls=lss[j % len(lss)], label=f"航迹 {int(p['trk'])}")
    ax.set_xlabel("东向 (km)")
    ax.set_ylabel("北向 (km)")
    ax.set_title("(a) 地面投影", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.0, ncol=2)
    style_axes(ax)

    ax = axes[0, 1]
    for tid in tid_list:
        tr = next(p["truth"] for p in prepared if p["tid"] == tid)
        r = np.sqrt(tr["e0"] ** 2 + tr["e1"] ** 2 + tr["e2"] ** 2) / 1e3
        ax.plot(r, tr["e2"] / 1e3, color=tid_color[tid], lw=1.4, ls="-")
        for j, p in enumerate([q for q in prepared if q["tid"] == tid]):
            f = p["f"]
            rf = np.sqrt(f["e0"] ** 2 + f["e1"] ** 2 + f["e2"] ** 2) / 1e3
            ax.plot(rf, f["e2"] / 1e3, color=tid_color[tid], lw=1.0,
                    ls=lss[j % len(lss)])
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("高度 (km)")
    ax.set_title("(b) 距离–高度剖面（颜色同 (a)）", fontsize=10.5)
    style_axes(ax)

    ax = axes[1, 0]
    for k, p in enumerate(prepared):
        ax.plot(p["f"]["t"], db20(p["e3"]),
                color=CYCLE[k % len(CYCLE)], lw=1.2,
                label=f"目标 {int(p['tid'])} / 航迹 {int(p['trk'])}")
    ax.axhline(0.0, color=C["neutral"], ls=":", lw=0.9, label="0 dB = 1 m")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("三维定位误差 (dB re 1 m)")
    ax.set_title("(c) 误差时序（真值时段内）", fontsize=10.5)
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
    ax.hist(lev[okv], bins=40, color=C["accent1"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    q50, q95 = np.percentile(all_e, [50, 95])
    ax.axvline(float(db20(max(q50, 1e-3))), color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {q50:.0f} m")
    ax.axvline(float(db20(max(q95, 1e-3))), color=C["accent4"], ls=":", lw=1.1,
               label=f"95% {q95:.0f} m")
    ax.set_xlabel("三维定位误差 (dB re 1 m)")
    ax.set_ylabel("航迹帧数")
    ax.set_title("(d) 误差分布", fontsize=10.5)
    ax.annotate(f"零误差 {int((all_e <= 0).sum())} 帧未入直方图\n"
                f"样本 {int(okv.sum())}/{all_e.size} 帧",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
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
                f"最大占用 {float(np.max(used)):.0f} 单元")

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
                f"宽角切面按 1° 分箱取峰值包络")

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

    rows = []
    for _, r in h.iterrows():
        for pos, amp in _parse(r["centers"]):
            rows.append((float(r["t"]), pos, amp, int(r.get("npeaks", 0) or 0)))
    if not rows:
        return None
    tab = pd.DataFrame(rows, columns=["t", "pos", "amp", "npeaks"])

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    n_single = int((h["npeaks"].to_numpy(float) == 1).sum())
    n_double = int((h["npeaks"].to_numpy(float) == 2).sum())
    title_block(fig, "图 35  HRRP 距离像：散射中心位置、幅度与间距",
                f"距离像 {len(h)} 帧，散射中心 {len(tab)} 个；"
                f"单峰 {n_single} 帧 / 双峰 {n_double} 帧")

    ax = axes[0, 0]
    sc = ax.scatter(tab["t"], tab["pos"], c=tab["amp"], s=18, cmap="plasma",
                    alpha=0.85, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("幅度 (dBsm)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("散射中心位置 (m)")
    ax.set_title("(a) 距离像随时间的散射中心", fontsize=10.5)
    style_axes(ax)

    ax = axes[0, 1]
    pos = tab["pos"].to_numpy(float)
    amp = tab["amp"].to_numpy(float)
    ups = np.unique(np.round(pos, 3))
    cx = [float(p) for p in ups]
    cy = [float(np.median(amp[np.isclose(pos, p, atol=1e-3)])) for p in ups]
    if cx:
        ax.stem(cx, cy, basefmt=" ", linefmt=C["accent1"], markerfmt="o")
        ax.annotate(f"散射中心 {len(cx)} 个\n"
                    f"峰值幅度 {max(cy):.1f} dBsm\n"
                    f"位置 {min(cx):.2f}–{max(cx):.2f} m",
                    xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.4,
                    va="top", bbox=dict(boxstyle="round,pad=0.35",
                                        fc="#f7f9fb", ec="#c8d0d8"))
        ax.set_xlim(min(cx) - 0.3, max(cx) + 0.3)
    ax.set_xlabel("距离 (m)")
    ax.set_ylabel("中位幅度 (dBsm)")
    ax.set_title("(b) 平均距离像（按散射中心位置聚合）", fontsize=10.5)
    style_axes(ax)

    ax = axes[1, 0]
    for k, p in enumerate(sorted(tab["pos"].unique())):
        s = tab[tab["pos"] == p].sort_values("t")
        ax.plot(s["t"], s["amp"], lw=0.9, color=CYCLE[k % len(CYCLE)],
                label=f"{p:.2f} m")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("幅度 (dBsm)")
    ax.set_title("(c) 各散射中心幅度时序", fontsize=10.5)
    ax.legend(loc="best", fontsize=8, title="位置")
    style_axes(ax)

    ax = axes[1, 1]
    lens = h["length"].to_numpy(float)
    npk = h["npeaks"].to_numpy(float)
    ax.hist(lens, bins=np.arange(np.nanmin(lens) - 0.25,
                                 np.nanmax(lens) + 0.35, 0.25),
            color=C["accent3"], alpha=0.8, edgecolor="white", linewidth=0.4)
    ax.set_xlabel("距离像长度 (m)")
    ax.set_ylabel("帧数")
    ax.set_title("(d) 距离像长度分布", fontsize=10.5)
    ax.annotate(f"单峰 {int((npk == 1).sum())} 帧 / 双峰 {int((npk == 2).sum())} 帧\n"
                f"长度 {_rng_txt(lens, ' m')}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.4,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
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
