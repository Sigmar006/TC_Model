# -*- coding: utf-8 -*-
"""图 17–25：MTD 通道、回波距离律、特征新鲜度、RCS、识别、HRRP 与事件链。"""
from __future__ import annotations

import re

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, LogNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import NullFormatter

from .common import C, CYCLE, W_RADAR, style_axes, title_block


# ---------------------------------------------------------------- 小工具
def _ch(df, col="ch"):
    """把 'a,b,c,…' 通道串解析成 (n, 8) 数组。"""
    return np.vstack([np.fromstring(s, sep=",") for s in df[col].astype(str)])


def _rollmed(y, w=51):
    """居中滑动中位数（用于压住逐帧抖动）。"""
    y = np.asarray(y, dtype=float)
    half = w // 2
    out = np.full_like(y, np.nan)
    for i in range(half, len(y) - half):
        out[i] = np.median(y[i - half:i + half + 1])
    return out


def _cbar(fig, mappable, ax, label):
    cb = fig.colorbar(mappable, ax=ax, pad=0.02)
    cb.set_label(label, fontsize=8.5)
    cb.ax.tick_params(labelsize=7.5)
    return cb


# ---------------------------------------------------------------- 图 17
def fig17(D):
    jg = D["radar_output"]["jam_gain"].sort_values("t")
    nz = D["radar_output"]["noise"]
    J = _ch(jg)
    noise0 = float(np.median(_ch(nz)))
    t = jg["t"].to_numpy(float)
    db = 10.0 * np.log10(J)
    suppr = jg["suppr"].to_numpy(float)
    p_mti = jg["p_mti"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.2))
    title_block(fig, "图 17  MTD 八通道自适应处理",
                f"{len(jg)} 个处理周期，各通道热噪声恒为 {noise0:.3e} W；"
                f"干扰通道功率 {J.min():.2e}–{J.max():.2e} W，"
                f"自适应抑制比 {suppr.min():.2f}–{suppr.max():.2f} dB（中位 {np.median(suppr):.2f} dB）")

    ax = axes[0, 0]
    im = ax.imshow(db.T, aspect="auto", origin="lower", cmap="magma",
                   extent=[t.min(), t.max(), 0.5, 8.5], interpolation="nearest")
    ax.set_yticks(range(1, 9))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("MTD 通道")
    ax.set_title("(a) 干扰通道功率–时间热图", fontsize=10.5)
    _cbar(fig, im, ax, "通道功率 (dBW)")
    ax.grid(False)
    style_axes(ax, minor=False)

    ax = axes[0, 1]
    med = np.median(db, axis=0)
    rel = med - 10 * np.log10(noise0)
    bars = ax.bar(np.arange(1, 9), rel, width=0.62,
                  color=[CYCLE[k] for k in range(8)], alpha=0.9)
    for b, m in zip(bars, rel):
        ax.text(b.get_x() + b.get_width() / 2, m + 0.45,
                f"{10**(m/10):.1f}×", ha="center", fontsize=7.6,
                fontweight="bold")
    ax.set_xticks(range(1, 9))
    ax.set_xlabel("MTD 通道")
    ax.set_ylabel("相对热噪声 (dB)")
    ax.set_title("(b) 通道残余干扰与噪声比", fontsize=10.5)
    ax.axhline(0, color=C["warn"], ls="--", lw=1.1, label="热噪声基线")
    ax.set_ylim(-3, max(rel) * 1.35 + 1.0)
    ax.legend(loc="lower right", fontsize=8.5)
    ax.grid(axis="x", visible=False)
    style_axes(ax)

    ax = axes[1, 0]
    ax.hist(suppr, bins=32, color=C["accent1"], alpha=0.82, edgecolor="white",
            linewidth=0.4)
    ax.axvline(np.median(suppr), color=C["warn"], ls="--", lw=1.2,
               label=f"中位 {np.median(suppr):.2f} dB")
    ax.axvline(0, color=C["neutral"], lw=1.0)
    ax.set_xlabel("自适应抑制比 (dB)")
    ax.set_ylabel("处理周期数")
    ax.set_title("(c) 抑制比分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.annotate(f"最小 {suppr.min():.2f} dB（{int((suppr < 3).sum())} 个周期 < 3 dB）\n"
                f"90% 分位 {np.percentile(suppr, 90):.1f} dB",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    x = 10.0 * np.log10(p_mti)
    sc = ax.scatter(x, suppr, c=t, s=16, cmap="viridis", alpha=0.85,
                    linewidths=0)
    rho = float(np.corrcoef(x, suppr)[0, 1])
    ax.set_xlabel("MTI 前干扰总功率 (dBW)")
    ax.set_ylabel("自适应抑制比 (dB)")
    ax.set_title("(d) 抑制能力随干扰强度变化", fontsize=10.5)
    _cbar(fig, sc, ax, "仿真时间 (s)")
    if rho > 0.15:
        trend = "抑制比随干扰功率上升"
    elif rho < -0.15:
        trend = "抑制比随干扰功率下降"
    else:
        trend = "与干扰功率无显著线性关系"
    ax.annotate(f"Pearson r = {rho:.2f}；{len(jg)} 个周期\n{trend}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 18
def fig18(D):
    e = D["radar_output"]["echo"].sort_values("t")
    rng = e["rng"].to_numpy(float) / 1e6            # 10^3 km
    p = e["p_echo"].to_numpy(float)
    t = e["t"].to_numpy(float)

    # 远近段分界由两段各自做 log–log 拟合的残差平方和最小确定，
    # 避免把某次想定的分界时刻写死；换数据后自动跟着走。
    lr, lp = np.log10(rng), np.log10(p)

    def _ss(m):
        if m.sum() < 12:
            return np.inf
        c = np.polyfit(lr[m], lp[m], 1)
        return float(np.sum((lp[m] - np.polyval(c, lr[m])) ** 2))

    split = float(t.min())
    best = np.inf
    for c in np.quantile(t, np.linspace(0.15, 0.90, 151)):
        m1 = t <= c
        if m1.sum() < 20 or (~m1).sum() < 20:
            continue
        v = _ss(m1) + _ss(~m1)
        if v < best:
            best, split = v, float(c)
    early = t <= split

    lg = np.polyfit(np.log10(rng[early]), np.log10(p[early]), 1)
    k = p * (rng * 1e6) ** 4
    k0 = np.median(k[early])

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.2))
    title_block(fig, "图 18  回波功率的距离律检验",
                f"回波 {p.min():.2e}–{p.max():.2e} W，斜距 {rng.min()*1e3:.0f}–"
                f"{rng.max()*1e3:.0f}×10$^3$ km；远段（t ≤ {split:.0f} s）拟合 $P_r$ ∝ "
                f"$R^{{{lg[0]:.2f}}}$，近段出现系统性偏离")

    ax = axes[0, 0]
    ax.scatter(rng[early], p[early], s=16, color=C["accent1"], alpha=0.8,
               linewidths=0, label=f"远段（{int(early.sum())}）")
    ax.scatter(rng[~early], p[~early], s=16, facecolors="none",
               edgecolors=C["warn"], linewidths=0.8,
               label=f"近段 t > {split:.0f} s（{int((~early).sum())}）")
    xs = np.linspace(np.log10(rng.min()), np.log10(rng.max()), 60)
    ax.plot(10 ** xs, 10 ** np.polyval(lg, xs), color="#123f63", lw=1.6,
            label=f"远段拟合 {lg[0]:.2f}")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xticks([2.4, 2.8, 3.2, 3.6, 4.0, 4.4, 4.8])
    ax.set_xticklabels(["2.4", "2.8", "3.2", "3.6", "4.0", "4.4", "4.8"])
    ax.xaxis.set_minor_formatter(NullFormatter())
    ax.set_xlabel("斜距 ($10^3$ km)")
    ax.set_ylabel("回波功率 (W)")
    ax.set_title("(a) 回波功率–距离双对数", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2)
    ax.annotate(f"理论 $R^{{-4}}$；实测 {lg[0]:.2f}",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3,
                va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[0, 1]
    ax.scatter(t[early], (k / k0)[early], s=6, color=C["accent4"], alpha=0.55,
               linewidths=0, label="远段")
    ax.scatter(t[~early], (k / k0)[~early], s=14, color=C["warn"], zorder=3,
               linewidths=0, label="近段")
    ax.axhline(1.0, color=C["neutral"], ls="--", lw=1.0)
    ax.axvspan(split, t.max(), color=C["warn"], alpha=0.07, lw=0)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("$P_r R^4$ / 中位")
    ax.set_title("(b) $P_rR^4$ 恒定性", fontsize=10.5)
    ax.set_ylim(0, 1.3)
    ax.legend(loc="lower left", fontsize=8.5, markerscale=2.5)
    ax.annotate(f"远段起伏 {100*(k[early].std()/k[early].mean()):.1f}%"
                f"（{int(early.sum())} 点）\n"
                f"近段最低降至 {100*(k/k0).min():.0f}%",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    ax.plot(t, e["p_echo"], lw=0.8, color=C["accent2"], label="回波 $P_r$")
    ax.plot(t, e["p_noise"], lw=0.8, color=C["neutral"], label="热噪声 $P_n$")
    ax.plot(t, e["p_jam"], lw=0.8, color=C["warn"], label="干扰 $P_j$")
    ax.set_yscale("log")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率 (W)")
    ax.set_title("(c) 回波、噪声与干扰功率时序", fontsize=10.5)
    ax.legend(loc="center right", fontsize=8.2, ncol=1)
    ax.annotate(f"$P_j/P_r$ 中位 {np.median(e['p_jam']/e['p_echo']):.0f}×"
                f"（{10*np.log10(np.median(e['p_jam']/e['p_echo'])):.1f} dB）",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    resid = np.log10(p[early]) - np.polyval(lg, np.log10(rng[early]))
    ax.hist(resid, bins=26, color=C["accent3"], alpha=0.82, edgecolor="white",
            linewidth=0.4)
    ax.axvline(0, color=C["warn"], ls="--", lw=1.1)
    ax.set_xlim(-0.2, 0.12)
    ax.set_xlabel("$\\log_{10}P_r$ 拟合残差 (dex)")
    ax.set_ylabel("样本数")
    ax.set_title("(d) 远段拟合残差", fontsize=10.5)
    outlier = int((resid < -0.2).sum())
    ax.annotate(f"残差 σ = {resid.std():.3f} dex"
                f"（{100*(10**resid.std()-1):.0f}%）\n"
                f"决定系数 $R^2$ = {1-resid.var()/np.log10(p[early]).var():.3f}\n"
                f"{outlier} 个离群点 < −0.2 dex 未显示",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 19
def fig19(D):
    tf = D["radar_int"]["trk_feat"].sort_values("t")
    tracks = sorted(tf["track"].unique())
    lag = (tf["t"] - tf["rec_cyc"]).to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 19  特征掩码与记录新鲜度",
                f"特征记录 {len(tf)} 条；掩码 15 共 {int(tf['mask'].eq(15).sum())} 条、"
                f"掩码 10 共 {int(tf['mask'].eq(10).sum())} 条；"
                f"特征滞后 $t-t_{{feat}}≡{int((tf['t']-tf['feat_cyc']).median())}$ s，"
                f"记录滞后最大 {lag.max():.0f} s")

    ax = axes[0, 0]
    t0, t1 = int(tf["t"].min()), int(tf["t"].max())
    M = np.full((len(tracks), t1 - t0 + 1), np.nan)
    for i, tr in enumerate(tracks):
        g = tf[tf["track"] == tr]
        M[i, (g["t"].astype(int) - t0).to_numpy()] = g["mask"].to_numpy(float)
    code = np.where(M == 15, 1.0, np.where(M == 10, 0.0, np.nan))
    cmap = ListedColormap([C["accent2"], C["accent3"]])
    cmap.set_bad("white")
    ax.imshow(code, aspect="auto", interpolation="nearest", cmap=cmap,
              vmin=-0.5, vmax=1.5,
              extent=[t0, t1 + 1, len(tracks) - 0.5, -0.5])
    ax.set_xlim(t0, t1 + 1)
    ax.set_yticks(range(len(tracks)))
    ax.set_yticklabels([f"航迹 {int(t)}" for t in tracks])
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("航迹")
    ax.set_title("(a) 特征掩码时间线", fontsize=10.5)
    ax.legend(handles=[Patch(facecolor=C["accent3"], label="掩码 15（全维）"),
                       Patch(facecolor=C["accent2"], label="掩码 10（降维）")],
              loc="lower left", fontsize=8.2)
    ax.grid(False)
    style_axes(ax, minor=False)

    ax = axes[0, 1]
    for k, tr in enumerate(tracks):
        g = tf[tf["track"] == tr]
        ax.scatter(g["t"], g["t"] - g["rec_cyc"], s=5, color=CYCLE[k], alpha=0.45,
                   linewidths=0, label=f"航迹 {int(tr)}")
    ax.axhline(-1, color=C["warn"], ls="--", lw=1.0, label="滞后 −1 s（同周期）")
    ax.set_xlim(t0 - 20, t1 + 20)
    ax.set_ylim(-12, max(50.0, float(lag.max()) * 1.15))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("$t - t_{rec}$ (s)")
    ax.set_title("(b) 记录滞后随时间", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.4, ncol=2, markerscale=2.5)
    style_axes(ax)

    ax = axes[1, 0]
    m15 = [int((tf[tf["track"] == tr]["mask"] == 15).sum()) for tr in tracks]
    m10 = [int((tf[tf["track"] == tr]["mask"] == 10).sum()) for tr in tracks]
    x = np.arange(len(tracks))
    ax.bar(x, m15, width=0.55, color=C["accent3"], label="掩码 15")
    ax.bar(x, m10, bottom=m15, width=0.55, color=C["accent2"], label="掩码 10")
    span = max(max(a + b for a, b in zip(m15, m10)), 1)
    for xi, a, b in zip(x, m15, m10):
        ax.text(xi, a + b + 0.02 * span, f"{a + b}", ha="center", fontsize=8.5,
                fontweight="bold")
        if a > 0.1 * span:
            ax.text(xi, a / 2, f"{a}", ha="center", fontsize=8, color="white")
        if b > 0.1 * span:
            ax.text(xi, a + b / 2, f"{b}", ha="center", fontsize=8, color="white")
    ax.set_xticks(x)
    ax.set_xticklabels([f"航迹 {int(t)}" for t in tracks])
    ax.set_ylabel("特征记录数")
    ax.set_title("(c) 各航迹掩码构成", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(axis="x", visible=False)
    per = [f"航迹 {int(tr)} {100.0 * b / max(a + b, 1):.0f}%"
           for tr, a, b in zip(tracks, m15, m10)]
    rows = [" · ".join(per[i:i + 2]) for i in range(0, len(per), 2)]
    ax.annotate("掩码 10 占比：\n" + "\n".join(rows),
                xy=(0.97, 0.55), xycoords="axes fraction", fontsize=8.2, va="top",
                ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    data = [(tf[tf["track"] == tr]["t"] - tf[tf["track"] == tr]["rec_cyc"]).to_numpy(float)
            for tr in tracks]
    bp = ax.boxplot(data, widths=0.55, patch_artist=True, showfliers=False,
                    medianprops=dict(color="#1b2733", lw=1.2))
    for k, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(CYCLE[k])
        patch.set_alpha(0.65)
        patch.set_edgecolor("#4a4f55")
    for k, tr in enumerate(tracks):
        med = float(np.median(tf[tf["track"] == tr]["t"] - tf[tf["track"] == tr]["rec_cyc"]))
        ax.text(k + 1, med + 0.02 * max(float(lag.max()), 50.0), f"{med:.0f}",
                ha="center", fontsize=8, fontweight="bold")
    ax.axhline(-1, color=C["warn"], ls="--", lw=1.0)
    ax.set_xticks(range(1, len(tracks) + 1))
    ax.set_xticklabels([f"航迹 {int(t)}" for t in tracks])
    ax.set_ylabel("$t - t_{rec}$ (s)")
    ax.set_title("(d) 记录滞后分布（箱体 25–75%）", fontsize=10.5)
    ax.set_ylim(-30, max(50.0, float(lag.max()) * 1.15))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 20
def fig20(D):
    rc = D["radar_output"]["rcs_feat"].sort_values("t")
    tracks = sorted(rc["track"].unique())
    lv = rc["mean"].round(3)                    # 均值档位由数据决定，不写死
    levels = sorted(float(v) for v in lv.unique())
    hi, lo = levels[-1], levels[0]
    ntier = len(levels)
    n_hi, n_lo = int(lv.eq(hi).sum()), int(lv.eq(lo).sum())
    t_hi = rc.loc[lv.eq(hi), "t"]
    t_lo = rc.loc[lv.eq(lo), "t"]
    t0, t1 = float(rc["t"].min()), float(rc["t"].max())
    pad = 0.02 * max(t1 - t0, 1.0)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    if ntier == 1:
        head = f"RCS 均值恒为 {hi:.1f} dBsm（{n_hi} 条，{t0:.0f}–{t1:.0f} s）；"
    elif ntier == 2:
        head = (f"RCS 均值 {hi:.1f} dBsm（{n_hi} 条，{t_hi.min():.0f}–"
                f"{t_hi.max():.0f} s）与 {lo:.1f} dBsm（{n_lo} 条，"
                f"{t_lo.min():.0f}–{t_lo.max():.0f} s）两档；")
    else:
        head = (f"RCS 均值共 {ntier} 档（{lo:.1f}–{hi:.1f} dBsm），"
                f"高档 {n_hi} 条、低档 {n_lo} 条；")
    title_block(fig, "图 20  RCS 特征均值、起伏与收敛",
                head + f"起伏 σ {rc['std'].min():.2f}–{rc['std'].max():.2f} dB")

    ax = axes[0, 0]
    for k, tr in enumerate(tracks):
        g = rc[rc["track"] == tr]
        ax.scatter(g["t"], g["mean"], s=5, color=CYCLE[k], alpha=0.45,
                   linewidths=0, label=f"航迹 {int(tr)}")
    shade = ["#f7e4d6", "#dbe4ec", "#dcecdc", "#efe2f2"]
    for i, lvv in enumerate(levels):
        s0 = float(rc.loc[lv.eq(lvv), "std"].median())
        ax.axhspan(lvv - s0, lvv + s0, color=shade[i % len(shade)],
                   alpha=0.75, lw=0, zorder=0)
    if ntier >= 2:
        ax.axvline(t_hi.max(), color=C["warn"], ls="--", lw=1.0)
        ax.text(t_hi.max() + 0.01 * (t1 - t0), hi + 0.8,
                f"高档结束 {t_hi.max():.0f} s", color=C["warn"], fontsize=7.8,
                va="center")
    ax.set_xlim(t0 - pad, t1 + pad)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("RCS 均值 (dBsm)")
    ax.set_title(f"(a) 均值随时间（{'两' if ntier == 2 else ntier} 档）",
                 fontsize=10.5)
    ax.legend(loc="center right", fontsize=7.8, ncol=2, markerscale=2.5)
    ax.annotate("阴影为各档 ±1σ 中位带宽",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.2, va="bottom",
                bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[0, 1]
    for k, tr in enumerate(tracks):
        g = rc[rc["track"] == tr]
        ax.scatter(g["t"], g["std"], s=6, color=CYCLE[k], alpha=0.5,
                   linewidths=0, label=f"航迹 {int(tr)}")
    ax.axhline(rc["std"].median(), color=C["neutral"], ls="--", lw=1.0,
               label=f"中位 {rc['std'].median():.2f} dB")
    ax.set_xlim(t0 - pad, t1 + pad)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("RCS 起伏 σ (dB)")
    ax.set_title("(b) 起伏强度随时间", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=7.6, ncol=2, markerscale=2.5)
    style_axes(ax)

    ax = axes[1, 0]
    for k, lvv in enumerate(levels):
        g = rc.loc[lv.eq(lvv), "std"].sort_values().to_numpy(float)
        ax.step(g, np.arange(1, len(g) + 1) / len(g), where="post", lw=1.5,
                color=CYCLE[k], label=f"均值 {lvv:.1f} dBsm（{len(g)} 条）")
        ax.axvline(np.median(g), color=CYCLE[k], ls=":", lw=1.0)
        ax.text(np.median(g) + 0.015, 0.55 - 0.08 * k, f"{np.median(g):.2f}",
                color=CYCLE[k], fontsize=8, ha="left")
    ax.axhline(0.5, color=C["neutral"], ls="--", lw=0.9)
    ax.set_xlabel("起伏 σ (dB)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title(f"(c) {'两' if ntier == 2 else ntier} 档状态的 σ 经验分布",
                 fontsize=10.5)
    ax.legend(loc="upper left", fontsize=8.0)
    style_axes(ax)

    ax = axes[1, 1]
    early = rc[lv.eq(hi)]
    post = rc.loc[lv.eq(lo)]
    names = ["归一化 RCS", "起伏 σ", "起伏度"]
    chg = [100 * (post[c].median() / early[c].median() - 1)
           for c in ("norm", "std", "fluct")]
    y = np.arange(len(names))
    bars = ax.barh(y, chg, height=0.5, color=CYCLE[:3], alpha=0.88)
    span = max(max(abs(v) for v in chg), 1.0)
    off = 0.04 * span
    for b, v in zip(bars, chg):
        inside = abs(v) > 0.15 * span
        if inside:
            tx = v + off if v < 0 else v - off
            ha, color = ("left" if v < 0 else "right"), "white"
        else:
            tx = v - off if v < 0 else v + off
            ha, color = ("right" if v < 0 else "left"), "#1b2733"
        ax.text(tx, b.get_y() + b.get_height() / 2, f"{v:.0f}%",
                ha=ha, va="center", fontsize=9, fontweight="bold", color=color)
    ax.axvline(0, color="#1b2733", lw=1.0)
    ax.set_yticks(y)
    ax.set_yticklabels(names)
    ax.set_xlim(min(chg) - 0.12 * span, max(chg) + 0.12 * span)
    ax.set_xlabel("低档相对高档的变化 (%)")
    ax.set_title(f"(d) {'两' if ntier == 2 else ntier} 档特征变化", fontsize=10.5)
    if ntier >= 2:
        note = (f"RCS 均值 {hi:.1f} → {lo:.1f} dBsm（{lo - hi:+.1f} dB）\n"
                f"高档 {len(early)} 条、低档 {len(post)} 条")
    else:
        note = f"RCS 均值恒为 {hi:.1f} dBsm（无档位切换）"
    ax.annotate(note, xy=(0.03, 0.97), xycoords="axes fraction", fontsize=8.2,
                va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.grid(axis="y", visible=False)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 21
def fig21(D):
    tf = D["radar_int"]["trk_feat"].sort_values("t")
    m = tf["margin"].to_numpy(float)
    s1 = tf["s1"].to_numpy(float)
    t = tf["t"].to_numpy(float)
    cls = tf["cls"].astype(str)
    t0, t1 = float(t.min()), float(t.max())
    pad = 0.02 * max(t1 - t0, 1.0)
    s2 = tf["s2"].to_numpy(float)
    ylo = max(0.0, float(min(s1.min(), s2.min())) - 0.05)
    yhi = min(1.0, float(max(s1.max(), s2.max())) + 0.05)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 21  目标识别置信度与判决结果",
                f"{len(tf)} 条识别记录；置信度 $s_1$ {s1.min():.3f}–{s1.max():.3f}、"
                f"$s_2$ {tf['s2'].min():.3f}–{tf['s2'].max():.3f}；"
                f"判别余量 {m.min():.4f}–{m.max():.4f}；"
                f"弹道目标 {int(cls.eq('弹道目标').sum())} 条、未知 {int(cls.eq('未知').sum())} 条")

    ax = axes[0, 0]
    ax.scatter(t, s1, s=5, color=C["accent1"], alpha=0.45, linewidths=0,
               label="$s_1$（弹道目标）")
    ax.scatter(t, tf["s2"], s=5, color=C["accent2"], alpha=0.45, linewidths=0,
               label="$s_2$（其他类）")
    ax.set_xlim(t0 - pad, t1 + pad)
    ax.set_ylim(ylo, yhi)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("置信度")
    ax.set_title("(a) 两类置信度时序", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8.2, markerscale=3.0)
    style_axes(ax)

    ax = axes[0, 1]
    ax.hist(m, bins=34, color=C["accent3"], alpha=0.82, edgecolor="white",
            linewidth=0.4)
    ax.axvline(np.median(m), color=C["warn"], ls="--", lw=1.2,
               label=f"中位 {np.median(m):.4f}")
    ax.set_xlabel("判别余量 $s_1-s_2$")
    ax.set_ylabel("样本数")
    ax.set_title("(b) 判别余量分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.annotate(f"最小 {m.min():.4f}；5% 分位 {np.percentile(m, 5):.4f}\n"
                f"余量 < 0 的样本 {int((m < 0).sum())} 条",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    tracks = sorted(tf["track"].unique())
    for k, tr in enumerate(tracks):
        g = tf[tf["track"] == tr]
        unknown = g["cls"].astype(str).eq("未知")
        ax.scatter(g["t"][~unknown], np.full((~unknown).sum(), k),
                   s=5, color=C["accent3"], alpha=0.5, linewidths=0)
        ax.scatter(g["t"][unknown], np.full(unknown.sum(), k), s=30,
                   marker="v", color=C["warn"], linewidths=0,
                   label="判为未知" if k == 0 else None)
    ax.set_xlim(t0 - pad, t1 + pad)
    ax.set_yticks(range(len(tracks)))
    ax.set_yticklabels([f"航迹 {int(t)}" for t in tracks])
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("航迹")
    ax.set_title("(c) 识别结果时间线", fontsize=10.5)
    ax.legend(handles=[Line2D([], [], marker="o", ls="", ms=4.5,
                              color=C["accent3"], label="弹道目标"),
                       Line2D([], [], marker="v", ls="", ms=6,
                              color=C["warn"], label="判为未知")],
              loc="upper right", fontsize=8.5)
    ax.grid(axis="y", visible=False)
    style_axes(ax)

    ax = axes[1, 1]
    for k, cl in enumerate(sorted(cls.unique())):
        g = cls == cl
        ax.scatter(s1[g.to_numpy()], m[g.to_numpy()], s=8, color=CYCLE[k],
                   alpha=0.5, linewidths=0, label=f"{cl}（{int(g.sum())}）")
    ax.set_xlabel("$s_1$")
    ax.set_ylabel("判别余量 $s_1-s_2$")
    ax.set_title("(d) 余量与主类置信度", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=8.2, markerscale=2.0)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 22
def fig22(D):
    ro = D["radar_output"]
    r = ro["rcs_feat"][["track", "t", "contrib"]].rename(columns={"contrib": "RCS"})
    p = ro["pol_feat"][["track", "t", "contrib"]].rename(columns={"contrib": "极化"})
    h = ro["hrrp"][["track", "t", "contrib"]].rename(columns={"contrib": "HRRP"})
    j = r.merge(p, on=["track", "t"]).merge(h, on=["track", "t"])
    g = j.groupby("t")[["RCS", "极化", "HRRP"]].mean().sort_index()
    cols = ["RCS", "极化", "HRRP"]
    colors = [C["accent1"], C["accent2"], C["accent3"]]
    gt0, gt1 = float(g.index.min()), float(g.index.max())
    gpad = 0.02 * max(gt1 - gt0, 1.0)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 22  识别特征贡献构成",
                f"{len(j)} 组同时刻特征（{len(g)} 个时刻）；RCS {j['RCS'].min():.3f}–"
                f"{j['RCS'].max():.3f}、极化 {j['极化'].min():.3f}–{j['极化'].max():.3f}、"
                f"HRRP {j['HRRP'].min():.3f}–{j['HRRP'].max():.3f}；"
                f"三者之和 {j[cols].sum(axis=1).min():.2f}–{j[cols].sum(axis=1).max():.2f}")

    ax = axes[0, 0]
    ax.stackplot(g.index, [g[c] for c in cols], labels=cols, colors=colors,
                 alpha=0.85, linewidth=0)
    ax.set_xlim(gt0 - gpad, gt1 + gpad)
    ax.set_ylim(0, max(0.1, float(g[cols].sum(axis=1).max()) * 1.08))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("特征贡献")
    ax.set_title("(a) 贡献堆叠（按时刻均值）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2, ncol=3)
    style_axes(ax)

    ax = axes[0, 1]
    share = g[cols].div(g[cols].sum(axis=1), axis=0) * 100
    ax.stackplot(share.index, [share[c] for c in cols], labels=cols,
                 colors=colors, alpha=0.85, linewidth=0)
    ax.set_xlim(gt0 - gpad, gt1 + gpad)
    ax.set_ylim(0, 100)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("占比 (%)")
    ax.set_title("(b) 贡献占比构成", fontsize=10.5)
    ax.annotate(f"RCS {share['RCS'].mean():.0f}% · 极化 {share['极化'].mean():.0f}%"
                f" · HRRP {share['HRRP'].mean():.0f}%",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    tot = j[cols].sum(axis=1).to_numpy(float)
    ax.hist(tot, bins=44, color=C["accent4"], alpha=0.82, edgecolor="white",
            linewidth=0.4)
    ax.axvline(np.median(tot), color=C["warn"], ls="--", lw=1.2,
               label=f"中位 {np.median(tot):.3f}")
    t_lo, t_hi = float(tot.min()), float(tot.max())
    t_pad = 0.05 * max(t_hi - t_lo, 0.05)
    ax.set_xlim(t_lo - t_pad, t_hi + t_pad)
    ax.set_xlabel("三项贡献之和")
    ax.set_ylabel("样本数")
    ax.set_title("(c) 总贡献分布（未归一）", fontsize=10.5)
    ax.legend(loc="center left", fontsize=8.2)
    ax.annotate(f"取值 {tot.min():.2f}–{tot.max():.2f}（归一化参考 1.0）",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3,
                va="top", ha="left",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    bp = ax.boxplot([j[c].to_numpy(float) for c in cols], widths=0.5,
                    patch_artist=True, showfliers=False,
                    medianprops=dict(color="#1b2733", lw=1.2))
    for k, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(colors[k])
        patch.set_alpha(0.75)
        patch.set_edgecolor("#4a4f55")
    ax.set_xticks(range(1, len(cols) + 1))
    ax.set_xticklabels(cols)
    ax.set_ylabel("特征贡献")
    ax.set_title("(d) 各特征贡献分布", fontsize=10.5)
    yoff = 0.03 * max(float(np.ptp(j[c].to_numpy(float))) for c in cols)
    for k, c in enumerate(cols):
        med = float(np.median(j[c]))
        ax.text(k + 1, med + yoff, f"{med:.3f}", ha="center", fontsize=8.5,
                fontweight="bold")
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 23
def fig23(D):
    h = D["radar_output"]["hrrp"].sort_values("t")
    pat = re.compile(r"\(([-\d.]+)m,([-\d.]+)dBsm\)")
    rows = []
    for s in h["centers"]:
        mm = pat.findall(str(s))
        rows.append([float(mm[0][0]), float(mm[0][1]),
                     float(mm[1][0]) if len(mm) > 1 else np.nan,
                     float(mm[1][1]) if len(mm) > 1 else np.nan])
    arr = np.asarray(rows, dtype=float)
    a1, a2 = arr[:, 1], arr[:, 3]
    t = h["t"].to_numpy(float)
    track = h["track"].to_numpy(float)
    d = a2 - a1

    fig = plt.figure(figsize=(13.4, 8.2))
    title_block(fig, "图 23  HRRP 双峰距离像瀑布",
                f"{len(h)} 帧距离像；恒为 0.0 m 与 2.5 m 两个散射中心；"
                f"峰 1 {a1.min():.1f}–{a1.max():.1f} dBsm，峰 2 {a2.min():.1f}–{a2.max():.1f} dBsm，"
                f"峰间差 {np.nanmin(d):.1f}–{np.nanmax(d):.1f} dB")

    ax = fig.add_subplot(2, 2, 1, projection="3d")
    idx = np.linspace(0, len(h) - 1, 90).astype(int)
    cmap = plt.get_cmap("viridis")
    tn = (t - t.min()) / max(t.max() - t.min(), 1.0)
    for i in idx:
        ax.plot([0.0, 2.5], [t[i], t[i]], [a1[i], a2[i]],
                color=cmap(tn[i]), lw=1.2, alpha=0.85)
    ax.set_xlabel("距离 (m)", labelpad=2)
    ax.set_ylabel("时间 (s)", labelpad=2)
    ax.set_zlabel("幅度 (dBsm)", labelpad=4, fontsize=9)
    ax.zaxis.set_rotate_label(False)
    ax.zaxis.label.set_rotation(90)
    ax.set_title("(a) 距离像瀑布（抽稀 90 帧，颜色由早到晚）", fontsize=10.5, pad=2)
    ax.view_init(elev=24, azim=-58)
    ax.tick_params(labelsize=7.5)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(t, a1, lw=0.5, color=C["accent1"], alpha=0.45, label="峰 1（0.0 m）")
    ax.plot(t, a2, lw=0.5, color=C["accent2"], alpha=0.45, label="峰 2（2.5 m）")
    ax.plot(t, _rollmed(a1), lw=1.6, color="#123f63")
    ax.plot(t, _rollmed(a2), lw=1.6, color="#8c2d04")
    ax.set_xlim(*W_RADAR)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("幅度 (dBsm)")
    ax.set_title("(b) 两峰幅度时序（深色为滑动中位）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8.2)
    ax.annotate(f"峰 1 中位 {np.median(a1):.1f} dBsm\n峰 2 中位 {np.nanmedian(a2):.1f} dBsm",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = fig.add_subplot(2, 2, 3)
    for k, tr in enumerate(sorted(np.unique(track))):
        g = track == tr
        ax.scatter(a1[g], a2[g], s=7, color=CYCLE[k], alpha=0.5, linewidths=0,
                   label=f"航迹 {int(tr)}")
    lim = [np.nanmin(a1) - 1, np.nanmax(a1) + 1]
    ax.plot(lim, lim, color=C["neutral"], ls="--", lw=1.0, label="等幅线")
    ax.set_xlim(*lim)
    ax.set_ylim(np.nanmin(a2) - 1, np.nanmax(a2) + 1)
    ax.set_xlabel("峰 1 幅度 (dBsm)")
    ax.set_ylabel("峰 2 幅度 (dBsm)")
    ax.set_title("(c) 双峰幅度相关性", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.8, markerscale=2.0)
    style_axes(ax)

    ax = fig.add_subplot(2, 2, 4)
    ax.hist(d, bins=34, color=C["accent4"], alpha=0.82, edgecolor="white",
            linewidth=0.4)
    ax.axvline(np.nanmedian(d), color=C["warn"], ls="--", lw=1.2,
               label=f"中位 {np.nanmedian(d):.1f} dB")
    ax.axvline(0, color=C["neutral"], lw=1.0)
    ax.set_xlabel("峰间差 $a_2-a_1$ (dB)")
    ax.set_ylabel("帧数")
    ax.set_title("(d) 峰间差分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.annotate(f"峰 2 弱于峰 1：{int((d < 0).sum())}/{len(d)} 帧\n"
                f"（{100*(d < 0).mean():.1f}%）；仅 {int((d > 0).sum())} 帧反转",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 24
def fig24(D):
    p = D["radar_output"]["pol_feat"].sort_values("t")
    ratio = (p["span_m"] / p["det_m"]).to_numpy(float)
    ratio_med = float(np.median(ratio))
    t = p["t"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 24  极化不变量与散射结构",
                f"{len(p)} 组极化特征；行列式 {p['det_m'].min():.2e}–{p['det_m'].max():.2e}，"
                f"Span {p['span_m'].min():.2e}–{p['span_m'].max():.2e}；"
                f"Span/det 为 {ratio.min():.3f}–{ratio.max():.3f}（中位 {np.median(ratio):.3f}）；"
                f"退极化比 {p['depol_m'].min():.4f}–{p['depol_m'].max():.4f}")

    ax = axes[0, 0]
    t0, t1 = float(t.min()), float(t.max())
    pad = 0.02 * max(t1 - t0, 1.0)
    ax.plot(t, p["det_m"], lw=0.55, color=C["accent1"], alpha=0.8, label="行列式 det")
    ax.plot(t, p["span_m"], lw=0.55, color=C["accent2"], alpha=0.8, label="Span")
    ax.set_yscale("log")
    ax.set_xlim(t0 - pad, t1 + pad)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率量 (W)")
    ax.set_title("(a) 行列式与 Span 时序", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8.2, ncol=1)
    style_axes(ax)

    ax = axes[0, 1]
    ax.hist(ratio, bins=32, color=C["accent3"], alpha=0.82, edgecolor="white",
            linewidth=0.4)
    ratio_lo = float(ratio.min())
    ratio_p90 = float(np.percentile(ratio, 90))
    ax.axvline(ratio_lo, color=C["warn"], ls="--", lw=1.2,
               label=f"实测下界 {ratio_lo:.3f}")
    ax.axvline(ratio.mean(), color="#123f63", ls=":", lw=1.2,
               label=f"均值 {ratio.mean():.3f}")
    ax.set_xlabel("Span / det")
    ax.set_ylabel("样本数")
    ax.set_title("(b) 不变量比值分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2)
    ax.annotate(f"中位 {ratio_med:.3f}，高出下界 "
                f"{100*(ratio_med/ratio_lo-1):.2f}%；\n"
                f"上十分位 {ratio_p90:.3f}–{ratio.max():.3f}",
                xy=(0.97, 0.55), xycoords="axes fraction", fontsize=8.3,
                va="top", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    sc = ax.scatter(p["ori_m"], p["depol_m"], c=p["ell_m"], s=10,
                    cmap="plasma", alpha=0.8, linewidths=0)
    ax.set_xlabel("取向角 (°)")
    ax.set_ylabel("退极化比")
    ax.set_title("(c) 取向角–退极化–椭圆率", fontsize=10.5)
    _cbar(fig, sc, ax, "椭圆率角 (°)")
    style_axes(ax)

    ax = axes[1, 1]
    tracks = sorted(p["track"].unique())
    data = [p[p["track"] == tr]["ell_m"].to_numpy(float) for tr in tracks]
    bp = ax.boxplot(data, widths=0.5, patch_artist=True, showfliers=False,
                    medianprops=dict(color="#1b2733", lw=1.2))
    for k, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(CYCLE[k])
        patch.set_alpha(0.7)
        patch.set_edgecolor("#4a4f55")
    ax.set_xticks(range(1, len(tracks) + 1))
    ax.set_xticklabels([f"航迹 {int(t)}" for t in tracks])
    ax.set_ylabel("椭圆率角 (°)")
    ax.set_title("(d) 各航迹椭圆率分布", fontsize=10.5)
    ax.annotate(f"整体 {p['ell_m'].min():.1f}…{p['ell_m'].max():.1f}°",
                xy=(0.97, 0.06), xycoords="axes fraction", fontsize=8.3,
                va="bottom", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 25
def fig25(D):
    ev = D["sat"]["event"].sort_values(["sat", "tid", "t_event"])
    lanes = sorted(ev.groupby(["sat", "tid"]).size().index.tolist())
    e0, e1 = float(ev["t_event"].min()), float(ev["t_event"].max())
    epad = 0.02 * max(e1 - e0, 1.0)
    colors = {"首次探测": C["accent3"], "跟踪中断": C["warn"],
              "跟踪中断(滑行)": C["accent1"]}
    markers = {"首次探测": "o", "跟踪中断": "x", "跟踪中断(滑行)": "."}

    fig, axes = plt.subplots(2, 2, figsize=(13.6, 8.2))
    title_block(fig, "图 25  卫星全链路事件时间轴",
                f"{len(ev)} 条事件：首次探测 {int(ev['event'].eq('首次探测').sum())}、"
                f"跟踪中断 {int(ev['event'].eq('跟踪中断').sum())}、"
                f"滑行中断 {int(ev['event'].eq('跟踪中断(滑行)').sum())}；"
                f"阶段构成 宽视场搜索 {int(ev['stage'].eq('宽视场搜索').sum())}、"
                f"窄视场捕获 {int(ev['stage'].eq('窄视场捕获').sum())}、"
                f"窄视场跟踪 {int(ev['stage'].eq('窄视场跟踪').sum())}")

    ax = axes[0, 0]
    for i, (sat, tid) in enumerate(lanes):
        g = ev[(ev["sat"] == sat) & (ev["tid"] == tid)]
        for name in colors:
            gg = g[g["event"] == name]
            if len(gg):
                ax.scatter(gg["t_event"], np.full(len(gg), i), s=14,
                           marker=markers[name], color=colors[name],
                           alpha=0.75, linewidths=0.9,
                           label=name if i == 0 else None)
    ax.set_xlim(max(0.0, e0 - epad), e1 + epad)
    ax.set_ylim(-0.6, len(lanes) + 0.9)
    ax.set_yticks(range(len(lanes)))
    ax.set_yticklabels([f"卫星 {int(s)} · 目标 {int(t)}" for s, t in lanes])
    ax.set_xlabel("事件时刻 (s)")
    ax.set_ylabel("卫星–目标链路")
    ax.set_title("(a) 事件时刻分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.8, markerscale=1.6, ncol=3)
    ax.grid(axis="y", visible=False)
    style_axes(ax)

    ax = axes[0, 1]
    events = ["首次探测", "跟踪中断", "跟踪中断(滑行)"]
    sats = sorted(ev["sat"].unique())
    piv = (ev.groupby(["event", "sat"]).size().unstack(fill_value=0)
           .reindex(index=events, columns=sats, fill_value=0))
    M = np.ma.masked_equal(piv.to_numpy(float), 0.0)
    cmap = plt.get_cmap("Blues").copy()
    cmap.set_bad("#f2f4f6")
    im = ax.imshow(M, aspect="auto", cmap=cmap,
                   norm=LogNorm(vmin=1, vmax=piv.to_numpy().max()),
                   extent=[-0.5, len(sats) - 0.5, len(events) - 0.5, -0.5])
    for r in range(len(events)):
        for c in range(len(sats)):
            v = int(piv.iloc[r, c])
            ax.text(c, r, f"{v}", ha="center", va="center", fontsize=9.5,
                    color="white" if v > 0.15 * piv.to_numpy().max() else "#1b2733",
                    fontweight="bold" if v else "normal")
    ax.set_xticks(range(len(sats)))
    ax.set_xticklabels([f"卫星 {int(s)}" for s in sats])
    ax.set_yticks(range(len(events)))
    ax.set_yticklabels(events)
    ax.set_title("(b) 事件类型 × 卫星计数", fontsize=10.5)
    _cbar(fig, im, ax, "事件数（对数色标）")
    ax.grid(False)
    style_axes(ax, minor=False)

    ax = axes[1, 0]
    for name, color in colors.items():
        g = ev[ev["event"] == name].sort_values("t_event")
        ax.step(g["t_event"], np.arange(1, len(g) + 1), where="post", lw=1.6,
                color=color, label=f"{name}（{len(g)}）")
    ax.set_xlim(max(0.0, e0 - epad), e1 + epad)
    ax.set_xlabel("事件时刻 (s)")
    ax.set_ylabel("累计事件数")
    ax.set_title("(c) 累计事件曲线", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=8.2)
    late = ev["t_event"] >= e0 + 0.75 * (e1 - e0)
    cnt = ev.loc[late, "event"].value_counts()
    if len(cnt):
        note = (f"后 25% 时段 {int(late.sum())} 条事件\n"
                f"{cnt.index[0]} {int(cnt.iloc[0])} 条"
                f"（{100 * cnt.iloc[0] / int(late.sum()):.0f}%）")
    else:
        note = "后 25% 时段无事件"
    ax.annotate(note, xy=(0.97, 0.30),
                xycoords="axes fraction", fontsize=8.3, ha="right", va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    dt = np.sort(np.concatenate([np.diff(np.sort(g.to_numpy(float)))
                                 for _, g in ev.groupby(["sat", "tid"])["t_event"]]))
    ax.step(dt, np.arange(1, len(dt) + 1) / len(dt), where="post", lw=1.6,
            color=C["accent1"])
    ax.set_xscale("log")
    dpos = dt[dt > 0]
    dt_lo = max(float(dpos.min()) * 0.8, 0.5) if len(dpos) else 0.5
    ax.set_xlim(dt_lo, max(float(dt.max()) * 1.25, 2.0))
    ax.set_ylim(0, 1.03)
    if len(dpos):
        vals, cnts = np.unique(dpos, return_counts=True)
        mode = float(vals[np.argmax(cnts)])
        share_mode = 100.0 * float(cnts.max()) / len(dt)
        ax.axvline(mode, color=C["warn"], ls="--", lw=1.1,
                   label=f"{mode:g} s：{share_mode:.1f}% 的间隔")
    ax.axhline(0.5, color=C["neutral"], ls=":", lw=0.9)
    ax.set_xlabel("相邻事件间隔 (s，对数轴)")
    ax.set_ylabel("累积概率")
    ax.set_title("(d) 相邻事件间隔累积分布", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8.5)
    ax.annotate(f"{len(dt)} 个间隔，来自 {ev.groupby(['sat', 'tid']).ngroups} 条链路；"
                f"最长 {dt.max():.0f} s",
                xy=(0.97, 0.45), xycoords="axes fraction", fontsize=8.3,
                va="center", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


FIGURES = [
    ("17", "fig17_mtd_channels", "图 17 MTD 八通道", fig17),
    ("18", "fig18_echo_range_law", "图 18 回波距离律", fig18),
    ("19", "fig19_feature_freshness", "图 19 特征新鲜度", fig19),
    ("20", "fig20_rcs_control", "图 20 RCS 特征", fig20),
    ("21", "fig21_class_confidence", "图 21 识别置信度", fig21),
    ("22", "fig22_feature_contrib", "图 22 特征贡献", fig22),
    ("23", "fig23_hrrp_waterfall", "图 23 HRRP 瀑布", fig23),
    ("24", "fig24_pol_invariants", "图 24 极化不变量", fig24),
    ("25", "fig25_event_timeline", "图 25 事件时间轴", fig25),
]
