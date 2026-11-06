# -*- coding: utf-8 -*-
"""图 17–25：MTD 通道、回波距离律、特征新鲜度、RCS、识别、HRRP 与事件链。

坐标口径
--------
* 功率、功率谱与计数一律换算成 dB 后线性绘制（10·lg），时间间隔同样取 dB(s)；
* 特征类图（19–24）一律按目标分开：一行 / 一组曲线只属于一个目标，
  标题与注释直接写出目标号，不再把多目标的特征、置信度、加权概率混在一起平均。
"""
from __future__ import annotations

import re

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D

from .common import (C, CYCLE, W_RADAR, by_target, db10, style_axes,
                     target_title, title_block)

COLS = ["RCS", "极化", "HRRP"]


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


def _ecdf(v):
    """返回 (排序后的值, 累积概率)，用于逐目标经验分布。"""
    v = np.sort(np.asarray(v, dtype=float))
    return v, np.arange(1, len(v) + 1) / max(len(v), 1)


def _tier_txt(g):
    """单个目标的 RCS 均值档位描述（档位数由数据决定，不写死）。"""
    lv = g["mean"].round(3)
    levels = sorted(float(v) for v in lv.unique())
    if not levels:
        return "无记录"
    if len(levels) == 1:
        return f"恒为 {levels[0]:.1f} dBsm"
    if len(levels) == 2:
        return f"{levels[0]:.1f} → {levels[1]:.1f} dBsm"
    return f"{len(levels)} 档（{levels[0]:.1f}–{levels[-1]:.1f} dBsm）"


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
                f"{len(jg)} 个处理周期，各通道热噪声恒为 {noise0:.3e} W"
                f"（{10*np.log10(noise0):.1f} dBW）；"
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
    x = db10(p_mti)
    sc = ax.scatter(x, suppr, c=t, s=16, cmap="viridis", alpha=0.85,
                    linewidths=0)
    ok = np.isfinite(x)
    rho = float(np.corrcoef(x[ok], suppr[ok])[0, 1]) if ok.sum() > 2 else np.nan
    ax.set_xlabel("MTI 后残余干扰功率 (dBW)")
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
    e_all = D["radar_output"]["echo"].sort_values("t")
    groups = by_target(e_all)

    # 每个目标单独定远近段分界、单独拟合：不同目标的 RCS 不同，混在一起拟合
    # 只会得到一条谁都不满足的平均线。
    prepared = []
    for k, (tid, g) in enumerate(groups):
        t = g["t"].to_numpy(float)
        rng = g["rng"].to_numpy(float) / 1e6            # 10^3 km
        p = g["p_echo"].to_numpy(float)
        if len(t) < 40:
            continue
        lr, lp = np.log10(rng), np.log10(p)

        def _ss(m):
            if m.sum() < 12:
                return np.inf
            c = np.polyfit(lr[m], lp[m], 1)
            return float(np.sum((lp[m] - np.polyval(c, lr[m])) ** 2))

        # 远近段分界由两段各自做 log–log 拟合的残差平方和最小确定，
        # 避免把某次想定的分界时刻写死；换数据后自动跟着走。
        split, best = float(t.min()), np.inf
        for c in np.quantile(t, np.linspace(0.15, 0.90, 151)):
            m1 = t <= c
            if m1.sum() < 20 or (~m1).sum() < 20:
                continue
            v = _ss(m1) + _ss(~m1)
            if v < best:
                best, split = v, float(c)
        early = t <= split
        lg = np.polyfit(lr[early], lp[early], 1)
        kg = p * (rng * 1e6) ** 4
        prepared.append(dict(tid=tid, color=CYCLE[k % len(CYCLE)], t=t, rng=rng,
                             p=p, p_db=db10(p),
                             p_n=db10(g["p_noise"].to_numpy(float)),
                             p_j=db10(g["p_jam"].to_numpy(float)),
                             split=split, early=early, lg=lg, k=kg,
                             k0=float(np.median(kg[early]))))
    if not prepared:
        return None

    t_lo = min(float(d["t"].min()) for d in prepared)
    t_hi = max(float(d["t"].max()) for d in prepared)
    r_lo = min(float(d["rng"].min()) for d in prepared)
    r_hi = max(float(d["rng"].max()) for d in prepared)
    p_lo = min(float(np.nanmin(d["p_db"])) for d in prepared)
    p_hi = max(float(np.nanmax(d["p_db"])) for d in prepared)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.2))
    title_block(fig, "图 18  回波功率的距离律检验（按目标）",
                f"回波 {p_lo:.1f}–{p_hi:.1f} dBW，斜距 {r_lo*1e3:.0f}–{r_hi*1e3:.0f}×10$^3$ km；"
                f"{len(prepared)} 个目标各自定远近段分界、单独拟合，互不混算")

    ax = axes[0, 0]
    for d in prepared:
        ax.scatter(d["rng"][d["early"]], d["p_db"][d["early"]], s=13,
                   color=d["color"], alpha=0.75, linewidths=0,
                   label=f"目标 {d['tid']}：$R^{{{d['lg'][0]:.2f}}}$")
        ax.scatter(d["rng"][~d["early"]], d["p_db"][~d["early"]], s=16,
                   facecolors="none", edgecolors=d["color"], linewidths=0.8)
        xs = np.linspace(float(d["rng"].min()), float(d["rng"].max()), 60)
        ax.plot(xs, 10 * np.polyval(d["lg"], np.log10(xs)), color=d["color"], lw=1.2)
    ax.set_xlabel("斜距 ($10^3$ km)")
    ax.set_ylabel("回波功率 (dBW)")
    ax.set_title("(a) 回波功率–距离（实心远段 / 空心近段）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.8)
    ax.annotate("理论 $R^{-4}$（−40 dB/十倍程）",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.0,
                va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[0, 1]
    for d in prepared:
        ax.scatter(d["t"][d["early"]], (d["k"] / d["k0"])[d["early"]], s=6,
                   color=d["color"], alpha=0.5, linewidths=0,
                   label=f"目标 {d['tid']}")
        ax.scatter(d["t"][~d["early"]], (d["k"] / d["k0"])[~d["early"]], s=16,
                   color=d["color"], alpha=0.9, marker="x", linewidths=0.9)
    ax.axhline(1.0, color=C["neutral"], ls="--", lw=1.0)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("$P_r R^4$ / 该目标中位")
    ax.set_title("(b) 逐目标 $P_rR^4$ 恒定性（× 为近段）", fontsize=10.5)
    ax.set_ylim(0, 1.35)
    ax.legend(loc="lower left", fontsize=7.4, markerscale=2.5)
    style_axes(ax)

    ax = axes[1, 0]
    for d in prepared:
        ax.plot(d["t"], d["p_db"], lw=0.8, color=d["color"])
        ax.plot(d["t"], d["p_n"], lw=0.6, color=d["color"], ls=":")
        ax.plot(d["t"], d["p_j"], lw=0.6, color=d["color"], ls="--")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率 (dBW)")
    ax.set_title("(c) 回波（实线）/ 噪声（点线）/ 干扰（虚线）", fontsize=10.5)
    ax.legend(handles=[Line2D([], [], color=d["color"], lw=1.4,
                              label=f"目标 {d['tid']}")
                       for d in prepared],
              loc="center right", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 1]
    resid_all, n_out = [], 0
    for d in prepared:
        r = 10.0 * (np.log10(d["p"][d["early"]])
                    - np.polyval(d["lg"], np.log10(d["rng"][d["early"]])))
        resid_all.append(r)
        n_out += int((r < -2.0).sum())
        ax.hist(r, bins=26, density=True, histtype="step", lw=1.4,
                color=d["color"], label=f"目标 {d['tid']}：σ {r.std():.2f} dB")
    ax.axvline(0, color=C["warn"], ls="--", lw=1.1)
    ax.set_xlim(-2.2, 1.2)
    ax.set_xlabel("回波功率拟合残差 (dB)")
    ax.set_ylabel("概率密度 (1/dB)")
    ax.set_title("(d) 各目标远段拟合残差", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.4)
    ax.annotate(f"{n_out} 个残差 < −2 dB 的点未显示",
                xy=(0.97, 0.06), xycoords="axes fraction", fontsize=8.0,
                va="bottom", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 19
def fig19(D):
    tf = D["radar_int"]["trk_feat"].sort_values("t")
    groups = by_target(tf)
    labels = [target_title(tid, g["track"]) for tid, g in groups]
    lag = (tf["t"] - tf["rec_cyc"]).to_numpy(float)
    t0, t1 = int(tf["t"].min()), int(tf["t"].max())

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 19  各目标的特征掩码与记录新鲜度",
                f"特征记录 {len(tf)} 条，{len(groups)} 个目标"
                f"（{'、'.join(labels)}）；掩码 15 共 {int(tf['mask'].eq(15).sum())} 条、"
                f"掩码 10 共 {int(tf['mask'].eq(10).sum())} 条；"
                f"特征滞后 $t-t_{{feat}}≡{int((tf['t']-tf['feat_cyc']).median())}$ s，"
                f"记录滞后最大 {lag.max():.0f} s")

    ax = axes[0, 0]
    M = np.full((len(groups), t1 - t0 + 1), np.nan)
    for i, (tid, g) in enumerate(groups):
        agg = g.assign(ti=g["t"].astype(int)).groupby("ti")["mask"].max()
        M[i, (agg.index.to_numpy() - t0)] = agg.to_numpy(float)
    code = np.where(M == 15, 1.0, np.where(M == 10, 0.0, np.nan))
    cmap = ListedColormap([C["accent2"], C["accent3"]])
    cmap.set_bad("white")
    ax.imshow(code, aspect="auto", interpolation="nearest", cmap=cmap,
              vmin=-0.5, vmax=1.5,
              extent=[t0, t1 + 1, len(groups) - 0.5, -0.5])
    ax.set_xlim(t0, t1 + 1)
    ax.set_yticks(range(len(groups)))
    ax.set_yticklabels(labels)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("目标")
    ax.set_title("(a) 特征掩码时间线（每行一个目标；绿=掩码 15，橙=掩码 10）",
                 fontsize=10.5)
    ax.grid(False)
    style_axes(ax, minor=False)

    ax = axes[0, 1]
    for k, (tid, g) in enumerate(groups):
        ax.scatter(g["t"], g["t"] - g["rec_cyc"], s=5, color=CYCLE[k], alpha=0.45,
                   linewidths=0, label=target_title(tid, g["track"]))
    ax.axhline(-1, color=C["warn"], ls="--", lw=1.0, label="滞后 −1 s（同周期）")
    ax.set_xlim(t0 - 20, t1 + 20)
    ax.set_ylim(-12, max(50.0, float(lag.max()) * 1.15))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("$t - t_{rec}$ (s)")
    ax.set_title("(b) 记录滞后随时间", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.0, ncol=2, markerscale=2.5)
    style_axes(ax)

    ax = axes[1, 0]
    n15 = [int(g["mask"].eq(15).sum()) for _, g in groups]
    n10 = [int(g["mask"].eq(10).sum()) for _, g in groups]
    x = np.arange(len(groups))
    ax.bar(x, n15, width=0.55, color=C["accent3"], label="掩码 15")
    ax.bar(x, n10, bottom=n15, width=0.55, color=C["accent2"], label="掩码 10")
    span = max(max(a + b for a, b in zip(n15, n10)), 1)
    for xi, a, b in zip(x, n15, n10):
        ax.text(xi, a + b + 0.02 * span, f"{a + b}", ha="center", fontsize=8.5,
                fontweight="bold")
        if a > 0.1 * span:
            ax.text(xi, a / 2, f"{a}", ha="center", fontsize=8, color="white")
        if b > 0.1 * span:
            ax.text(xi, a + b / 2, f"{b}", ha="center", fontsize=8, color="white")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("特征记录数")
    ax.set_title("(c) 各目标掩码构成（绿=掩码 15，橙=掩码 10）", fontsize=10.5)
    ax.grid(axis="x", visible=False)
    per = [f"{lab} {100.0 * b / max(a + b, 1):.0f}%" for lab, a, b in zip(labels, n15, n10)]
    ax.annotate("掩码 10 占比：\n" + "\n".join(per),
                xy=(0.5, 0.12), xycoords="axes fraction", fontsize=7.4, va="center",
                ha="center",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#c8d0d8",
                          alpha=0.92))
    style_axes(ax)

    ax = axes[1, 1]
    data = [(g["t"] - g["rec_cyc"]).to_numpy(float) for _, g in groups]
    bp = ax.boxplot(data, widths=0.55, patch_artist=True, showfliers=False,
                    medianprops=dict(color="#1b2733", lw=1.2))
    for k, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(CYCLE[k])
        patch.set_alpha(0.65)
        patch.set_edgecolor("#4a4f55")
    for k, (tid, g) in enumerate(groups):
        med = float(np.median(g["t"] - g["rec_cyc"]))
        ax.text(k + 1, med + 0.02 * max(float(lag.max()), 50.0), f"{med:.0f}",
                ha="center", fontsize=8, fontweight="bold")
    ax.axhline(-1, color=C["warn"], ls="--", lw=1.0)
    ax.set_xticks(range(1, len(groups) + 1))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("$t - t_{rec}$ (s)")
    ax.set_title("(d) 记录滞后分布（箱体 25–75%）", fontsize=10.5)
    ax.set_ylim(-30, max(50.0, float(lag.max()) * 1.15))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 20
def fig20(D):
    rc = D["radar_output"]["rcs_feat"].sort_values("t")
    groups = by_target(rc)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 20  RCS 特征（按目标）",
                f"{len(rc)} 条记录，{len(groups)} 个目标"
                f"（{'、'.join(target_title(t, g['track']) for t, g in groups)}）；"
                f"均值档位、起伏 σ 与归一化值逐目标统计，阴影为该目标 ±1σ")

    ax = axes[0, 0]
    for k, (tid, g) in enumerate(groups):
        g = g.sort_values("t")
        ax.plot(g["t"], g["mean"], "o-", ms=2.5, lw=1.0, color=CYCLE[k],
                label=target_title(tid, g["track"]))
        mu = float(np.median(g["mean"]))
        sig = float(np.median(g["std"]))
        ax.axhspan(mu - sig, mu + sig, color=CYCLE[k], alpha=0.10, lw=0)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("RCS 均值 (dBsm)")
    ax.set_title("(a) 各目标 RCS 均值随时间", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.4, ncol=2, markerscale=2.5)
    ax.annotate("阴影为各目标 ±1σ（中位）带宽",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.2, va="bottom",
                bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[0, 1]
    for k, (tid, g) in enumerate(groups):
        g = g.sort_values("t")
        ax.plot(g["t"], g["std"], "o-", ms=2.2, lw=0.9, color=CYCLE[k],
                label=target_title(tid, g["track"]))
    ax.axhline(rc["std"].median(), color=C["neutral"], ls="--", lw=1.0,
               label=f"整体中位 {rc['std'].median():.2f} dB")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("RCS 起伏 σ (dB)")
    ax.set_title("(b) 各目标起伏强度", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=7.2, ncol=2, markerscale=2.5)
    style_axes(ax)

    ax = axes[1, 0]
    for k, (tid, g) in enumerate(groups):
        v, f = _ecdf(g["std"].to_numpy(float))
        ax.step(v, f, where="post", lw=1.5, color=CYCLE[k],
                label=f"{target_title(tid, g['track'])}（{len(v)} 条）")
        ax.axvline(float(np.median(v)), color=CYCLE[k], ls=":", lw=1.0)
    ax.axhline(0.5, color=C["neutral"], ls="--", lw=0.9)
    ax.set_xlabel("起伏 σ (dB)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title("(c) 各目标 σ 经验分布", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.8)
    style_axes(ax)

    ax = axes[1, 1]
    ax.axis("off")
    head = ["目标", "记录数", "RCS 均值档位", "σ 中位 (dB)", "起伏度中位"]
    rows = [[target_title(tid, g["track"]), str(len(g)), _tier_txt(g),
             f"{g['std'].median():.2f}", f"{g['fluct'].median():.3f}"]
            for tid, g in groups]
    ax.text(0.0, 0.98, "逐目标 RCS 特征汇总", fontsize=10.5, fontweight="bold",
            va="top", color="#2c3e50")
    y = 0.86
    for j, h in enumerate(head):
        ax.text(0.0 + j * 0.20, y, h, fontsize=8.6, fontweight="bold",
                color="#5b6773", va="center")
    ax.plot([0.0, 0.98], [y - 0.04, y - 0.04], color="#c8d0d8", lw=0.9)
    for i, r in enumerate(rows):
        yy = y - 0.10 - i * 0.10
        for j, v in enumerate(r):
            ax.text(0.0 + j * 0.20, yy, v, fontsize=8.4, va="center")
        ax.plot([0.0, 0.98], [yy - 0.04, yy - 0.04], color="#e3e8ec", lw=0.7)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    return fig


# ---------------------------------------------------------------- 图 21
def fig21(D):
    tf = D["radar_int"]["trk_feat"].sort_values("t")
    groups = by_target(tf)
    m_all = tf["margin"].to_numpy(float)
    s1_all = tf["s1"].to_numpy(float)
    s2_all = tf["s2"].to_numpy(float)
    t0, t1 = float(tf["t"].min()), float(tf["t"].max())
    pad = 0.02 * max(t1 - t0, 1.0)
    ylo = max(0.0, float(min(s1_all.min(), s2_all.min())) - 0.05)
    yhi = min(1.0, float(max(s1_all.max(), s2_all.max())) + 0.05)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 21  各目标的识别置信度与判决结果",
                f"{len(tf)} 条识别记录，{len(groups)} 个目标；置信度 $s_1$ "
                f"{s1_all.min():.3f}–{s1_all.max():.3f}、$s_2$ "
                f"{s2_all.min():.3f}–{s2_all.max():.3f}；判别余量 "
                f"{m_all.min():.4f}–{m_all.max():.4f}")

    ax = axes[0, 0]
    for k, (tid, g) in enumerate(groups):
        ax.plot(g["t"], g["s1"], lw=0.8, color=CYCLE[k], alpha=0.85,
                label=f"{target_title(tid, g['track'])} $s_1$")
        ax.plot(g["t"], g["s2"], lw=0.8, color=CYCLE[k], alpha=0.45, ls="--")
    ax.set_xlim(t0 - pad, t1 + pad)
    ax.set_ylim(ylo, yhi)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("置信度（实线 $s_1$ / 虚线 $s_2$）")
    ax.set_title("(a) 各目标两类置信度时序", fontsize=10.5)
    ax.legend(loc="center right", fontsize=6.8, ncol=2)
    style_axes(ax)

    ax = axes[0, 1]
    lo = float(np.nanmin(m_all)) - 0.02
    hi = float(np.nanmax(m_all)) + 0.02
    bins = np.linspace(lo, hi, 40)
    for k, (tid, g) in enumerate(groups):
        ax.hist(g["margin"].to_numpy(float), bins=bins, density=True,
                histtype="step", lw=1.4, color=CYCLE[k],
                label=f"{target_title(tid, g['track'])}（{len(g)}）")
    ax.axvline(np.median(m_all), color=C["warn"], ls="--", lw=1.2,
               label=f"整体中位 {np.median(m_all):.4f}")
    ax.set_xlabel("判别余量 $s_1-s_2$")
    ax.set_ylabel("概率密度 (1/余量)")
    ax.set_title("(b) 判别余量分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.4)
    ax.annotate(f"整体最小 {m_all.min():.4f}；5% 分位 {np.percentile(m_all, 5):.4f}\n"
                f"余量 < 0 的样本 {int((m_all < 0).sum())} 条",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    for k, (tid, g) in enumerate(groups):
        unknown = g["cls"].astype(str).eq("未知")
        ax.scatter(g["t"][~unknown], np.full((~unknown).sum(), k),
                   s=5, color=C["accent3"], alpha=0.5, linewidths=0)
        ax.scatter(g["t"][unknown], np.full(unknown.sum(), k), s=30,
                   marker="v", color=C["warn"], linewidths=0)
    ax.set_xlim(t0 - pad, t1 + pad)
    ax.set_ylim(-0.5, len(groups) + 1.0)
    ax.set_yticks(range(len(groups)))
    ax.set_yticklabels([target_title(t, g["track"]) for t, g in groups], fontsize=8)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("目标")
    ax.set_title("(c) 识别结果时间线（每行一个目标）", fontsize=10.5)
    ax.legend(handles=[Line2D([], [], marker="o", ls="", ms=4.5,
                              color=C["accent3"], label="弹道目标"),
                       Line2D([], [], marker="v", ls="", ms=6,
                              color=C["warn"], label="判为未知")],
              loc="upper right", fontsize=8.5)
    ax.grid(axis="y", visible=False)
    style_axes(ax)

    ax = axes[1, 1]
    for k, (tid, g) in enumerate(groups):
        ax.scatter(g["s1"], g["margin"], s=8, color=CYCLE[k], alpha=0.5,
                   linewidths=0, label=f"{target_title(tid, g['track'])}（{len(g)}）")
    ax.axhline(0, color=C["neutral"], ls=":", lw=0.9)
    ax.set_xlabel("$s_1$")
    ax.set_ylabel("判别余量 $s_1-s_2$")
    ax.set_title("(d) 余量与主类置信度", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.2, markerscale=2.0)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 22
def fig22(D):
    ro = D["radar_output"]
    r = ro["rcs_feat"][["tid", "track", "t", "contrib"]].rename(columns={"contrib": "RCS"})
    p = ro["pol_feat"][["tid", "track", "t", "contrib"]].rename(columns={"contrib": "极化"})
    h = ro["hrrp"][["tid", "track", "t", "contrib"]].rename(columns={"contrib": "HRRP"})
    j = (r.merge(p, on=["tid", "track", "t"]).merge(h, on=["tid", "track", "t"])
         .dropna(subset=["tid", "track"]))
    groups = by_target(j)
    colors = [C["accent1"], C["accent2"], C["accent3"]]

    # 主目标＝同时刻特征最多的目标；堆叠图只画它，避免把不同目标的加权概率平均。
    p_tid, pj = groups[0]
    g = pj.groupby("t")[COLS].mean().sort_index()
    tot = j[COLS].sum(axis=1).to_numpy(float)
    gt0, gt1 = float(g.index.min()), float(g.index.max())
    gpad = 0.02 * max(gt1 - gt0, 1.0)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 22  识别特征加权概率构成（按目标）",
                f"{len(j)} 组同时刻特征，{len(groups)} 个目标"
                f"（{'、'.join(target_title(t, gg['track']) for t, gg in groups)}）；"
                f"(a)(c)(d) 画主目标 {p_tid}（{len(pj)} 组）；"
                f"三项加权概率之和 {tot.min():.2f}–{tot.max():.2f}（≤1）")

    ax = axes[0, 0]
    ax.stackplot(g.index, [g[c] for c in COLS], labels=COLS, colors=colors,
                 alpha=0.85, linewidth=0)
    ax.set_xlim(gt0 - gpad, gt1 + gpad)
    ax.set_ylim(0, max(0.1, float(g[COLS].sum(axis=1).max()) * 1.32))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("加权概率")
    ax.set_title(f"(a) 目标 {p_tid} 的加权概率堆叠", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2, ncol=3)
    style_axes(ax)

    ax = axes[0, 1]
    means = np.asarray([[float(gg[c].mean()) for c in COLS] for _, gg in groups])
    x = np.arange(len(groups))
    bottom = np.zeros(len(groups))
    for i, c in enumerate(COLS):
        ax.bar(x, means[:, i], bottom=bottom, width=0.6, color=colors[i],
               label=c, alpha=0.9)
        bottom += means[:, i]
    for xi, b in zip(x, bottom):
        ax.text(xi, b + 0.015, f"{b:.2f}", ha="center", fontsize=8.5,
                fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels([target_title(t, gg["track"]) for t, gg in groups], fontsize=8)
    ax.set_ylim(0, max(0.05, float(bottom.max()) * 1.15))
    ax.set_ylabel("平均加权概率")
    ax.set_title("(b) 各目标平均加权概率（逐目标，不跨目标平均）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2, ncol=3)
    ax.grid(axis="x", visible=False)
    ax.annotate("柱顶为三项之和",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0, va="top",
                bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    ax.hist(pj[COLS].sum(axis=1).to_numpy(float), bins=40, color=C["accent4"],
            alpha=0.82, edgecolor="white", linewidth=0.4)
    med = float(np.median(pj[COLS].sum(axis=1)))
    ax.axvline(med, color=C["warn"], ls="--", lw=1.2, label=f"中位 {med:.3f}")
    lo, hi = float(tot.min()), float(tot.max())
    t_pad = 0.05 * max(hi - lo, 0.05)
    ax.set_xlim(lo - t_pad, hi + t_pad)
    ax.set_xlabel("三项加权概率之和")
    ax.set_ylabel("样本数")
    ax.set_title(f"(c) 目标 {p_tid} 的加权概率之和分布（未归一）", fontsize=10.5)
    ax.legend(loc="center left", fontsize=8.2)
    ax.annotate(f"取值 {lo:.2f}–{hi:.2f}（归一化参考 1.0）",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3,
                va="top", ha="left",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    bp = ax.boxplot([pj[c].to_numpy(float) for c in COLS], widths=0.5,
                    patch_artist=True, showfliers=False,
                    medianprops=dict(color="#1b2733", lw=1.2))
    for k, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(colors[k])
        patch.set_alpha(0.75)
        patch.set_edgecolor("#4a4f55")
    ax.set_xticks(range(1, len(COLS) + 1))
    ax.set_xticklabels(COLS)
    ax.set_ylabel("加权概率")
    ax.set_title(f"(d) 目标 {p_tid} 各特征加权概率分布", fontsize=10.5)
    yoff = 0.03 * max(float(np.ptp(pj[c].to_numpy(float))) for c in COLS)
    for k, c in enumerate(COLS):
        m = float(np.median(pj[c]))
        ax.text(k + 1, m + yoff, f"{m:.3f}", ha="center", fontsize=8.5,
                fontweight="bold")
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 23
def fig23(D):
    h_all = D["radar_output"]["hrrp"].sort_values("t")
    groups = by_target(h_all)
    pat = re.compile(r"\(([-\d.]+)m,([-\d.]+)dBsm\)")

    # 瀑布图只画记录最多的目标：不同目标的距离像混在一张 3D 图里没有意义。
    p_tid, p_df = groups[0]

    def _peaks(df):
        rows = []
        for s_ in df["centers"]:
            mm = pat.findall(str(s_))
            rows.append([float(mm[0][0]), float(mm[0][1]),
                         float(mm[1][0]) if len(mm) > 1 else np.nan,
                         float(mm[1][1]) if len(mm) > 1 else np.nan])
        return np.asarray(rows, dtype=float)

    arr = _peaks(p_df)
    a1, a2 = arr[:, 1], arr[:, 3]
    t = p_df["t"].to_numpy(float)
    d = a2 - a1

    fig = plt.figure(figsize=(13.4, 8.2))
    title_block(fig, "图 23  HRRP 双峰距离像与散射结构（按目标）",
                f"{len(h_all)} 帧距离像，{len(groups)} 个目标"
                f"（{'、'.join(target_title(t_, g['track']) for t_, g in groups)}）；"
                f"瀑布图取目标 {p_tid}（{len(p_df)} 帧）；"
                f"峰 1 {np.nanmin(a1):.1f}–{np.nanmax(a1):.1f} dBsm，"
                f"峰 2 {np.nanmin(a2):.1f}–{np.nanmax(a2):.1f} dBsm，"
                f"峰间差 {np.nanmin(d):.1f}–{np.nanmax(d):.1f} dB")

    ax = fig.add_subplot(2, 2, 1, projection="3d")
    idx = np.linspace(0, len(p_df) - 1, min(90, len(p_df))).astype(int)
    cmap = plt.get_cmap("viridis")
    tn = (t - t.min()) / max(t.max() - t.min(), 1.0)
    for i in idx:
        ax.plot([arr[i, 0], arr[i, 2]], [t[i], t[i]], [a1[i], a2[i]],
                color=cmap(tn[i]), lw=1.2, alpha=0.85)
    ax.set_xlabel("距离 (m)", labelpad=2)
    ax.set_ylabel("时间 (s)", labelpad=2)
    ax.set_zlabel("幅度 (dBsm)", labelpad=4, fontsize=9)
    ax.zaxis.set_rotate_label(False)
    ax.zaxis.label.set_rotation(90)
    ax.set_title(f"(a) 目标 {p_tid} 距离像瀑布（抽稀 {len(idx)} 帧）",
                 fontsize=10.5, pad=2)
    ax.view_init(elev=24, azim=-58)
    ax.tick_params(labelsize=7.5)

    ax = fig.add_subplot(2, 2, 2)
    for k, (tid, g) in enumerate(groups):
        gg = g.sort_values("t")
        a = _peaks(gg)
        tg = gg["t"].to_numpy(float)
        ax.plot(tg, a[:, 1], lw=0.5, color=CYCLE[k], alpha=0.35)
        ax.plot(tg, a[:, 3], lw=0.5, color=CYCLE[k], alpha=0.35, ls="--")
        ax.plot(tg, _rollmed(a[:, 1]), lw=1.4, color=CYCLE[k])
        ax.plot(tg, _rollmed(a[:, 3]), lw=1.4, color=CYCLE[k], ls="--")
    ax.set_xlim(*W_RADAR)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("幅度 (dBsm)")
    ax.set_title("(b) 各目标两峰幅度时序（实线峰 1 / 虚线峰 2）", fontsize=10.5)
    ax.legend(handles=[Line2D([], [], color=CYCLE[k], lw=1.6,
                              label=target_title(t_, g["track"]))
                       for k, (t_, g) in enumerate(groups)],
              loc="lower right", fontsize=7.4)
    style_axes(ax)

    ax = fig.add_subplot(2, 2, 3)
    for k, (tid, g) in enumerate(groups):
        a = _peaks(g)
        ax.scatter(a[:, 1], a[:, 3], s=7, color=CYCLE[k], alpha=0.5,
                   linewidths=0, label=target_title(tid, g["track"]))
    lim = [float(np.nanmin(arr[:, 1])) - 1, float(np.nanmax(arr[:, 1])) + 1]
    ax.plot(lim, lim, color=C["neutral"], ls="--", lw=1.0, label="等幅线")
    ax.set_xlim(*lim)
    ax.set_ylim(float(np.nanmin(arr[:, 3])) - 1, float(np.nanmax(arr[:, 3])) + 1)
    ax.set_xlabel("峰 1 幅度 (dBsm)")
    ax.set_ylabel("峰 2 幅度 (dBsm)")
    ax.set_title("(c) 双峰幅度相关性", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=7.2, markerscale=2.0)
    style_axes(ax)

    ax = fig.add_subplot(2, 2, 4)
    lo = float(np.nanmin(d)) - 0.5
    hi = float(np.nanmax(d)) + 0.5
    bins = np.linspace(lo, hi, 36)
    for k, (tid, g) in enumerate(groups):
        a = _peaks(g)
        dd = a[:, 3] - a[:, 1]
        ax.hist(dd, bins=bins, density=True, histtype="step", lw=1.4,
                color=CYCLE[k], label=f"{target_title(tid, g['track'])}（{len(dd)}）")
        ax.axvline(float(np.nanmedian(dd)), color=CYCLE[k], ls=":", lw=1.0)
    ax.axvline(0, color=C["neutral"], lw=1.0)
    ax.set_xlabel("峰间差 $a_2-a_1$ (dB)")
    ax.set_ylabel("概率密度 (1/dB)")
    ax.set_title("(d) 峰间差分布", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.4)
    ax.annotate(f"主目标 {p_tid}：峰 2 弱于峰 1 的 {int((d < 0).sum())}/{len(d)} 帧"
                f"（{100*(d < 0).mean():.1f}%）",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 24
def fig24(D):
    p_all = D["radar_output"]["pol_feat"].sort_values("t")
    groups = by_target(p_all)
    det_db = db10(p_all["det_m"].to_numpy(float))
    span_db = db10(p_all["span_m"].to_numpy(float))
    ratio = (p_all["span_m"] / p_all["det_m"]).to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.0))
    title_block(fig, "图 24  极化不变量与散射结构（按目标）",
                f"{len(p_all)} 组极化特征，{len(groups)} 个目标"
                f"（{'、'.join(target_title(t_, g['track']) for t_, g in groups)}）；"
                f"行列式 {np.nanmin(det_db):.1f}–{np.nanmax(det_db):.1f} dBW，"
                f"Span {np.nanmin(span_db):.1f}–{np.nanmax(span_db):.1f} dBW；"
                f"Span/det 中位 {np.nanmedian(ratio):.3f}")

    ax = axes[0, 0]
    for k, (tid, g) in enumerate(groups):
        g = g.sort_values("t")
        ax.plot(g["t"], db10(g["det_m"].to_numpy(float)), lw=0.6,
                color=CYCLE[k], alpha=0.85,
                label=f"{target_title(tid, g['track'])} 行列式")
        ax.plot(g["t"], db10(g["span_m"].to_numpy(float)), lw=0.6,
                color=CYCLE[k], alpha=0.5, ls="--")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率量 (dBW)")
    ax.set_title("(a) 行列式（实线）与 Span（虚线）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.0, ncol=2)
    style_axes(ax)

    ax = axes[0, 1]
    for k, (tid, g) in enumerate(groups):
        rr = (g["span_m"] / g["det_m"]).to_numpy(float)
        v, f = _ecdf(rr)
        ax.step(v, f, where="post", lw=1.5, color=CYCLE[k],
                label=f"{target_title(tid, g['track'])}（{len(v)}）")
        ax.axvline(float(np.nanmedian(v)), color=CYCLE[k], ls=":", lw=1.0)
    ax.axhline(0.5, color=C["neutral"], ls="--", lw=0.9)
    ax.set_xlabel("Span / det")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title("(b) 不变量比值分布（按目标）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.4)
    style_axes(ax)

    ax = axes[1, 0]
    for k, (tid, g) in enumerate(groups):
        ell = np.abs(g["ell_m"].to_numpy(float))
        size = 6.0 + 24.0 * (ell - np.nanmin(ell)) / max(np.nanmax(ell) - np.nanmin(ell), 1e-9)
        ax.scatter(g["ori_m"], g["depol_m"], s=size, color=CYCLE[k], alpha=0.55,
                   linewidths=0, label=target_title(tid, g["track"]))
    ax.set_xlabel("取向角 (°)")
    ax.set_ylabel("退极化比")
    ax.set_title("(c) 取向角–退极化（点大小 ∝ |椭圆率|）", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.4, markerscale=1.6)
    style_axes(ax)

    ax = axes[1, 1]
    data = [g["ell_m"].to_numpy(float) for _, g in groups]
    bp = ax.boxplot(data, widths=0.5, patch_artist=True, showfliers=False,
                    medianprops=dict(color="#1b2733", lw=1.2))
    for k, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(CYCLE[k])
        patch.set_alpha(0.7)
        patch.set_edgecolor("#4a4f55")
    ax.set_xticks(range(1, len(groups) + 1))
    ax.set_xticklabels([target_title(t_, g["track"]) for t_, g in groups], fontsize=8)
    ax.set_ylabel("椭圆率角 (°)")
    ax.set_title("(d) 各目标椭圆率分布", fontsize=10.5)
    ax.annotate(f"整体 {p_all['ell_m'].min():.1f}…{p_all['ell_m'].max():.1f}°",
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
    M = piv.to_numpy(float)
    # 计数改用 dB（相对 1 条）：0 条保持留白，颜色仍按对数尺度铺开。
    Md = np.where(M > 0, 10.0 * np.log10(np.maximum(M, 1.0)), np.nan)
    cmap = plt.get_cmap("Blues").copy()
    cmap.set_bad("#f2f4f6")
    im = ax.imshow(Md, aspect="auto", cmap=cmap,
                   norm=plt.Normalize(vmin=0.0, vmax=float(np.nanmax(Md))),
                   extent=[-0.5, len(sats) - 0.5, len(events) - 0.5, -0.5])
    for r_ in range(len(events)):
        for c_ in range(len(sats)):
            v = int(M[r_, c_])
            ax.text(c_, r_, f"{v}", ha="center", va="center", fontsize=9.5,
                    color="white" if v > 0.15 * float(np.nanmax(M)) else "#1b2733",
                    fontweight="bold" if v else "normal")
    ax.set_xticks(range(len(sats)))
    ax.set_xticklabels([f"卫星 {int(s)}" for s in sats])
    ax.set_yticks(range(len(events)))
    ax.set_yticklabels(events)
    ax.set_title("(b) 事件类型 × 卫星计数", fontsize=10.5)
    _cbar(fig, im, ax, "事件数 (dB，相对 1 条)")
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
    dt_db = db10(dt)                       # 间隔按 dB(s) 表示，0 间隔本身无定义
    dt_db = np.sort(dt_db[np.isfinite(dt_db)])
    ax.step(dt_db, np.arange(1, len(dt_db) + 1) / max(len(dt_db), 1), where="post",
            lw=1.6, color=C["accent1"])
    ax.set_xlim(float(np.nanmin(dt_db)) - 0.5, float(np.nanmax(dt_db)) + 0.5)
    ax.set_ylim(0, 1.03)
    dpos = dt[dt > 0]
    if len(dpos):
        vals, cnts = np.unique(dpos, return_counts=True)
        mode = float(vals[np.argmax(cnts)])
        share_mode = 100.0 * float(cnts.max()) / len(dt)
        ax.axvline(float(db10(mode)), color=C["warn"], ls="--", lw=1.1,
                   label=f"{mode:g} s（{float(db10(mode)):.1f} dB）："
                         f"{share_mode:.1f}% 的间隔")
    ax.axhline(0.5, color=C["neutral"], ls=":", lw=0.9)
    ax.set_xlabel("相邻事件间隔 (dB，相对 1 s)")
    ax.set_ylabel("累积概率")
    ax.set_title("(d) 相邻事件间隔累积分布", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8.0)
    ax.annotate(f"{len(dt_db)} 个间隔，来自 {ev.groupby(['sat', 'tid']).ngroups} 条链路；"
                f"最长 {dt.max():.0f} s",
                xy=(0.97, 0.45), xycoords="axes fraction", fontsize=8.0,
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
    ("22", "fig22_feature_contrib", "图 22 识别特征加权概率", fig22),
    ("23", "fig23_hrrp_waterfall", "图 23 HRRP 瀑布", fig23),
    ("24", "fig24_pol_invariants", "图 24 极化不变量", fig24),
    ("25", "fig25_event_timeline", "图 25 事件时间轴", fig25),
]
