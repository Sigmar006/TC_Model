# -*- coding: utf-8 -*-
"""图 36–39：红外辐射链路、大气透过率、跟踪一致性与宽场候选。

坐标口径
--------
* 信噪比、接收功率、能量一律换算成 dB 后线性绘制（10·lg），不使用对数坐标轴；
* 链路各阶段的记录量用 dB 表示，柱端标原始条数；
* NIS 与 χ²(2) 门限直接比较，超门比例按滚动窗口统计。
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm
from scipy.stats import chi2

from .common import C, CYCLE, db10, needs, style_axes, title_block


def _rng_txt(v, unit="", nd=2):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if not v.size:
        return "未记录"
    return f"{v.min():.{nd}f}–{v.max():.{nd}f}{unit}"


def _safe_db(x, floor=1e-30):
    return db10(np.clip(np.asarray(x, dtype=float), floor, None))


# ---------------------------------------------------------------- 图 36
@needs(("ir_int", "ir_rad", "红外辐射噪声与检测判决交互接口测试"),
       title="图 36 红外辐射 SNR–距离与检测门限")
def fig36(D):
    rad = D["ir_int"]["ir_rad"]
    snr = rad["snr"].to_numpy(float)
    snr_db = _safe_db(snr)
    rng = rad["rng"].to_numpy(float) / 1e3
    tau = rad["tau"].to_numpy(float)
    p_rx = _safe_db(rad["p_rx"].to_numpy(float))
    thr_lin = float(np.nanmedian(rad["thr"].to_numpy(float)))
    thr_db = float(db10(thr_lin))
    fov_ok = rad["fov_res"].astype(str).eq("通过").to_numpy()
    admit_ok = rad["admit"].astype(str).eq("通过").to_numpy()
    snr_ok = rad["snr_res"].astype(str).eq("通过").to_numpy()

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 36  红外辐射 SNR–距离、接收功率与检测门限",
                f"辐射记录 {len(rad)} 条，SNR {_rng_txt(snr_db, ' dB', 1)}；"
                f"门限 {thr_lin:.0f}（≈{thr_db:.1f} dB）；"
                f"视场门通过 {int(fov_ok.sum())} 条")

    ax = axes[0, 0]
    hb = ax.hexbin(rng, snr_db, gridsize=(42, 30), cmap="viridis",
                   mincnt=1, norm=PowerNorm(gamma=0.2, vmin=1.0))
    cb = fig.colorbar(hb, ax=ax, pad=0.02)
    cb.set_label("样本数（γ=0.2 提亮）", fontsize=8)
    cb.ax.tick_params(labelsize=7)
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
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)

    ax = axes[0, 1]
    sc = ax.scatter(rng, p_rx, c=tau, s=5, cmap="plasma", alpha=0.35,
                    linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("大气透过率 τ", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    if rng.size:
        # 1/R² 参考线锚定 τ≈1 的最近距离样本：τ 更低的样本叠加了大气衰减，
        # 直接对全部样本取中点会把参考线压到两条带之间，故取 τ 最高的样本。
        hi_tau = tau >= 0.98
        pool = np.where(hi_tau)[0] if hi_tau.any() else np.arange(rng.size)
        i0 = int(pool[np.argmin(rng[pool])])
        xs = np.linspace(float(np.min(rng)), float(np.max(rng)), 60)
        ref = p_rx[i0] - 20.0 * np.log10(xs / rng[i0])
        ax.plot(xs, ref, color=C["ok"], ls="--", lw=1.1,
                label=f"1/R² 参考（τ≈1 锚定 {rng[i0]:.0f} km）")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("接收功率 (dBW)")
    ax.set_title("(b) 接收功率–距离（颜色为 τ）", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=7.6)
    style_axes(ax)

    ax = axes[1, 0]
    ax.hist(snr_db, bins=44, color=C["accent2"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.axvline(thr_db, color=C["warn"], ls="--", lw=1.1,
               label=f"门限 {thr_db:.1f} dB")
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("辐射记录数")
    ax.set_title("(c) SNR 分布与门限", fontsize=10.5)
    ax.annotate(f"SNR 门通过 {int(snr_ok.sum())}/{len(snr_ok)}\n"
                f"视场门通过 {int(fov_ok.sum())}/{len(fov_ok)}\n"
                f"综合准入 {int(admit_ok.sum())}/{len(admit_ok)}",
                xy=(0.14, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    margin = snr_db - thr_db
    ax.hist(margin, bins=44, color=C["accent1"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.set_xlabel("SNR 余量（SNR − 门限） (dB)")
    ax.set_ylabel("辐射记录数")
    ax.set_title("(d) 检测余量分布", fontsize=10.5)
    ax.annotate(f"余量 {_rng_txt(margin, ' dB', 1)}\n"
                f"低于门限 {int((margin < 0).sum())} 条",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 37
@needs(("ir_int", "ir_rad", "红外辐射噪声与检测判决交互接口测试"),
       title="图 37 大气透过率与辐射传输")
def fig37(D):
    rad = D["ir_int"]["ir_rad"]
    rng = rad["rng"].to_numpy(float) / 1e3
    tau = rad["tau"].to_numpy(float)
    snr_db = _safe_db(rad["snr"].to_numpy(float))
    p_rx = _safe_db(rad["p_rx"].to_numpy(float))
    e_sig = _safe_db(rad["e_sig"].to_numpy(float))
    e_noise = _safe_db(rad["e_noise"].to_numpy(float))

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 37  大气透过率、接收功率与能量预算",
                f"斜距 {_rng_txt(rng, ' km', 0)}，τ {_rng_txt(tau, '', 3)}；"
                f"信号能量 {_rng_txt(e_sig, ' dBJ', 1)}")

    ax = axes[0, 0]
    hb = ax.hexbin(rng, tau, gridsize=(42, 30), cmap="viridis",
                   mincnt=1, norm=PowerNorm(gamma=0.2, vmin=1.0))
    cb = fig.colorbar(hb, ax=ax, pad=0.02)
    cb.set_label("样本数（γ=0.2 提亮）", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("大气透过率 τ")
    ax.set_title("(a) τ–距离密度", fontsize=10.5)
    style_axes(ax)

    ax = axes[0, 1]
    ax.hist(tau, bins=44, color=C["accent3"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    q10, q50, q90 = np.percentile(tau, [10, 50, 90])
    ax.axvline(q50, color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {q50:.3f}")
    ax.axvline(q10, color=C["accent4"], ls=":", lw=1.0,
               label=f"10% {q10:.3f}")
    ax.axvline(q90, color=C["accent4"], ls=":", lw=1.0,
               label=f"90% {q90:.3f}")
    ax.set_xlabel("大气透过率 τ")
    ax.set_ylabel("辐射记录数")
    ax.set_title("(b) τ 分布", fontsize=10.5)
    ax.legend(loc="center right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    hb = ax.hexbin(rng, p_rx, gridsize=(42, 30), cmap="magma",
                   mincnt=1, norm=PowerNorm(gamma=0.2, vmin=1.0))
    cb = fig.colorbar(hb, ax=ax, pad=0.02)
    cb.set_label("样本数（γ=0.2 提亮）", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("接收功率 (dBW)")
    ax.set_title("(c) 接收功率–距离", fontsize=10.5)
    style_axes(ax)

    ax = axes[1, 1]
    sc = ax.scatter(tau, snr_db, c=rng, s=5, cmap="viridis", alpha=0.35,
                    linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("斜距 (km)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("大气透过率 τ")
    ax.set_ylabel("SNR (dB)")
    ax.set_title("(d) SNR 与 τ（颜色为距离）", fontsize=10.5)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 38
@needs(("ir_int", "ir_trk", "红外量测与跟踪滤波交互接口测试"),
       title="图 38 红外跟踪 NIS 一致性")
def fig38(D):
    trk = D["ir_int"]["ir_trk"].sort_values("t")
    nis = trk["nis"].to_numpy(float)
    t = trk["t"].to_numpy(float)
    over = trk["nis_res"].astype(str).str.startswith("超门").to_numpy()
    g95 = float(chi2.ppf(0.95, 2))
    g99 = float(chi2.ppf(0.99, 2))

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 38  红外跟踪 NIS 与新息一致性",
                f"跟踪记录 {len(trk)} 条，NIS {_rng_txt(nis, '', 1)}；χ²(2) 95% 门限 {g95:.2f}、99% 门限 {g99:.2f}，超门 {int(over.sum())} 条")

    ax = axes[0, 0]
    ok = ~over
    nis_db = db10(nis)
    fin = np.isfinite(nis_db)
    n_zero = int((~fin).sum())
    ax.plot(t[ok & fin], nis_db[ok & fin], ".", ms=1.6,
            color=C["accent1"], alpha=0.5, label="通过")
    ax.plot(t[over & fin], nis_db[over & fin], ".", ms=1.6,
            color=C["warn"], alpha=0.6, label="超门")
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
    ax.legend(loc="lower right", fontsize=8, framealpha=0.9)
    style_axes(ax)

    ax = axes[0, 1]
    edges = np.linspace(0.0, 20.0, 45)
    binsz = float(edges[1] - edges[0])
    hi = nis[nis <= 20]
    # 用全样本归一化：截到 0～20 只是为了看清主峰，不能改变概率密度的分母。
    w = np.ones(hi.size) / (len(nis) * binsz)
    ax.hist(hi, bins=edges, weights=w, color=C["accent1"], alpha=0.78,
            edgecolor="white", linewidth=0.3, label="实测 NIS（全样本归一）")
    xs = np.linspace(0.01, 20, 400)
    ax.plot(xs, chi2.pdf(xs, 2), color=C["warn"], lw=1.4,
            label="χ²(2) 理论")
    ax.axvline(g95, color=C["ok"], ls="--", lw=0.9)
    ax.axvline(g99, color=C["accent4"], ls=":", lw=1.0)
    ax.set_xlabel("NIS")
    ax.set_ylabel("概率密度")
    ax.set_ylim(0, 0.56)
    ax.set_title("(b) NIS 分布与 χ²(2) 理论", fontsize=10.5)
    ax.annotate(f"中位 {float(np.median(nis)):.2f}（理论 1.39）\n"
                f"NIS>20 共 {int((nis > 20).sum())} 条（已计入归一化）\n"
                f"超 95% 门限 {float(over.mean())*100:.1f}%",
                xy=(0.42, 0.94), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="center right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 0]
    win = max(20, len(nis) // 30)
    rate = np.convolve(over.astype(float), np.ones(win) / win, mode="valid")
    ax.plot(t[win - 1:], rate * 100.0, color=C["accent2"], lw=1.2)
    ax.axhline(5.0, color=C["warn"], ls="--", lw=1.0, label="理论 5%")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("超门比例 (%)")
    ax.set_title(f"(c) 滚动超门率（{win} 点窗）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    pos = nis[nis > 0]
    s = np.sort(pos)
    p = (np.arange(1, s.size + 1) - 0.5) / s.size
    th = chi2.ppf(p, 2)
    th_db, s_db = db10(th), db10(s)
    ax.plot(th_db, s_db, ".", ms=2, color=C["accent1"],
            alpha=0.5, label="NIS 分位")
    lim = [float(np.nanmin(th_db)), float(np.nanmax(th_db))]
    ax.plot(lim, lim, color=C["warn"], ls="--", lw=1.0, label="1:1")
    ax.set_xlabel("χ²(2) 理论分位 (dB)")
    ax.set_ylabel("NIS 实测分位 (dB)")
    ax.set_title("(d) Q–Q 一致性（NIS>0）", fontsize=10.5)
    ax.annotate(f"NIS=0 的 {int((nis <= 0).sum())} 条（10·lg 无定义）\n"
                "已从 Q–Q 样本中剔除",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=8)
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

    az = trk["az"].to_numpy(float)
    el = trk["el"].to_numpy(float)
    caz = cand["az"].to_numpy(float)
    cel = cand["el"].to_numpy(float)
    csnr = _safe_db(cand["snr"].to_numpy(float))
    csat = cand["sat"].to_numpy(float)
    ct = cand["tmeas"].to_numpy(float)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 39  红外跟踪视线角、视场门与宽场候选",
                f"跟踪 {len(trk)} 条、候选 {len(cand)} 条；"
                f"几何门控记录 {len(geom)} 条")

    ax = axes[0, 0]
    hb = ax.hexbin(az, el, gridsize=(44, 32), cmap="viridis",
                   mincnt=1, norm=PowerNorm(gamma=0.2, vmin=1.0))
    cb = fig.colorbar(hb, ax=ax, pad=0.02)
    cb.set_label("跟踪样本数（γ=0.2 提亮）", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.plot(caz, cel, "o", ms=6, mfc="none", mec=C["warn"], mew=1.3,
            label=f"候选 {len(cand)} 条")
    ax.set_xlabel("方位 (°)")
    ax.set_ylabel("俯仰 (°)")
    ax.set_title("(a) 跟踪视线角分布与候选位置", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[0, 1]
    wf = geom[geom["stage"].astype(str).eq("WFOV")]
    res = wf["fov_res"].astype(str)
    sats = sorted(wf["sat"].dropna().unique())
    tot = [int((wf["sat"] == s).sum()) for s in sats]
    pas = [int(((wf["sat"] == s) & res.eq("通过")).sum()) for s in sats]
    rate = [100.0 * p_ / t_ if t_ else 0.0 for p_, t_ in zip(pas, tot)]
    x = np.arange(len(sats))
    ax.bar(x, rate, width=0.58, color=C["accent3"], alpha=0.9)
    ax.set_xticks(x)
    ax.set_xticklabels([f"卫星 {int(s)}" for s in sats], fontsize=8)
    ax.set_ylabel("视场门通过率 (%)")
    ax.set_ylim(0, max(rate + [0.0]) * 1.45 if max(rate) > 0 else 1.0)
    ax.set_title("(b) 各卫星视场门通过率", fontsize=10.5)
    for xi, p_, t_, r_ in zip(x, pas, tot, rate):
        # 通过数为 0 时柱高为 0，必须直接用文字标出计数，避免误读为漏画。
        ax.annotate(f"{p_}/{t_}\n{r_:.2f}%", xy=(xi, r_), xytext=(0, 3),
                    textcoords="offset points", ha="center", fontsize=6.8,
                    va="bottom",
                    color=C["accent3"] if p_ else C["warn"])
    ax.annotate(f"合计 {sum(pas)}/{sum(tot)} 条通过",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    occ = geom[geom["stage"].astype(str).eq("地球遮挡")]
    margin = occ["margin"].to_numpy(float) / 1e3
    margin = margin[np.isfinite(margin)]
    ax.hist(margin, bins=44, color=C["accent4"], alpha=0.82,
            edgecolor="white", linewidth=0.3)
    ax.set_xlabel("遮挡余量 (km)")
    ax.set_ylabel("记录数")
    ax.set_title("(c) 地球遮挡余量分布", fontsize=10.5)
    ax.annotate(f"{len(margin)} 条遮挡记录\n"
                f"余量 {_rng_txt(margin, ' km', 0)}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    sc = ax.scatter(ct, csnr, c=csat, s=26, cmap="viridis", alpha=0.9,
                    edgecolors="white", linewidths=0.4)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("卫星 ID", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("候选量测时刻 (s)")
    ax.set_ylabel("候选 SNR (dB)")
    ax.set_title("(d) 候选 SNR 与时刻", fontsize=10.5)
    ax.annotate(f"连续命中均为 {int(cand['hits'].min())}\n"
                f"候选状态：{cand['state'].iloc[0] if len(cand) else '无'}",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


FIGURES = [
    ("36", "fig36_ir_snr_range", "图 36 红外辐射 SNR–距离", fig36),
    ("37", "fig37_ir_tau_budget", "图 37 大气透过率与辐射传输", fig37),
    ("38", "fig38_ir_nis", "图 38 红外跟踪 NIS 一致性", fig38),
    ("39", "fig39_ir_coverage_cand", "图 39 红外视线角与候选", fig39),
]

