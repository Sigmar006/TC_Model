# -*- coding: utf-8 -*-
"""图 12–15：红外覆盖、跟踪、辐射检测与卫星导航/落点。

坐标口径
--------
* 信噪比、接收功率、协方差迹等一律换算成 dB 后线性绘制，不再出现对数刻度；
* 跟踪图（13）按目标分色分线，NIS、视线角速率与门限通过率逐目标统计；
* 辐射检测（14）的链路各阶段记录量也用 dB 表示，柱端仍标原始条数。
"""
from __future__ import annotations

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

from .common import (C, CYCLE, W_IR, by_target, db10, style_axes,
                     target_title, title_block)


def _valid_centers(wfov):
    """解析 wfov.center 的 'lat,lon'，返回有效掩码与数组。"""
    txt = wfov["center"].astype(str)
    ok = txt.ne("miss")
    lat = np.full(len(wfov), np.nan)
    lon = np.full(len(wfov), np.nan)
    vals = txt[ok].to_numpy()
    lat[ok.to_numpy()] = [float(v.split(",")[0]) for v in vals]
    lon[ok.to_numpy()] = [float(v.split(",")[1]) for v in vals]
    return ok.to_numpy(), lat, lon


def _bin_stat(x, y, nbins=40):
    """按 x 分箱，返回箱中心、中位数与 10/90 分位。"""
    edges = np.linspace(np.nanmin(x), np.nanmax(x), nbins + 1)
    idx = np.digitize(x, edges)
    cx, med, lo, hi = [], [], [], []
    for k in range(1, len(edges)):
        m = idx == k
        if m.sum() >= 20:
            cx.append(0.5 * (edges[k - 1] + edges[k]))
            med.append(np.median(y[m]))
            lo.append(np.percentile(y[m], 10))
            hi.append(np.percentile(y[m], 90))
    return (np.asarray(cx), np.asarray(med), np.asarray(lo), np.asarray(hi))


# ---------------------------------------------------------------- 图 12
def fig12(D):
    wf = D["sat"]["wfov"].sort_values("t")
    mr = D["sat"]["maxrange"].sort_values("t")
    det = D["sat"]["ir_detect"].sort_values("t")
    ok, lat, lon = _valid_centers(wf)
    vv = wf[ok].copy()
    vv["lat"] = lat[ok]
    vv["lon"] = lon[ok]

    cnt = vv.groupby("t").size().reindex(
        np.arange(int(wf["t"].min()), int(wf["t"].max()) + 1), fill_value=0)

    fig, axes = plt.subplots(2, 2, figsize=(13.6, 8.4))
    title_block(fig, "图 12  红外卫星宽视场覆盖与可用性",
                f"宽视场覆盖 {int(ok.sum())}/{len(wf)} 个星时有效（{cnt.gt(0).sum()} 个时刻），"
                f"最多 {int(cnt.max())} 星同时覆盖；目标视线角集中于阵面法向附近")

    ax = axes[0, 0]
    for k, (sat, g) in enumerate(vv.groupby("sat")):
        ax.scatter(g["lon"], g["lat"], s=6, color=CYCLE[k], alpha=0.7,
                   linewidths=0, label=f"卫星 {int(sat)}")
    ax.plot(-149.19, 64.29, marker="^", ms=10, color=C["warn"], ls="none",
            label="雷达站")
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xticks(np.arange(-180, 181, 60))
    ax.set_yticks(np.arange(-90, 91, 30))
    ax.set_xlabel("经度 (°)")
    ax.set_ylabel("纬度 (°)")
    ax.set_title("(a) 宽视场覆盖中心分布", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=7.5, ncol=2, markerscale=1.6)
    ax.set_aspect("equal", adjustable="box")
    style_axes(ax, minor=False)

    ax = axes[0, 1]
    sats = sorted(vv["sat"].unique())
    for k, sat in enumerate(sats):
        ts = np.sort(vv[vv["sat"] == sat]["t"].unique())
        segs = []
        start = prev = ts[0]
        for x in ts[1:]:
            if x - prev <= 1.01:
                prev = x
            else:
                segs.append((start, prev - start + 1))
                start = prev = x
        segs.append((start, prev - start + 1))
        ax.broken_barh(segs, (k - 0.32, 0.64), facecolors=CYCLE[k], alpha=0.85)
    ax.set_xlim(*W_IR)
    ax.set_ylim(-0.6, len(sats) - 0.4)
    ax.set_yticks(range(len(sats)))
    ax.set_yticklabels([f"卫星 {int(s)}" for s in sats])
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("覆盖卫星")
    ax.set_title("(b) 各卫星有效覆盖时段", fontsize=10.5)
    ax.annotate(f"覆盖时隙 {int(cnt.gt(0).sum())}/{len(cnt)}"
                f"（{100*cnt.gt(0).mean():.1f}%）\n"
                f"有效时段平均 {cnt[cnt.gt(0)].mean():.2f} 星同时覆盖",
                xy=(0.03, 0.96), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.grid(axis="y", visible=False)
    style_axes(ax)

    ax = axes[1, 0]
    for k, (sat, g) in enumerate(det.groupby("sat")):
        ax.scatter(g["az"], g["el"], s=4, color=CYCLE[k], alpha=0.5,
                   linewidths=0, label=f"卫星 {int(sat)}")
    ax.set_xlabel("方位视线角 (°)")
    ax.set_ylabel("俯仰视线角 (°)")
    ax.set_title("(c) 目标视线角分布", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=7.5, markerscale=1.8)
    ax.annotate(f"方位 {det['az'].min():.2f}…{det['az'].max():.2f}°\n"
                f"俯仰 {det['el'].min():.2f}…{det['el'].max():.2f}°",
                xy=(0.97, 0.95), xycoords="axes fraction", fontsize=8.3,
                va="top", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    mr = mr[mr["rmax"].notna()]
    data = [mr[mr["sat"] == s]["rmax"].to_numpy(float) / 1e7 for s in sorted(mr["sat"].unique())]
    parts = ax.violinplot(data, showmedians=True, showextrema=False, widths=0.75)
    for k, body in enumerate(parts["bodies"]):
        body.set_facecolor(CYCLE[k])
        body.set_alpha(0.7)
        body.set_edgecolor("none")
    parts["cmedians"].set_color("#1b2733")
    ax.set_xticks(range(1, len(data) + 1))
    ax.set_xticklabels([f"卫星 {int(s)}" for s in sorted(mr["sat"].unique())])
    ax.set_ylabel("最大作用距离 ($10^7$ m)")
    ax.set_title("(d) 各卫星最大作用距离分布", fontsize=10.5)
    ax.annotate(f"整体 {mr['rmax'].min()/1e7:.3f}–{mr['rmax'].max()/1e7:.3f}×10$^7$ m\n"
                f"相当于 {mr['rmax'].min()/1e3:.0f}–{mr['rmax'].max()/1e3:.0f} km",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.grid(axis="x", visible=False)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 13
def fig13(D):
    tr = D["ir_int"]["ir_trk"].sort_values("t")
    groups = by_target(tr)
    gate = 5.991
    gate_db = float(db10(gate))
    nis_all = tr["nis"].to_numpy(float)
    passed = nis_all <= gate
    nis_db = db10(nis_all)

    fig, axes = plt.subplots(2, 2, figsize=(13.2, 8.0))
    title_block(fig, "图 13  红外跟踪滤波与 NIS 一致性检验（按目标）",
                f"跟踪记录 {len(tr)} 条，{len(groups)} 个目标"
                f"（{'、'.join(target_title(t) for t, _ in groups)}）；"
                f"NIS 中位 {np.median(nis_all):.2f}（{float(db10(np.median(nis_all))):.2f} dB），"
                f"χ²(2, 95%) 门限 {gate}（{gate_db:.2f} dB），"
                f"通过率 {100*passed.mean():.1f}%")

    ax = axes[0, 0]
    for k, (tid, g) in enumerate(groups):
        t = g["t"].to_numpy(float)
        nis = g["nis"].to_numpy(float)
        y = db10(nis)
        ok = nis <= gate
        ax.scatter(t[ok], y[ok], s=3.5, color=CYCLE[k], alpha=0.35, linewidths=0,
                   label=target_title(tid))
        if (~ok).any():
            ax.scatter(t[~ok], y[~ok], s=7, facecolors="none",
                       edgecolors=C["warn"], linewidths=0.6)
        roll = pd.Series(y).rolling(101, min_periods=10).median().to_numpy()
        ax.plot(t, roll, color=CYCLE[k], lw=1.3)
    ax.axhline(gate_db, color=C["warn"], ls="--", lw=1.1,
               label=f"门限 {gate_db:.2f} dB")
    ax.set_xlim(*W_IR)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("NIS (dB)")
    ax.set_title("(a) NIS 时序与门限（空心点为超门限）", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=7.2, ncol=3)
    style_axes(ax)

    ax = axes[0, 1]
    lo = float(np.nanmin(nis_db)) - 1.0
    hi = float(np.nanmax(nis_db)) + 1.0
    bins = np.linspace(lo, hi, 62)
    for k, (tid, g) in enumerate(groups):
        ax.hist(db10(g["nis"].to_numpy(float)), bins=bins, density=True,
                histtype="step", lw=1.4, color=CYCLE[k],
                label=f"{target_title(tid)}（{len(g)} 条）")
    xs = np.linspace(lo, hi, 400)
    nu = 10 ** (xs / 10.0)
    pdf = 0.5 * np.exp(-nu / 2.0) * (np.log(10.0) / 10.0) * nu
    ax.plot(xs, pdf, color=C["warn"], lw=1.6, label="χ²(2) 理论密度")
    ax.axvline(gate_db, color=C["warn"], ls="--", lw=1.1)
    ax.axvline(float(np.nanmedian(nis_db)), color="#123f63", ls=":", lw=1.1)
    ax.set_xlabel("NIS (dB)")
    ax.set_ylabel("概率密度 (1/dB)")
    ax.set_title("(b) NIS 分布与理论对照", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.6)
    note = []
    for tid, g in groups:
        nis = g["nis"].to_numpy(float)
        iqr = float(np.percentile(nis, 75) - np.percentile(nis, 25))
        iqr_th = 2.0 * np.log(3.0)                  # χ²(2) 的四分位距
        if iqr > 1.15 * iqr_th:
            spread = "偏宽"
        elif iqr < 0.87 * iqr_th:
            spread = "偏窄"
        else:
            spread = "接近理论"
        note.append(f"{target_title(tid)}：中位 {float(db10(np.median(nis))):.2f} dB、"
                    f"超门限 {int((nis > gate).sum())} 条、{spread}")
    ax.annotate("\n".join(note), xy=(0.02, 0.97), xycoords="axes fraction",
                fontsize=7.4, va="top", ha="left",
                bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    for k, (tid, g) in enumerate(groups):
        ax.scatter(g["t"], np.degrees(g["los_rate"].to_numpy(float)), s=2.5,
                   color=CYCLE[k], alpha=0.35, linewidths=0,
                   label=target_title(tid))
    ax.set_xlim(*W_IR)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("视线角速率 (°/s)")
    ax.set_title("(c) 视线角速率随时间（按目标）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=7.8, ncol=2, markerscale=4.0)
    ax.annotate(f"中位 {np.degrees(np.median(tr['los_rate'])):.2f} °/s\n"
                f"最大 {np.degrees(tr['los_rate'].max()):.2f} °/s",
                xy=(0.03, 0.08), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    rates, labels = [], []
    for tid, g in groups:
        nis = g["nis"].to_numpy(float)
        rates.append(100.0 * float((nis <= gate).mean()))
        sats = sorted(int(v) for v in g["sat"].unique())
        labels.append(f"{target_title(tid)}\n卫星 {'、'.join(str(s) for s in sats)}")
    bars = ax.bar(range(len(rates)), rates, width=0.55,
                  color=[CYCLE[k] for k in range(len(rates))], alpha=0.88)
    for b, r in zip(bars, rates):
        ax.text(b.get_x() + b.get_width() / 2, r + 2, f"{r:.1f}%", ha="center",
                fontsize=9, fontweight="bold")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylim(0, 105)
    ax.set_ylabel("NIS 门限通过率 (%)")
    ax.set_title("(d) 各目标跟踪一致性", fontsize=10.5)
    ax.axhline(100 * passed.mean(), color=C["neutral"], ls="--", lw=1.0)
    ax.text(0.99, 100 * passed.mean() + 1.5, f"整体 {100*passed.mean():.1f}%",
            transform=ax.get_yaxis_transform(), ha="right", va="bottom",
            fontsize=8.3, color=C["neutral"])
    ax.grid(axis="x", visible=False)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 14
def fig14(D):
    rad = D["ir_int"]["ir_rad"]
    geom = D["ir_int"]["ir_geom"]
    cand = D["ir_int"]["ir_cand"]
    obs = D["ir_ext"]["ir_obs"]
    trk = D["ir_int"]["ir_trk"]

    snr = rad["snr"].to_numpy(float)
    snr_db = db10(snr)
    rng = rad["rng"].to_numpy(float) / 1e7            # 10^4 km
    ok_fov = rad["admit"].astype(str).eq("通过").to_numpy()
    thr = float(rad["thr"].median())
    thr_db = float(db10(thr))

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.2))
    title_block(fig, "图 14  红外辐射检测与链路通过率",
                f"辐射计算 {len(rad)} 次，SNR {np.nanmin(snr_db):.1f}–{np.nanmax(snr_db):.1f} dB"
                f"（门限 {thr_db:.1f} dB）；通过视场门控 {int(ok_fov.sum())} 次"
                f"（{100 * ok_fov.mean():.2f}%）")

    ax = axes[0, 0]
    ax.scatter(rng[~ok_fov], snr_db[~ok_fov], s=3, color=C["neutral"], alpha=0.18,
               linewidths=0, label=f"视场排除（{int((~ok_fov).sum())}）")
    cx, med, lo, hi = _bin_stat(rng, snr_db, 42)
    ax.fill_between(cx, lo, hi, color=C["accent1"], alpha=0.18, lw=0,
                    label="10–90% 分位")
    ax.plot(cx, med, color="#123f63", lw=1.5, label="分箱中位")
    ax.scatter(rng[ok_fov], snr_db[ok_fov], s=26, color=C["accent3"], alpha=0.9,
               edgecolors="white", linewidths=0.4,
               label=f"视场通过（{int(ok_fov.sum())}）")
    ax.axhline(thr_db, color=C["warn"], ls="--", lw=1.1,
               label=f"SNR 门限 {thr_db:.1f} dB")
    ax.set_xlabel("卫星–目标斜距 ($10^4$ km)")
    ax.set_ylabel("接收信噪比 (dB)")
    ax.set_title("(a) 信噪比–距离关系", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=7.8, markerscale=1.6)
    style_axes(ax)

    ax = axes[0, 1]
    stages = ["宽视场几何评估", "视场门控通过", "SNR 门限通过",
              "调度候选", "窄视场观测", "跟踪滤波输出"]
    vals = [len(geom[geom["stage"].astype(str).eq("WFOV")]), int(ok_fov.sum()),
            int(rad["snr_res"].astype(str).eq("通过").sum()), len(cand),
            len(obs), len(trk)]
    vals_db = db10(np.asarray(vals, dtype=float))
    colors = ["#9fb3c8", C["accent1"], C["accent4"], C["accent3"],
              C["accent2"], "#123f63"]
    y = np.arange(len(stages))[::-1]
    ax.barh(y, vals_db, color=colors, alpha=0.9, height=0.6)
    ax.set_yticks(y)
    ax.set_yticklabels(stages)
    ax.set_xlabel("记录数 (dB，相对 1 条)")
    ax.set_title("(b) 红外链路各阶段记录量", fontsize=10.5)
    for yy, v, d in zip(y, vals, vals_db):
        ax.text(d + 0.4, yy, f"{v:,}", va="center", fontsize=8.5)
    ax.set_xlim(0, float(np.nanmax(vals_db)) * 1.18)
    ax.grid(axis="y", visible=False)
    style_axes(ax)

    ax = axes[1, 0]
    ax.hist(snr_db, bins=60, color=C["accent1"], alpha=0.8,
            edgecolor="white", linewidth=0.3)
    ax.axvline(thr_db, color=C["warn"], ls="--", lw=1.1)
    ax.set_xlabel("信噪比 (dB)")
    ax.set_ylabel("样本数")
    ax.set_title(f"(c) 信噪比分布（{snr.min()/thr:.1f}–{snr.max()/thr:.1f} 倍门限）",
                 fontsize=10.5)
    ax.annotate(f"中位 {float(db10(np.median(snr))):.1f} dB，最小 {np.nanmin(snr_db):.1f} dB\n"
                f"最低 {snr.min()/thr:.1f} 倍门限",
                xy=(0.04, 0.94), xycoords="axes fraction", fontsize=8.5, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 1]
    prx = rad["p_rx"].to_numpy(float)
    pn_db = db10(prx / np.clip(rad["tau"].to_numpy(float), 1e-3, None))
    cx, med, lo, hi = _bin_stat(rng, pn_db, 36)
    ax.fill_between(cx, lo, hi, color=C["accent4"], alpha=0.18, lw=0,
                    label="10–90% 分位")
    ax.plot(cx, med, "o-", ms=2.6, lw=1.2, color=C["accent4"], label="分箱中位")
    # 幂律指数仍按 log–log 拟合求，再换算到 dB 坐标上画直线（−20 dB/十倍程 ↔ R⁻²）。
    # 只对正距离、有限中位的箱做拟合；样本不足时不出拟合线，避免整张图失败。
    ok = (cx > 0) & np.isfinite(med)
    lg = None
    if ok.sum() >= 2:
        lg = np.polyfit(np.log10(cx[ok]), med[ok], 1)
        xs = np.linspace(float(cx[ok].min()), float(cx[ok].max()), 80)
        ax.plot(xs, lg[0] * np.log10(xs) + lg[1], color=C["warn"], lw=1.4,
                label=f"拟合 {lg[0]:.1f} dB/十倍程")
    ax.set_xlabel("斜距 ($10^4$ km)")
    ax.set_ylabel("归一化接收功率 $P_r/\\tau$ (dBW)")
    ax.set_title("(d) 接收功率随距离衰减", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=8)
    if lg is None:
        fit = "拟合样本不足，未出拟合线"
    else:
        fit = f"$P_r/\\tau$ ∝ $R^{{{lg[0]/10.0:.2f}}}$（理论 $R^{{-2}}$）"
    ax.annotate(f"{fit}\n中位 {float(db10(np.median(prx))):.1f} dBW（原始 $P_r$ 中位）",
                xy=(0.04, 0.06), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 15
def fig15(D):
    pe = D["sat"]["pos_err"].sort_values("t")
    uk = D["sat"]["ukf"].sort_values("t")
    imp = D["sat"]["impact"]
    lau = D["sat"]["launch"]

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    i_lon, i_lat = float(imp["lon"].median()), float(imp["lat"].median())
    l_lon, l_lat = float(lau["lon"].median()), float(lau["lat"].median())
    title_block(fig, "图 15  卫星导航误差、UKF 收敛与落点/发射点估计",
                f"导航位置误差 0–{pe['err'].max():.2f} m（中位 {pe['err'].median():.2f} m），"
                f"UKF 协方差迹中位 {float(db10(uk['covtr'].median())):.0f} dB；"
                f"落点中位 ({i_lon:.1f}°, {i_lat:.1f}°)、"
                f"发射点中位 ({l_lon:.1f}°, {l_lat:.1f}°)")

    ax = axes[0, 0]
    for k, (sat, g) in enumerate(pe.groupby("sat")):
        ax.plot(g["t"], g["err"], lw=0.45, color=CYCLE[k], alpha=0.65,
                label=f"卫星 {int(sat)}")
    ax.set_xlim(0, 7000)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("位置误差 (m)")
    ax.set_title("(a) 导航解算位置误差", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=7.5, ncol=2)
    ax.annotate(f"均值 {pe['err'].mean():.2f} m\n95% 分位 "
                f"{np.percentile(pe['err'], 95):.2f} m",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[0, 1]
    uk_t = uk["t"].to_numpy(float)
    uk_db = db10(uk["covtr"].to_numpy(float))
    ax.plot(uk_t, uk_db, lw=1.0, color=C["accent4"])
    ax.set_xlim(float(uk_t.min()), float(uk_t.max()))
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("协方差迹 tr(P) (dB，相对 1 m²)")
    ax.set_title("(b) UKF 协方差收敛过程", fontsize=10.5)
    # 同一时刻可能有多条记录，取包络（逐时刻最大值）判定末段抬升；
    # 阈值取全段中位 +10 dB（即 10 倍），不写死时间窗。
    env = db10(uk.groupby("t")["covtr"].max().sort_index().to_numpy(float))
    env_t = np.sort(uk["t"].unique()).astype(float)
    med_db = float(np.nanmedian(uk_db))
    elevated = np.isfinite(env) & (env > med_db + 10.0)
    calm = np.where(~elevated)[0]
    if len(env_t) == len(elevated) and calm.size and calm[-1] + 1 < len(env_t):
        start = float(env_t[calm[-1] + 1])
        ax.axvspan(start, float(uk_t.max()), color=C["warn"], alpha=0.08, lw=0)
        ax.text(0.98, 0.16, f"末段协方差抬升\n（包络 > 中位 +10 dB，"
                f"{start:.0f} s 起）", transform=ax.transAxes, fontsize=7.8,
                color=C["warn"], va="bottom", ha="right")
    ax.annotate(f"初始 {uk_db[0]:.0f} dB\n稳态中位 {med_db:.0f} dB\n"
                f"末条记录 {uk_db[-1]:.0f} dB",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb",
                                       ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1, 0]
    for k, (key, g) in enumerate(imp.groupby("key")):
        ax.scatter(g["lon"], g["lat"], s=9, color=CYCLE[k], alpha=0.5,
                   linewidths=0, label=f"目标 {int(key)}（{len(g)} 点）")
        ax.plot(g["lon"].iloc[-1], g["lat"].iloc[-1], marker="*", ms=13,
                color=CYCLE[k], mec="white", mew=0.6, ls="none")
    # 以经度中位为心、±60° 内的点为主群；跨日界线的离群点不参与定窗
    lon_i = imp["lon"].to_numpy(float)
    lat_i = imp["lat"].to_numpy(float)
    dlon = np.abs(((lon_i - float(np.median(lon_i)) + 180.0) % 360.0) - 180.0)
    keep = dlon <= 60.0
    out = int((~keep).sum())
    xpad = 0.05 * max(np.ptp(lon_i[keep]), 1.0)
    ypad = 0.05 * max(np.ptp(lat_i[keep]), 1.0)
    ax.set_xlim(float(lon_i[keep].min()) - xpad, float(lon_i[keep].max()) + xpad)
    ax.set_ylim(float(lat_i[keep].min()) - ypad, float(lat_i[keep].max()) + ypad)
    ax.set_xlabel("经度 (°)")
    ax.set_ylabel("纬度 (°)")
    ax.set_title("(c) 落点估计散布（★ 为末次估计）", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=8, markerscale=2.0)
    note = f"落点半径 1σ {imp['r1'].median()/1e3:.1f} km（中位）"
    if out:
        note += f"\n另有 {out} 个偏离主群 > 60° 的点未显示"
    ax.annotate(note,
                xy=(0.97, 0.95), xycoords="axes fraction", fontsize=8.3,
                va="top", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax, minor=False)

    ax = axes[1, 1]
    for k, (key, g) in enumerate(lau.groupby("key")):
        ax.scatter(g["lon"], g["lat"], s=8, color=CYCLE[k], alpha=0.45,
                   linewidths=0, label=f"目标 {int(key)}（{len(g)} 点）")
        ax.plot(g["lon"].iloc[-1], g["lat"].iloc[-1], marker="*", ms=13,
                color=CYCLE[k], mec="white", mew=0.6, ls="none")
        # 末次估计的误差椭圆：ea/eb 为 1σ 半轴（m）
        e = g.iloc[-1]
        if np.isfinite(e["ea"]) and np.isfinite(e["eb"]) and e["ea"] < 2e5:
            wl = 2 * e["eb"] / (111320.0 * max(np.cos(np.radians(e["lat"])), 0.1))
            ht = 2 * e["ea"] / 111320.0
            ax.add_patch(Ellipse((e["lon"], e["lat"]), wl, ht, fill=False,
                                 ec=CYCLE[k], lw=1.1, ls="--"))
    lon_l = lau["lon"].to_numpy(float)
    lat_l = lau["lat"].to_numpy(float)
    xpad = 0.05 * max(np.ptp(lon_l), 1.0)
    ypad = 0.05 * max(np.ptp(lat_l), 1.0)
    ax.set_xlim(float(lon_l.min()) - xpad, float(lon_l.max()) + xpad)
    ax.set_ylim(float(lat_l.min()) - ypad, float(lat_l.max()) + ypad)
    ax.set_xlabel("经度 (°)")
    ax.set_ylabel("纬度 (°)")
    ax.set_title("(d) 发射点估计散布与误差椭圆", fontsize=10.5)
    ax.legend(loc="lower left", fontsize=8, markerscale=2.0)
    ax.annotate(f"1σ 误差 {lau['ea'].median()/1e3:.0f}×{lau['eb'].median()/1e3:.0f} km（中位）",
                xy=(0.97, 0.95), xycoords="axes fraction", fontsize=8.3,
                va="top", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax, minor=False)
    return fig


FIGURES = [
    ("12", "fig12_ir_coverage", "图 12 红外覆盖与可用性", fig12),
    ("13", "fig13_ir_tracking", "图 13 红外跟踪与 NIS", fig13),
    ("14", "fig14_ir_radiometry", "图 14 红外辐射检测", fig14),
    ("15", "fig15_sat_nav_impact", "图 15 卫星导航与落点", fig15),
]
