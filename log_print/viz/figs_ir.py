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
from matplotlib.lines import Line2D

from .common import (C, CYCLE, W_IR, by_target, color_map, db10, marker_map,
                     needs, style_axes, target_title, title_block)


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


def _panel_los(ax, det):
    """目标视线角分布。日志里方位/俯仰是 rad，图上统一按 ° 画。"""
    for k, (sat, g) in enumerate(det.groupby("sat")):
        ax.scatter(np.degrees(g["az"].to_numpy(float)),
                   np.degrees(g["el"].to_numpy(float)), s=4, color=CYCLE[k],
                   alpha=0.5, linewidths=0, label=f"卫星 {int(sat)}")
    ax.set_xlabel("方位视线角 (°)")
    ax.set_ylabel("俯仰视线角 (°)")
    ax.legend(loc="lower left", fontsize=7.5, markerscale=1.8)
    ax.annotate(f"方位 {np.degrees(det['az'].min()):.2f}…{np.degrees(det['az'].max()):.2f}°\n"
                f"俯仰 {np.degrees(det['el'].min()):.2f}…{np.degrees(det['el'].max()):.2f}°",
                xy=(0.97, 0.95), xycoords="axes fraction", fontsize=8.3,
                va="top", ha="right",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)


def _panel_maxrange(ax, mr):
    """各卫星最大作用距离分布。"""
    mr = mr[mr["rmax"].notna()]
    sats = sorted(int(v) for v in mr["sat"].unique())
    data = [mr[mr["sat"] == s]["rmax"].to_numpy(float) / 1e7 for s in sats]
    parts = ax.violinplot(data, showmedians=True, showextrema=False, widths=0.75)
    for k, body in enumerate(parts["bodies"]):
        body.set_facecolor(CYCLE[k])
        body.set_alpha(0.7)
        body.set_edgecolor("none")
    parts["cmedians"].set_color("#1b2733")
    ax.set_xticks(range(1, len(data) + 1))
    ax.set_xticklabels([f"卫星 {s}" for s in sats])
    ax.set_ylabel("最大作用距离 ($10^7$ m)")
    ax.annotate(f"整体 {mr['rmax'].min()/1e7:.3f}–{mr['rmax'].max()/1e7:.3f}×10$^7$ m\n"
                f"相当于 {mr['rmax'].min()/1e3:.0f}–{mr['rmax'].max()/1e3:.0f} km",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.grid(axis="x", visible=False)
    style_axes(ax)


def _fig12_partial(det, mr):
    """本批日志没有宽视场覆盖消息：只画视线角与最大作用距离，并注明缺项。"""
    fig, axes = plt.subplots(1, 2, figsize=(13.4, 5.2))
    title_block(fig, "图 12  红外覆盖与可用性（本批缺宽视场覆盖记录）",
                f"未找到「宽视场扫描探测功能测试」消息，覆盖中心与覆盖时段从略；"
                f"保留目标视线角 {len(det)} 条、最大作用距离 {len(mr)} 条")
    _panel_los(axes[0], det)
    axes[0].set_title("(a) 目标视线角分布", fontsize=10.5)
    _panel_maxrange(axes[1], mr)
    axes[1].set_title("(b) 各卫星最大作用距离分布", fontsize=10.5)
    return fig


# ---------------------------------------------------------------- 图 12
@needs(("sat", "ir_detect", "多传感器目标跟踪功能测试"),
       ("sat", "maxrange", "最大探测距离计算功能测试"),
       title="图 12 红外覆盖与可用性")
def fig12(D):
    sat = D["sat"]
    det = sat["ir_detect"].sort_values("t")
    mr = sat["maxrange"].sort_values("t")
    wf = sat.get("wfov")

    ok = np.zeros(0, dtype=bool)
    if wf is not None and len(wf):
        wf = wf.sort_values("t")
        ok, lat, lon = _valid_centers(wf)
    if wf is None or len(wf) == 0 or not ok.any():
        # 日志里没有宽视场覆盖消息（或全部无效）时不要把整张图判为失败。
        return _fig12_partial(det, mr)

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

    _panel_los(axes[1, 0], det)
    axes[1, 0].set_title("(c) 目标视线角分布", fontsize=10.5)

    _panel_maxrange(axes[1, 1], mr)
    axes[1, 1].set_title("(d) 各卫星最大作用距离分布", fontsize=10.5)
    return fig


# ---------------------------------------------------------------- 图 13
@needs(("ir_int", "ir_trk", "红外量测与跟踪滤波交互接口测试"),
       title="图 13 红外跟踪与 NIS")
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
@needs(("ir_int", "ir_rad", "红外辐射噪声与检测判决交互接口测试"),
       ("ir_int", "ir_geom", "红外场景几何与视场门控交互接口测试"),
       ("ir_int", "ir_cand", "红外宽场候选与窄场调度交互接口测试"),
       ("ir_ext", "ir_obs", "红外观测与航迹输出接口测试"),
       ("ir_int", "ir_trk", "红外量测与跟踪滤波交互接口测试"),
       title="图 14 红外辐射检测")
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

    sats = sorted(int(v) for v in rad["sat"].dropna().unique())
    tids = sorted(int(v) for v in rad["tid"].dropna().unique())
    scmap = color_map(sats)
    tmark = marker_map(tids)
    satv = rad["sat"].to_numpy(float)
    tidv = rad["tid"].to_numpy(float)

    def _xleg(ax, handles, loc, title):
        return ax.legend(handles=handles, loc=loc, fontsize=6.6, title=title,
                         title_fontsize=7.0, handletextpad=0.3, labelspacing=0.28)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 14  红外辐射检测与链路通过率（颜色＝卫星，标记＝目标）",
                f"辐射计算 {len(rad)} 次（卫星 {'、'.join(str(s) for s in sats)}；"
                f"目标 {'、'.join(str(t) for t in tids)}），SNR "
                f"{np.nanmin(snr_db):.1f}–{np.nanmax(snr_db):.1f} dB（门限 {thr_db:.1f} dB）；"
                f"通过视场门控 {int(ok_fov.sum())} 次（{100 * ok_fov.mean():.2f}%）")

    ax = axes[0, 0]
    for s in sats:
        for t in tids:
            m = (satv == s) & (tidv == t)
            if not m.any():
                continue
            on = m & ok_fov
            if on.any():
                ax.scatter(rng[on], snr_db[on], s=9, marker=tmark[t],
                           color=scmap[s], alpha=0.32, linewidths=0)
            off = m & ~ok_fov
            if off.any():
                ax.scatter(rng[off], snr_db[off], s=8, marker=tmark[t],
                           facecolors="none", edgecolors=scmap[s], alpha=0.3,
                           linewidths=0.4)
    ax.axhline(thr_db, color=C["warn"], ls="--", lw=1.1,
               label=f"SNR 门限 {thr_db:.1f} dB")
    ax.set_xlabel("卫星–目标斜距 ($10^4$ km)")
    ax.set_ylabel("接收信噪比 (dB)")
    ax.set_title("(a) 信噪比–距离关系（实心＝通过视场，空心＝排除）", fontsize=10.5)
    handles = [Line2D([], [], ls="none", marker="o", ms=5, color=scmap[s],
                      label=f"卫星 {s}") for s in sats]
    handles += [Line2D([], [], ls="none", marker=tmark[t], ms=5,
                       color=C["neutral"], label=f"目标 {t}") for t in tids]
    ax.legend(handles=handles, loc="lower center", ncol=4, fontsize=6.5,
              handletextpad=0.3, columnspacing=0.9, labelspacing=0.3)
    style_axes(ax)

    ax = axes[0, 1]
    stages = ["宽视场几何评估", "视场门控通过", "SNR 门限通过",
              "调度候选", "窄视场观测", "跟踪滤波输出"]
    wfov_ok = geom["stage"].astype(str).eq("WFOV").to_numpy()
    snr_ok = rad["snr_res"].astype(str).eq("通过").to_numpy()
    gsat = geom["sat"].to_numpy(float)
    csat = cand["sat"].to_numpy(float)
    osat = obs["sat"].to_numpy(float)
    ksat = trk["sat"].to_numpy(float)
    sat_stage = []
    for s in sats:
        g = gsat == s
        r = satv == s
        sat_stage.append([int((g & wfov_ok).sum()), int((r & ok_fov).sum()),
                          int((r & snr_ok).sum()), int((csat == s).sum()),
                          int((osat == s).sum()), int((ksat == s).sum())])
    n = max(len(sats), 1)
    bw = 0.8 / n
    vmax = 1.0
    for j, s in enumerate(sats):
        off = (j - (n - 1) / 2.0) * bw
        v = np.asarray(sat_stage[j], dtype=float)
        h = db10(v)
        vmax = max(vmax, float(np.nanmax(h)))
        ax.bar(np.arange(len(stages)) + off, np.nan_to_num(h, nan=0.0),
               width=bw * 0.9, color=scmap[s], alpha=0.9, label=f"卫星 {s}")
        for x, cnt, hv in zip(np.arange(len(stages)) + off, v, h):
            if cnt > 0 and np.isfinite(hv):
                ax.text(x, hv + 0.5, f"{int(cnt):,}", ha="center", va="bottom",
                        fontsize=5.6, rotation=90, color=scmap[s])
    ax.set_xticks(range(len(stages)))
    ax.set_xticklabels(stages, fontsize=7.6, rotation=22, ha="right")
    ax.set_xlabel("处理阶段")
    ax.set_ylabel("记录数 (dB，相对 1 条)")
    ax.set_title("(b) 各阶段记录量（按卫星分组）", fontsize=10.5)
    ax.set_xlim(-0.6, len(stages) - 0.4)
    ax.set_ylim(0, vmax * 1.35 + 0.5)
    ax.legend(loc="upper right", fontsize=6.8, ncol=2)
    ax.grid(axis="x", visible=False)
    style_axes(ax)

    ax = axes[1, 0]
    data = [snr_db[(satv == s) & np.isfinite(snr_db)] for s in sats]
    bp = ax.boxplot(data, positions=np.arange(1, len(sats) + 1), widths=0.55,
                    showfliers=False, patch_artist=True)
    for patch, s in zip(bp["boxes"], sats):
        patch.set_facecolor(scmap[s])
        patch.set_alpha(0.24)
        patch.set_edgecolor(scmap[s])
    for med in bp["medians"]:
        med.set_color("#1b2733")
        med.set_linewidth(1.2)
    for j, s in enumerate(sats):
        present = [t for t in tids if int(((satv == s) & (tidv == t)).sum()) >= 30]
        for k, t in enumerate(present):
            m = (satv == s) & (tidv == t) & np.isfinite(snr_db)
            off = (k - (len(present) - 1) / 2.0) * 0.13
            ax.plot([j + 1 + off], [float(np.median(snr_db[m]))], ls="none",
                    marker=tmark[t], ms=5, color=scmap[s], mec="white", mew=0.5)
    ax.axhline(thr_db, color=C["warn"], ls="--", lw=1.1)
    counts = [int((satv == s).sum()) for s in sats]
    ax.set_xticks(range(1, len(sats) + 1))
    ax.set_xticklabels([f"卫星 {s}\n({c:,})" for s, c in zip(sats, counts)],
                       fontsize=7.8)
    ax.set_ylabel("信噪比 (dB)")
    ax.set_title(f"(c) 信噪比分布（箱体＝卫星，标记＝目标中位；{snr.min()/thr:.1f}–"
                 f"{snr.max()/thr:.1f} 倍门限）", fontsize=9.6)
    ax.annotate(f"全体中位 {float(db10(np.median(snr))):.1f} dB，"
                f"最小 {np.nanmin(snr_db):.1f} dB",
                xy=(0.03, 0.95), xycoords="axes fraction", fontsize=8.0, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    _xleg(ax, [Line2D([], [], ls="none", marker=tmark[t], ms=5,
                      color=C["neutral"], label=f"目标 {t}") for t in tids],
          "lower right", "目标（标记）")
    style_axes(ax)

    ax = axes[1, 1]
    prx = rad["p_rx"].to_numpy(float)
    pn_db = db10(prx / np.clip(rad["tau"].to_numpy(float), 1e-3, None))
    step = max(len(rad) // 6000, 1)
    base_idx = np.arange(0, len(rad), step)
    for s in sats:
        for t in tids:
            m = (satv == s) & (tidv == t)
            if not m.any():
                continue
            sub = base_idx[(satv[base_idx] == s) & (tidv[base_idx] == t)]
            if sub.size:
                ax.scatter(rng[sub], pn_db[sub], s=7, marker=tmark[t],
                           color=scmap[s], alpha=0.3, linewidths=0)
    fits = []
    for s in sats:
        m = satv == s
        cx, med, lo, hi = _bin_stat(rng[m], pn_db[m], 36)
        ok = (cx > 0) & np.isfinite(med)
        if ok.sum() < 2:
            fits.append(f"卫星 {s}：分箱不足")
            continue
        lg = np.polyfit(np.log10(cx[ok]), med[ok], 1)
        xs = np.linspace(float(cx[ok].min()), float(cx[ok].max()), 80)
        ax.plot(xs, lg[0] * np.log10(xs) + lg[1], color=scmap[s], lw=1.4,
                label=f"卫星 {s} 拟合 {lg[0]:.1f} dB/十倍程")
        fits.append(f"卫星 {s}：{lg[0]:.1f} dB/十倍程（{int(m.sum()):,} 条）")
    ax.set_xlabel("斜距 ($10^4$ km)")
    ax.set_ylabel("归一化接收功率 $P_r/\\tau$ (dBW)")
    ax.set_title("(d) 接收功率随距离衰减（按卫星拟合）", fontsize=10.5)
    ax.annotate("理论 $R^{-2}$ ＝ −20 dB/十倍程\n" + "\n".join(fits),
                xy=(0.03, 0.97), xycoords="axes fraction", fontsize=6.9, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    _xleg(ax, [Line2D([], [], ls="-", lw=1.4, color=scmap[s], label=f"卫星 {s}")
               for s in sats], "upper right", "卫星（颜色）")
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 15
@needs(("sat", "pos_err", "卫星自身定位误差功能测试"),
       ("sat", "ukf", "UKF滤波"),
       ("sat", "impact", "落点预报与信息分发功能测试"),
       ("sat", "launch", "发射点预测功能测试"),
       title="图 15 卫星导航与落点")
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
