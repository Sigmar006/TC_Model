# -*- coding: utf-8 -*-
"""图 4–11、16：雷达检测、跟踪、干扰与资源调度。"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from matplotlib.patches import Rectangle

from . import extract
from .common import (C, CYCLE, W_RADAR, enu_to_lla, style_axes,
                     title_block, fnum)


# ---------------------------------------------------------------- 公共
def _enu(df, c0="e0", c1="e1", c2="e2"):
    e = df[c0].to_numpy(dtype=float)
    n = df[c1].to_numpy(dtype=float)
    u = df[c2].to_numpy(dtype=float)
    rng = np.sqrt(e * e + n * n + u * u)
    _, _, h = enu_to_lla(e, n, u)
    return rng, h


def _runs(t, ok):
    """把布尔序列切成 [start, end] 连续段。"""
    out = []
    start = None
    for i, v in enumerate(ok):
        if v and start is None:
            start = t[i]
        elif not v and start is not None:
            out.append((start, t[i - 1]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def _scatter_legend(ax, **kw):
    h, l = ax.get_legend_handles_labels()
    if h:
        ax.legend(h, l, **kw)


# ---------------------------------------------------------------- 图 4
def fig04(D):
    det = D["radar_int"]["det"].sort_values("t")
    plt_ = D["radar_ext"]["plot"].sort_values("t")
    cfar = D["radar_output"]["cfar"].sort_values("t")
    thr = float(det["thr"].median())

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 4  检测信噪比、检测概率与门限",
                f"检测记录 {len(det)} 条（{det['t'].min():.0f}–{det['t'].max():.0f} s），"
                f"点迹量测 {len(plt_)} 条；CFAR 门限恒为 {thr:.2f} dB")

    ax = axes[0]
    t = det["t"].to_numpy(float)
    snr = det["snr_out"].to_numpy(float)
    ax.plot(t, snr, color=C["accent1"], lw=1.2, label="检测前 SNR")
    ax.axhline(thr, color=C["warn"], ls="--", lw=1.2, label=f"门限 {thr:.2f} dB")
    below = snr < thr
    ax.fill_between(t, snr, thr, where=below, color=C["warn"], alpha=0.16,
                    lw=0, label=f"低于门限 {int(below.sum())} 次")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("SNR (dB)")
    ax.set_title("(a) 单周期检测 SNR 与门限", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)

    ax = axes[1]
    x = plt_["snr"].to_numpy(float)
    y = plt_["pd"].to_numpy(float)
    sc = ax.scatter(x, y, c=plt_["rng"].to_numpy(float) / 1e3, s=7,
                    cmap="viridis", alpha=0.65, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("斜距 (km)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.axvline(thr, color=C["warn"], ls="--", lw=1.0)
    ax.text(thr + 0.15, 0.03, "门限", color=C["warn"], fontsize=8, rotation=90,
            va="bottom")
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("单次检测概率 $P_d$")
    ax.set_title("(b) 检测概率随 SNR 分布", fontsize=10.5)
    ax.set_ylim(-0.02, 1.02)
    style_axes(ax)

    ax = axes[2]
    ax.plot(cfar["t"], cfar["r90"] / 1e3, color=C["accent3"], lw=1.4)
    ax.fill_between(cfar["t"], cfar["r90"] / 1e3, color=C["accent3"], alpha=0.12)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("$P_d=90\\%$ 作用距离 (km)")
    ax.set_title("(c) CFAR 90% 检测距离", fontsize=10.5)
    ax.set_ylim(cfar["r90"].min() / 1e3 * 0.85, cfar["r90"].max() / 1e3 * 1.1)
    ax.annotate(f"R$_{{90}}$ = {cfar['r90'].min()/1e3:.0f}–{cfar['r90'].max()/1e3:.0f} km\n"
                f"RCS 恒为 {cfar['rcs90'].median():.2f} m²",
                xy=(0.04, 0.88), xycoords="axes fraction", fontsize=8.5,
                va="top", bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb",
                                    ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 5
def fig05(D):
    p = D["radar_ext"]["plot"]
    p = p[p["tid"] == 109].sort_values("t")
    rng = p["rng"].to_numpy(float) / 1e3
    pd_ = p["pd"].to_numpy(float)
    snr = p["snr"].to_numpy(float)
    t = p["t"].to_numpy(float)

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 5  检测概率–距离包络",
                f"目标 109 点迹 {len(p)} 条，斜距 {rng.min():.0f}–{rng.max():.0f} km，"
                f"RCS 恒为 0.1 m²；同一距离上 $P_d$ 的离散来自干扰功率变化")

    ax = axes[0]
    sc = ax.scatter(rng, pd_, c=t, s=8, cmap="viridis", alpha=0.75, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("仿真时间 (s)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("单次检测概率 $P_d$")
    ax.set_title("(a) $P_d$–$R$ 散点（颜色为时间）", fontsize=10.5)
    ax.set_ylim(-0.02, 1.02)
    ax.invert_xaxis()
    style_axes(ax)

    ax = axes[1]
    bins = np.linspace(rng.min(), rng.max(), 16)
    idx = np.digitize(rng, bins)
    bx, bm, blo, bhi = [], [], [], []
    for k in range(1, len(bins)):
        m = idx == k
        if m.sum() >= 20:
            bx.append(0.5 * (bins[k - 1] + bins[k]))
            bm.append(np.median(pd_[m]))
            blo.append(pd_[m].min())
            bhi.append(pd_[m].max())
    ax.fill_between(bx, blo, bhi, color=C["accent2"], alpha=0.16, lw=0,
                    label="同距离 $P_d$ 上下界")
    ax.plot(bx, bm, "o-", color=C["accent2"], ms=3.5, lw=1.4, label="中位包络")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("$P_d$")
    ax.set_title("(b) 分距离段 $P_d$ 包络", fontsize=10.5)
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlim(rng.min() - 40, rng.max() + 40)
    ax.invert_xaxis()
    ax.legend(loc="lower left", fontsize=8)
    style_axes(ax)

    ax = axes[2]
    sc = ax.scatter(snr, pd_, c=rng, s=8, cmap="viridis", alpha=0.75,
                    linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("斜距 (km)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("$P_d$")
    ax.set_title("(c) $P_d$–SNR 检测曲线", fontsize=10.5)
    ax.set_ylim(-0.02, 1.02)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 6
def fig06(D):
    jp = D["radar_output"]["jam_power"].sort_values("t")
    echo = D["radar_output"]["echo"].sort_values("t")
    jg = D["radar_output"]["jam_gain"].sort_values("t")

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 6  干扰功率与抗干扰抑制",
                f"干扰源 {jp['src'].nunique()} 部（目标 {sorted(jp['src'].unique().astype(int).tolist())}），"
                f"接收干扰功率 {jp['power'].min():.2e}–{jp['power'].max():.2e} W")

    ax = axes[0]
    for k, (src, g) in enumerate(jp.groupby("src")):
        ax.plot(g["t"], g["power"], lw=0.8, color=CYCLE[k],
                label=f"干扰源 {int(src)}")
    ax.set_yscale("log")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("接收干扰功率 (W)")
    ax.set_title("(a) 各干扰源接收功率", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)

    ax = axes[1]
    js = echo["p_jam"] / echo["p_echo"]
    jn = echo["p_jam"] / echo["p_noise"]
    ax.plot(echo["t"], js, lw=1.1, color=C["warn"], label="J/S")
    ax.plot(echo["t"], jn, lw=1.1, color=C["accent4"], label="J/N")
    ax.set_yscale("log")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率比")
    ax.set_title("(b) 干信比与干噪比", fontsize=10.5)
    ax.annotate(f"J/S 中位 {np.median(js):.1f} 倍（{10*np.log10(np.median(js)):.1f} dB）\n"
                f"J/N 中位 {np.median(jn):.1f} 倍（{10*np.log10(np.median(jn)):.1f} dB）",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="center right", fontsize=8)
    style_axes(ax)

    ax = axes[2]
    suppr = jg["suppr"].to_numpy(float)
    ax.scatter(jg["p_mti"], suppr, s=9, color=C["accent3"], alpha=0.55,
               linewidths=0)
    ax.set_xscale("log")
    ax.axhline(np.median(suppr), color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {np.median(suppr):.1f} dB")
    ax.set_xlabel("MTI 后残余干扰功率 (W)")
    ax.set_ylabel("干扰抑制比 (dB)")
    ax.set_title("(c) MTI 干扰抑制效果", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 7
def fig07(D):
    ji = D["radar_ext"]["jam_in"].sort_values("t")
    fig = plt.figure(figsize=(13.2, 7.4))
    title_block(fig, "图 7  干扰源参数与空间几何",
                f"带限噪声压制干扰，中心频率 {ji['fc'].median()/1e9:.2f} GHz，"
                f"带宽 {ji['bw'].median()/1e6:.0f} MHz，发射功率 {ji['ptx'].median():.1f} dBW")

    gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.0])
    ax = fig.add_subplot(gs[0, :2])
    for k, (src, g) in enumerate(ji.groupby("src")):
        e = g["p0"].to_numpy(float)
        n = g["p1"].to_numpy(float)
        u = g["p2"].to_numpy(float)
        r = np.sqrt(e * e + n * n + u * u) / 1e3
        ax.plot(g["t"], r, lw=1.3, color=CYCLE[k], label=f"干扰源 {int(src)}")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("干扰机–雷达斜距 (km)")
    ax.set_title("(a) 干扰机接近过程", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = fig.add_subplot(gs[0, 2])
    for k, (src, g) in enumerate(ji.groupby("src")):
        e = g["p0"].to_numpy(float) / 1e3
        n = g["p1"].to_numpy(float) / 1e3
        ax.plot(e, n, lw=1.1, color=CYCLE[k], label=f"{int(src)}")
        ax.plot(e[0], n[0], "o", ms=4, color=CYCLE[k])
        ax.annotate("起点", (e[0], n[0]), textcoords="offset points",
                    xytext=(4, 4), fontsize=7.5, color=CYCLE[k])
    ax.plot(0, 0, marker="^", ms=8, color=C["warn"], ls="none", label="雷达")
    ax.set_xlabel("东向 (km)")
    ax.set_ylabel("北向 (km)")
    ax.set_title("(b) 干扰机地心 ENU 航迹", fontsize=10.5)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=7.5)
    style_axes(ax)

    ax = fig.add_subplot(gs[1, 0])
    g0 = ji.groupby("src")
    t_on = g0["t0"].agg(["min", "max"])
    for k, (src, row) in enumerate(t_on.iterrows()):
        ax.add_patch(Rectangle((row["min"], k - 0.28), row["max"] - row["min"],
                               0.56, color=CYCLE[k], alpha=0.75))
        ax.text(row["min"] + 40, k, f"{int(src)} 号干扰机", va="center",
                fontsize=8.5, color="white")
    ax.set_yticks(range(len(t_on)))
    ax.set_yticklabels([f"{int(s)}" for s in t_on.index])
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("干扰源编号")
    ax.set_title("(c) 干扰开机时段", fontsize=10.5)
    ax.set_ylim(-0.6, len(t_on) - 0.4)
    ax.set_xlim(float(t_on["min"].min()) - 60,
                float(t_on["max"].max()) + 60)
    style_axes(ax)

    ax = fig.add_subplot(gs[1, 1])
    jp = D["radar_output"]["jam_power"]
    for k, (src, g) in enumerate(jp.groupby("src")):
        ax.plot(g["t"], g["pden"], lw=1.0, color=CYCLE[k], label=f"{int(src)}")
    ax.set_yscale("log")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率谱密度 (W/Hz)")
    ax.set_title("(d) 到达功率谱密度", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)

    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    rows = [
        ("干扰样式", str(ji["wave"].iloc[0])),
        ("中心频率", f"{ji['fc'].median()/1e9:.2f} GHz"),
        ("带宽", f"{ji['bw'].median()/1e6:.0f} MHz"),
        ("发射功率", f"{ji['ptx'].median():.2f} dBW"),
        ("天线峰值增益", f"{ji['gpeak'].median():.1f} dBi"),
        ("极化编号", f"{ji['pol'].median():.0f}"),
        ("单次驻留", f"{ji['dur'].median()*1e3:.0f} ms"),
        ("干扰源数量", f"{ji['src'].nunique()} 部"),
    ]
    ax.text(0.02, 0.96, "干扰参数", fontsize=11, fontweight="bold",
            va="top", color="#2c3e50")
    for i, (k, v) in enumerate(rows):
        y = 0.80 - i * 0.098
        ax.text(0.05, y, k, fontsize=9, color="#5b6773", va="center")
        ax.text(0.50, y, v, fontsize=9.5, va="center", fontweight="bold")
        ax.plot([0.02, 0.98], [y - 0.045, y - 0.045], color="#e3e8ec", lw=0.7)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    return fig


# ---------------------------------------------------------------- 图 8
def fig08(D):
    truth = D["radar_ext"]["scene"]
    truth = truth[truth["tid"] == 109].sort_values("t")
    to = D["radar_ext"]["track_out"]
    to = to[(to["tid"] == 109) & (to["track"] == 1)].sort_values("t")
    mt = D["radar_int"]["meas_trk"]
    mt = mt[mt["tid"] == 109].sort_values("t")

    tr, th = _enu(truth)
    fr, fh = _enu(to)
    mr, mh = _enu(mt)
    tt = truth["t"].to_numpy(float)
    tf = to["t"].to_numpy(float)
    tm = mt["t"].to_numpy(float)
    t_end = float(min(tf.max(), tt.max()))          # 真值与航迹的重叠区间

    tv = np.hypot(np.hypot(truth["v0"].to_numpy(float), truth["v1"].to_numpy(float)),
                  truth["v2"].to_numpy(float))
    fv = np.hypot(np.hypot(to["v0"].to_numpy(float), to["v1"].to_numpy(float)),
                  to["v2"].to_numpy(float))
    m_trk = tf >= tt.min()

    # 三维定位误差（相对真值）
    e3 = np.sqrt((np.interp(tf, tt, truth["e0"].to_numpy(float)) - to["e0"].to_numpy(float)) ** 2
                 + (np.interp(tf, tt, truth["e1"].to_numpy(float)) - to["e1"].to_numpy(float)) ** 2
                 + (np.interp(tf, tt, truth["e2"].to_numpy(float)) - to["e2"].to_numpy(float)) ** 2)

    fig, axes = plt.subplots(2, 2, figsize=(12.8, 8.0))
    title_block(fig, "图 8  雷达目标航迹：真值 / 量测 / 滤波",
                f"目标 109；真值 {len(truth)} 点，量测 {len(mt)} 点，滤波航迹 {len(to)} 点；"
                f"观测窗 {tm.min():.0f}–{tm.max():.0f} s，之后转入外推")

    ax = axes[0, 0]
    ax.plot(tr / 1e3, th / 1e3, color=C["truth"], lw=1.8, label="真值")
    ax.plot(mr / 1e3, mh / 1e3, "o", ms=2.4, color=C["meas"], alpha=0.55,
            label="量测")
    m = tf <= t_end
    ax.plot(fr[m] / 1e3, fh[m] / 1e3, color=C["filter"], lw=1.3, label="滤波")
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("海拔高度 (km)")
    ax.set_title("(a) 距离–高度剖面", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=8.5)
    style_axes(ax)

    ax = axes[0, 1]
    ax.plot(tv, th / 1e3, color=C["truth"], lw=1.8, label="真值")
    m = m_trk & (tf <= tm.max())
    ax.plot(fv[m], fh[m] / 1e3, color=C["filter"], lw=1.3, label="滤波")
    ax.set_xlabel("速度 (m/s)")
    ax.set_ylabel("海拔高度 (km)")
    ax.set_title("(b) 速度–高度剖面（观测窗内）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.5)
    style_axes(ax)

    ax = axes[1, 0]
    m = m_trk & (tf <= tm.max())
    ax.plot(tf[m], (fr[m] - np.interp(tf[m], tt, tr)) / 1e3,
            color=C["filter"], lw=1.4, label="真值 − 滤波")
    ax.plot(tm, (mr - np.interp(tm, tt, tr)) / 1e3, ".", ms=3.0,
            color=C["meas"], alpha=0.6, label="真值 − 量测")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("斜距误差 (km)")
    ax.set_title("(c) 观测窗内斜距误差", fontsize=10.5)
    ax.set_xlim(tm.min() - 40, tm.max() + 40)
    ax.legend(loc="upper left", fontsize=8.5)
    style_axes(ax)

    ax = axes[1, 1]
    ax.semilogy(tf[m_trk], np.clip(e3[m_trk] / 1e3, 1e-4, None),
                color=C["filter"], lw=1.2)
    ax.axvline(tm.max(), color=C["warn"], ls="--", lw=1.0, label="观测结束")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("三维定位误差 (km)")
    ax.set_title("(d) 定位误差随跟踪时长", fontsize=10.5)
    ax.annotate(f"观测窗内中位 {np.median(e3[m_trk & (tf <= tm.max())])/1e3:.2f} km\n"
                f"外推末段 {e3[m_trk].max()/1e3:.0f} km",
                xy=(0.04, 0.9), xycoords="axes fraction", fontsize=8.5, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=8.5)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 9
def fig09(D):
    m = D["radar_output"]["meas"].sort_values("t")
    t = m["t"].to_numpy(float)
    dr = m["dr"].to_numpy(float) / 1e3
    daz = m["daz"].to_numpy(float)
    dele = m["del"].to_numpy(float)
    rng = m["rng"].to_numpy(float) / 1e3

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 9  量测误差时序特性",
                f"量测 {len(m)} 点；斜距误差 {np.nanmin(dr):.2f}–{np.nanmax(dr):.1f} km，"
                f"方位/俯仰误差峰值 {np.nanmax(np.abs(daz)):.2f}°/{np.nanmax(np.abs(dele)):.2f}°")

    roll = m["dr"].abs().rolling(45, min_periods=5).max() / 1e3

    ax = axes[0]
    ax.plot(t, dr, lw=0.5, color=C["accent2"], alpha=0.55, label="逐周期误差")
    ax.plot(t, roll, lw=1.5, color="#7a3b12", label="45 s 滑动峰值")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("斜距误差 (km)")
    ax.set_title("(a) 斜距误差", fontsize=10.5)
    ax.annotate(f"峰值 {np.nanmax(np.abs(dr)):.1f} km\n呈锯齿：跟踪更新后复位",
                xy=(0.03, 0.92), xycoords="axes fraction", fontsize=8.3, va="top",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="center right", fontsize=8)
    style_axes(ax)

    ax = axes[1]
    ax.plot(t, daz, lw=0.6, color=C["accent1"], alpha=0.85, label="方位误差")
    ax.plot(t, dele, lw=0.6, color=C["accent4"], alpha=0.85, label="俯仰误差")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("角度误差 (°)")
    ax.set_title("(b) 角度量测误差", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=8)
    style_axes(ax)

    ax = axes[2]
    sc = ax.scatter(rng, dr, c=t, s=5, cmap="viridis", alpha=0.6, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("仿真时间 (s)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("斜距误差 (km)")
    ax.set_title("(c) 误差随距离的变化", fontsize=10.5)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 10
def fig10(D):
    mo = D["radar_output"]["motion"]
    mo = mo[(mo["tid"] == 109) & (mo["track"] == 1)].sort_values("t")
    t = mo["t"].to_numpy(float)
    sp = mo["speed"].to_numpy(float)
    alt = mo["alt"].to_numpy(float) / 1e3
    acc = mo["acc"].to_numpy(float)
    ballistic = mo["cls"].astype(str).eq("弹道目标").to_numpy()

    fig = plt.figure(figsize=(14.2, 7.6))
    title_block(fig, "图 10  目标运动特征：速度–高度与机动判别",
                f"航迹 1（目标 109）运动特征 {len(mo)} 帧；速度 {sp.min():.0f}–{sp.max():.0f} m/s，"
                f"高度 {alt.min():.0f}–{alt.max():.0f} km，弹道判定 {int(ballistic.sum())}/{len(mo)} 帧")

    gs = fig.add_gridspec(2, 3, width_ratios=[1.25, 1.0, 1.0])

    ax = fig.add_subplot(gs[:, 0])
    sc = ax.scatter(sp, alt, c=t, s=8, cmap="viridis", linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("仿真时间 (s)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlabel("速度 (m/s)")
    ax.set_ylabel("高度 (km)")
    ax.set_title("(a) 速度–高度相图", fontsize=10.5)
    style_axes(ax)

    ax = fig.add_subplot(gs[0, 1:3])
    ax.plot(t, sp, color=C["accent1"], lw=1.0, label="速度")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("速度 (m/s)", color=C["accent1"])
    ax.tick_params(axis="y", colors=C["accent1"])
    ax.set_title("(b) 速度与加速度时序", fontsize=10.5)
    ax2 = ax.twinx()
    ax2.plot(t, acc, color=C["warn"], lw=0.8, alpha=0.75, label="加速度")
    ax2.set_ylabel("加速度 (m/s²)", color=C["warn"])
    ax2.tick_params(axis="y", colors=C["warn"])
    ax2.set_ylim(0, 55)
    ax2.grid(False)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8.5)
    style_axes(ax)
    for s in ("top", "right"):
        ax2.spines[s].set_visible(False)

    ax = fig.add_subplot(gs[1, 1])
    ax.fill_between(t, 0, ballistic.astype(float), step="post",
                    color=C["accent3"], alpha=0.55, lw=0)
    ax.plot(t, ballistic.astype(float), drawstyle="steps-post",
            color=C["accent3"], lw=1.2)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["未知", "弹道目标"])
    ax.set_ylim(-0.1, 1.1)
    ax.set_xlabel("仿真时间 (s)")
    ax.set_title("(c) 目标类别判定", fontsize=10.5)
    ax.grid(axis="y", visible=False)
    style_axes(ax)

    ax = fig.add_subplot(gs[1, 2])
    ax.hist(sp, bins=40, color=C["accent1"], alpha=0.85)
    ax.set_xlabel("速度 (m/s)")
    ax.set_ylabel("帧数")
    ax.set_title("(d) 速度分布", fontsize=10.5)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 11
def fig11(D):
    to = D["radar_ext"]["track_out"].sort_values("t")
    lost = D["radar_int"]["trk_lost"]
    res = D["radar_ext"]["res"].sort_values("t")

    fig, axes = plt.subplots(2, 1, figsize=(13.4, 7.2), sharex=True,
                             gridspec_kw={"height_ratios": [1.45, 1.0]})
    title_block(fig, "图 11  航迹生命周期与雷达资源占用",
                f"航迹输出 {len(to)} 条记录（{to['track'].nunique()} 条航迹），"
                f"特征链路丢失记录 {len(lost)} 条，资源池 128")

    ax = axes[0]
    tracks = sorted(to["track"].unique())
    for k, trk in enumerate(tracks):
        g = to[to["track"] == trk]
        t = g["t"].to_numpy(float)
        ok = g["state"].astype(str).eq("确认").to_numpy()
        for a, b in _runs(t, ok):
            ax.plot([a, b], [k, k], lw=8, solid_capstyle="butt",
                    color=C["accent3"], alpha=0.9)
        m = ~ok
        if m.any():
            ax.plot(t[m], np.full(m.sum(), k), lw=8, solid_capstyle="butt",
                    color=C["warn"], alpha=0.75)
    l = lost.groupby("track").size()
    ax.set_yticks(range(len(tracks)))
    ax.set_yticklabels([f"航迹 {int(x)}\n目标 {int(to[to['track'] == x]['tid'].iloc[0])}"
                        for x in tracks])
    ax.set_ylabel("航迹编号 / 目标")
    ax.set_title("(a) 航迹确认（绿）/ 丢失（红）时段", fontsize=10.5)
    ax.set_ylim(-0.7, len(tracks) - 0.3)
    ax.set_xlim(1100, 4700)
    ax.grid(axis="y", visible=False)
    style_axes(ax)

    ax = axes[1]
    ax.step(res["t"], res["used"], where="post", color=C["accent1"], lw=1.6,
            label="已用航迹槽位")
    ax.fill_between(res["t"], 0, res["used"], step="post", color=C["accent1"],
                    alpha=0.15)
    ax.step(res["t"], res["n_conf"], where="post", color=C["accent3"], lw=1.1,
            label="本周期确认数")
    ax.step(res["t"], res["n_lost"], where="post", color=C["warn"], lw=1.1,
            label="本周期丢失数")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("数量 / 槽位")
    ax.set_title("(b) 航迹槽位与确认/丢失去向", fontsize=10.5)
    ax.set_ylim(-0.2, 5.2)
    ax.legend(loc="upper right", fontsize=8.5, ncol=3)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 16
def fig16(D):
    echo = D["radar_output"]["echo"].sort_values("t")
    gn = D["radar_output"]["gain"].sort_values("t")
    det = D["radar_int"]["det"].sort_values("t")

    pc = float(np.median(gn["pc"]))
    mti = float(np.median(gn["mti"]))
    mtd = float(np.median(gn["mtd"]))

    fig = plt.figure(figsize=(14.6, 7.4))
    title_block(fig, "图 16  雷达处理链功率账本",
                f"脉压增益恒为 {pc:.2f} dB；MTI 增益 {mti:.1f} dB"
                f"（{gn['mti'].min():.0f}…{gn['mti'].max():.0f}）；MTD 增益 {mtd:.1f} dB；"
                f"输入回波 {np.median(echo['p_echo']):.2e} W → 输出 "
                f"{np.median(gn['p_after']):.2e} W，SNR 中位 {det['snr_out'].median():.1f} dB")

    ax = fig.add_subplot(1, 2, 1)
    labels = ["脉压", "MTI", "MTD"]
    gains = [pc, mti, mtd]
    base = 0.0
    for i, (lab, g) in enumerate(zip(labels, gains)):
        bottom = min(base, base + g)
        ax.bar(i, abs(g), bottom=bottom, width=0.55,
               color=[C["accent1"], C["accent3"], C["accent2"]][i], alpha=0.9)
        ax.text(i, max(base, base + g) + 1.2, f"{g:+.1f} dB", ha="center",
                fontsize=9.5, fontweight="bold")
        if i < len(gains) - 1:
            ax.plot([i + 0.28, i + 1 - 0.28], [base + g, base + g], ls="--",
                    lw=0.9, color=C["neutral"])
        base += g
    ax.bar(len(labels), base, width=0.55, color="#5b6773", alpha=0.9)
    ax.text(len(labels), base + 1.2, f"累计 {base:+.1f} dB", ha="center",
            fontsize=9.5, fontweight="bold")
    ax.axhline(0, color="#4a4f55", lw=0.9)
    ax.set_xticks(range(len(labels) + 1))
    ax.set_xticklabels(labels + ["链路总增益"])
    ax.set_ylabel("增益 (dB)")
    ax.set_title("(a) 处理链增益瀑布", fontsize=10.5)
    ax.set_ylim(min(0, base) - 4, base + 8)
    style_axes(ax)

    ax = fig.add_subplot(1, 2, 2)
    mg = echo[["cyc", "p_jam", "p_echo", "p_noise"]].merge(
        det[["cyc", "snr_out"]], on="cyc", how="inner")
    js = 10 * np.log10(mg["p_jam"] / mg["p_echo"])
    ax.scatter(js, mg["snr_out"], s=11, color=C["accent4"], alpha=0.65,
               linewidths=0)
    ax.set_xlabel("干信比 J/S (dB)")
    ax.set_ylabel("检测 SNR (dB)")
    ax.set_title("(b) 输出 SNR 随干信比的变化", fontsize=10.5)
    rho = float(np.corrcoef(js, mg["snr_out"])[0, 1])
    ax.annotate(f"Pearson r = {rho:.3f}，{len(mg)} 个处理周期\n"
                f"J/S 中位 {js.median():.0f} dB",
                xy=(0.04, 0.09), xycoords="axes fraction", fontsize=8.5,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb",
                                       ec="#c8d0d8"))
    style_axes(ax)
    return fig


FIGURES = [
    ("04", "fig04_detection_snr", "图 4 检测 SNR 与门限", fig04),
    ("05", "fig05_pd_range", "图 5 检测概率–距离包络", fig05),
    ("06", "fig06_jamming_power", "图 6 干扰功率与抑制", fig06),
    ("07", "fig07_jammer_timeline", "图 7 干扰源参数与几何", fig07),
    ("08", "fig08_track_states", "图 8 雷达轨迹三态", fig08),
    ("09", "fig09_meas_error", "图 9 量测误差时序", fig09),
    ("10", "fig10_speed_altitude", "图 10 速度–高度与机动判别", fig10),
    ("11", "fig11_track_gantt", "图 11 航迹生命周期", fig11),
    ("16", "fig16_power_ledger", "图 16 处理链功率账本", fig16),
]
