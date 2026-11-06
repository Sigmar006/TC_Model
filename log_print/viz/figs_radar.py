# -*- coding: utf-8 -*-
"""图 4–11、16：雷达检测、跟踪、干扰与资源调度。

坐标口径
--------
* 功率 / 功率谱 / 比值等动态范围大的量一律换算成 dB 后线性绘制（10·lg）；
* 不再使用对数坐标轴，刻度直接写 dBW、dBW/Hz、dB(m)、dB(m²)；
* 跟踪类图（8–11）按目标分行：一行只对应一个目标，行首标出目标号与航迹号，
  不同目标的误差、速度、航迹生命周期不再混在同一条曲线上。
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from .common import (C, CYCLE, by_target, db10, db20, enu_to_lla, style_axes,
                     target_title, title_block)


# ---------------------------------------------------------------- 公共
def _enu(df, c0="e0", c1="e1", c2="e2"):
    e = df[c0].to_numpy(dtype=float)
    n = df[c1].to_numpy(dtype=float)
    u = df[c2].to_numpy(dtype=float)
    rng = np.sqrt(e * e + n * n + u * u)
    _, _, h = enu_to_lla(e, n, u)
    return rng, h


def _speed(df):
    return np.hypot(np.hypot(df["v0"].to_numpy(float), df["v1"].to_numpy(float)),
                    df["v2"].to_numpy(float))


def _range_txt(v, unit="", tol=0.01):
    """数值近似恒定则说“恒为”，否则给区间。"""
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if not len(v):
        return "未记录"
    med = float(np.median(v))
    if np.ptp(v) <= tol * max(abs(med), 1e-12):
        return f"恒为 {med:.2f}{unit}"
    return f"{v.min():.2f}–{v.max():.2f}{unit}"


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


# ---------------------------------------------------------------- 图 4
def fig04(D):
    det = D["radar_int"]["det"].sort_values("t")
    plt_ = D["radar_ext"]["plot"].sort_values("t")
    cfar = D["radar_output"]["cfar"].sort_values("t")
    thr = float(det["thr"].median())
    n_tid = int(det["tid"].nunique()) if "tid" in det.columns else 0

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 4  检测信噪比、检测概率与门限",
                f"检测记录 {len(det)} 条（{n_tid} 个目标，"
                f"{det['t'].min():.0f}–{det['t'].max():.0f} s），"
                f"点迹量测 {len(plt_)} 条；CFAR 门限 {_range_txt(det['thr'], ' dB')}")

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
                f"RCS {_range_txt(cfar['rcs90'], ' m²')}",
                xy=(0.04, 0.88), xycoords="axes fraction", fontsize=8.5,
                va="top", bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb",
                                    ec="#c8d0d8"))
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 5
def fig05(D):
    ext = D["radar_ext"]
    p_all = ext["plot"]
    tid = int(p_all["tid"].value_counts().idxmax())     # 点迹最多的目标
    p = p_all[p_all["tid"] == tid].sort_values("t")
    rng = p["rng"].to_numpy(float) / 1e3
    pd_ = p["pd"].to_numpy(float)
    snr = p["snr"].to_numpy(float)
    t = p["t"].to_numpy(float)
    rcs = ext["scene"].loc[ext["scene"]["tid"] == tid, "rcs"].to_numpy(float)
    if len(p) > 2 and np.ptp(snr) > 0 and np.ptp(pd_) > 0:
        rho = float(np.corrcoef(snr, pd_)[0, 1])
    else:
        rho = 0.0
    if rho > 0.15:
        disp = f"同一距离上 $P_d$ 的差异与 SNR 同向（r = {rho:.2f}）"
    elif rho < -0.15:
        disp = f"同一距离上 $P_d$ 的差异与 SNR 反向（r = {rho:.2f}）"
    else:
        disp = "同一距离上 $P_d$ 的差异与 SNR 无显著线性关系"

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 5  检测概率–距离包络",
                f"目标 {tid} 点迹 {len(p)} 条，斜距 {rng.min():.0f}–{rng.max():.0f} km，"
                f"RCS {_range_txt(rcs, ' m²')}；{disp}")

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

    p_db = db10(jp["power"].to_numpy(float))
    js = db10(echo["p_jam"].to_numpy(float) / echo["p_echo"].to_numpy(float))
    jn = db10(echo["p_jam"].to_numpy(float) / echo["p_noise"].to_numpy(float))
    js_med, jn_med = float(np.nanmedian(js)), float(np.nanmedian(jn))

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 6  干扰功率与抗干扰抑制",
                f"干扰源 {jp['src'].nunique()} 部（目标 "
                f"{sorted(jp['src'].unique().astype(int).tolist())}），"
                f"接收干扰功率 {np.nanmin(p_db):.1f}–{np.nanmax(p_db):.1f} dBW")

    ax = axes[0]
    for k, (src, g) in enumerate(jp.groupby("src")):
        ax.plot(g["t"], db10(g["power"].to_numpy(float)), lw=0.8, color=CYCLE[k],
                label=f"干扰源 {int(src)}")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("接收干扰功率 (dBW)")
    ax.set_title("(a) 各干扰源接收功率", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)

    ax = axes[1]
    ax.plot(echo["t"], js, lw=1.1, color=C["warn"], label="J/S")
    ax.plot(echo["t"], jn, lw=1.1, color=C["accent4"], label="J/N")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率比 (dB)")
    ax.set_title("(b) 干信比与干噪比", fontsize=10.5)
    ax.annotate(f"J/S 中位 {js_med:.1f} dB（{10 ** (js_med / 10):.2f} 倍）\n"
                f"J/N 中位 {jn_med:.1f} dB（{10 ** (jn_med / 10):.2f} 倍）",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.3, va="bottom",
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="center right", fontsize=8)
    style_axes(ax)

    ax = axes[2]
    suppr = jg["suppr"].to_numpy(float)
    ax.scatter(db10(jg["p_mti"].to_numpy(float)), suppr, s=9, color=C["accent3"],
               alpha=0.55, linewidths=0)
    ax.axhline(np.median(suppr), color=C["warn"], ls="--", lw=1.0,
               label=f"中位 {np.median(suppr):.1f} dB")
    ax.set_xlabel("MTI 后残余干扰功率 (dBW)")
    ax.set_ylabel("干扰抑制比 (dB)")
    ax.set_ylim(float(np.nanmin(suppr)) - 1.0, float(np.nanmax(suppr)) + 2.2)
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
        ax.plot(g["t"], db10(g["pden"].to_numpy(float)), lw=1.0, color=CYCLE[k],
                label=f"{int(src)}")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("功率谱密度 (dBW/Hz)")
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
    scene = D["radar_ext"]["scene"]
    to_all = D["radar_ext"]["track_out"].sort_values("t")
    mt_all = D["radar_int"]["meas_trk"].sort_values("t")
    groups = by_target(to_all)

    # 先把每个目标的数据整理好，画图阶段只做绘制，避免行内逻辑交叉。
    prepared = []
    for tid, to in groups:
        truth = scene[scene["tid"] == tid].sort_values("t") if "tid" in scene.columns else scene
        meas = mt_all[mt_all["tid"] == tid].sort_values("t") if "tid" in mt_all.columns else mt_all
        tt = truth["t"].to_numpy(float)
        tr, th = _enu(truth)
        mr, mh = _enu(meas)
        fits = []
        for trk in sorted(int(v) for v in to["track"].unique()):
            g = to[to["track"] == trk].sort_values("t")
            tf = g["t"].to_numpy(float)
            fr, fh = _enu(g)
            if len(tt) > 1:
                e3 = np.sqrt((np.interp(tf, tt, truth["e0"].to_numpy(float))
                              - g["e0"].to_numpy(float)) ** 2
                             + (np.interp(tf, tt, truth["e1"].to_numpy(float))
                                - g["e1"].to_numpy(float)) ** 2
                             + (np.interp(tf, tt, truth["e2"].to_numpy(float))
                                - g["e2"].to_numpy(float)) ** 2)
                dr = fr - np.interp(tf, tt, tr)
            else:
                e3 = np.full(len(tf), np.nan)
                dr = np.full(len(tf), np.nan)
            fits.append(dict(track=trk, t=tf, rng=fr, h=fh, v=_speed(g),
                             e3=e3, dr=dr))
        prepared.append(dict(tid=tid, truth=truth, meas=meas, tt=tt, tr=tr,
                             th=th, mr=mr, mh=mh, fits=fits,
                             t_obs=float(meas["t"].max()) if len(meas) else np.nan))

    n = len(prepared)
    names = "、".join(f"目标 {p['tid']}" for p in prepared)
    fig = plt.figure(figsize=(18.2, 3.35 * n + 1.35))
    gs = fig.add_gridspec(n, 5, width_ratios=[0.07, 1, 1, 1, 1])
    heads = ["距离–高度剖面", "速度–高度剖面（观测窗内）",
             "斜距误差（观测窗内）", "三维定位误差（dB，相对 1 m）"]
    title_block(fig, "图 8  雷达多目标航迹：真值 / 量测 / 滤波（按目标分行）",
                f"{n} 个目标（{names}）；真值 {len(scene)} 点，"
                f"量测 {len(mt_all)} 点，航迹 {len(to_all)} 条记录；每行只画一个目标")

    for i, p in enumerate(prepared):
        # 首列只放行标题，保证"一行=一个目标"在图上直读。
        lab = fig.add_subplot(gs[i, 0])
        lab.axis("off")
        lab.text(0.5, 0.5, target_title(p["tid"], [f["track"] for f in p["fits"]]),
                 transform=lab.transAxes, rotation=90, va="center", ha="center",
                 fontsize=10.0, fontweight="bold", color="#2c3e50")
        row = [fig.add_subplot(gs[i, j]) for j in range(1, 5)]
        if i == 0:
            for ax, h in zip(row, heads):
                ax.set_title(h, fontsize=10.5)

        # (a) 距离–高度
        ax = row[0]
        ax.plot(p["tr"] / 1e3, p["th"] / 1e3, color=C["truth"], lw=1.8, label="真值")
        if len(p["meas"]):
            ax.plot(p["mr"] / 1e3, p["mh"] / 1e3, "o", ms=2.2, color=C["meas"],
                    alpha=0.5, label="量测")
        for k, f in enumerate(p["fits"]):
            ax.plot(f["rng"] / 1e3, f["h"] / 1e3, color=C["filter"], lw=1.2,
                    ls="-" if k == 0 else "--", label=f"滤波（航迹 {f['track']}）")
        ax.set_xlabel("斜距 (km)")
        ax.set_ylabel("海拔高度 (km)")
        ax.legend(loc="upper left", fontsize=7.4)
        style_axes(ax)

        # (b) 速度–高度（观测窗内）
        ax = row[1]
        ts = p["tt"]
        tv = _speed(p["truth"]) if len(p["truth"]) else np.array([])
        if len(ts):
            ax.plot(tv, p["th"] / 1e3, color=C["truth"], lw=1.8, label="真值")
        t_end = p["t_obs"]
        if not np.isfinite(t_end):
            t_end = max((float(f["t"].max()) for f in p["fits"]), default=np.nan)
        for k, f in enumerate(p["fits"]):
            m = f["t"] <= t_end
            ax.plot(f["v"][m], f["h"][m] / 1e3, color=C["filter"], lw=1.2,
                    ls="-" if k == 0 else "--", label=f"滤波（航迹 {f['track']}）")
        ax.set_xlabel("速度 (m/s)")
        ax.set_ylabel("海拔高度 (km)")
        ax.legend(loc="upper right", fontsize=7.4)
        style_axes(ax)

        # (c) 斜距误差（观测窗内）：真值 − 滤波 / 真值 − 量测
        ax = row[2]
        for k, f in enumerate(p["fits"]):
            m = f["t"] <= t_end
            ax.plot(f["t"][m], f["dr"][m] / 1e3, color=C["filter"], lw=1.3,
                    ls="-" if k == 0 else "--", label=f"真值 − 滤波 {f['track']}")
        if len(p["meas"]) and len(ts) > 1:
            dm = p["mr"] - np.interp(p["meas"]["t"].to_numpy(float), ts, p["tr"])
            ax.plot(p["meas"]["t"], dm / 1e3, ".", ms=2.6, color=C["meas"],
                    alpha=0.55, label="真值 − 量测")
        ax.set_xlabel("仿真时间 (s)")
        ax.set_ylabel("斜距误差 (km)")
        ax.legend(loc="center right", fontsize=7.2)
        style_axes(ax)

        # (d) 三维定位误差（dB）。误差为 0 时取 1 mm 下限，避免整段变成 NaN。
        ax = row[3]
        base = float(ts.min()) if len(ts) else -np.inf
        for k, f in enumerate(p["fits"]):
            m = f["t"] >= base
            ax.plot(f["t"][m], db20(np.clip(f["e3"][m], 1e-3, None)),
                    color=C["filter"], lw=1.2, ls="-" if k == 0 else "--",
                    label=f"航迹 {f['track']}")
        ax.set_xlabel("仿真时间 (s)")
        ax.set_ylabel("三维定位误差 (dB)")
        ax.legend(loc="lower right", fontsize=7.4)
        style_axes(ax)

        # 行内结论：观测窗中位与外推末段，km 与 dB 两种读数都给。
        f0 = p["fits"][0] if p["fits"] else None
        if f0 is not None:
            in_win = (f0["t"] >= base) & (f0["t"] <= t_end)
            med = float(np.nanmedian(f0["e3"][in_win])) if in_win.any() else np.nan
            last = float(f0["e3"][-1]) if len(f0["e3"]) else np.nan
            txt = (f"观测窗中位 {med/1e3:.2f} km（{float(db20(max(med, 1e-3))):.1f} dB）\n"
                   f"外推末段 {last/1e3:.0f} km（{float(db20(max(last, 1e-3))):.1f} dB）")
            ax.annotate(txt, xy=(0.03, 0.86), xycoords="axes fraction",
                        fontsize=7.6, va="top",
                        bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb",
                                  ec="#c8d0d8"))
    return fig


# ---------------------------------------------------------------- 图 9
def fig09(D):
    m = D["radar_output"]["meas"].sort_values("t")
    groups = by_target(m)

    fig, axes = plt.subplots(1, 3, figsize=(15.4, 4.9))
    title_block(fig, "图 9  量测误差时序特性（按目标）",
                f"量测 {len(m)} 点，{len(groups)} 个目标"
                f"（{'、'.join(target_title(t) for t, _ in groups)}）；"
                f"逐目标统计，不同目标的误差不合并")

    ax = axes[0]
    note = []
    for k, (tid, g) in enumerate(groups):
        t = g["t"].to_numpy(float)
        dr = g["dr"].to_numpy(float) / 1e3
        ax.plot(t, dr, lw=0.5, color=CYCLE[k], alpha=0.55, label=target_title(tid))
        ax.plot(t, g["dr"].abs().rolling(45, min_periods=5).max() / 1e3, lw=1.4,
                color=CYCLE[k])
        note.append(f"{target_title(tid)}：中位 {np.nanmedian(np.abs(dr)):.2f} km、"
                    f"峰值 {np.nanmax(np.abs(dr)):.2f} km")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("斜距误差 (km)")
    ax.set_title("(a) 斜距误差（细线逐周期，粗线 45 s 滑动峰值）", fontsize=10.5)
    ax.legend(loc="center right", fontsize=7.4)
    ax.annotate("\n".join(note), xy=(0.03, 0.92), xycoords="axes fraction",
                fontsize=7.6, va="top",
                bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax)

    ax = axes[1]
    for k, (tid, g) in enumerate(groups):
        ax.plot(g["t"], g["daz"], lw=0.6, color=CYCLE[k], alpha=0.85,
                label=f"{target_title(tid)} 方位")
        ax.plot(g["t"], g["del"], lw=0.6, color=CYCLE[k], alpha=0.55, ls="--",
                label=f"{target_title(tid)} 俯仰")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("角度误差 (°)")
    ax.set_title("(b) 角度量测误差（实线方位 / 虚线俯仰）", fontsize=10.5)
    ax.legend(loc="upper left", fontsize=6.8, ncol=1)
    style_axes(ax)

    ax = axes[2]
    for k, (tid, g) in enumerate(groups):
        ax.scatter(g["rng"].to_numpy(float) / 1e3, g["dr"].to_numpy(float) / 1e3,
                   s=5, color=CYCLE[k], alpha=0.55, linewidths=0,
                   label=target_title(tid))
    ax.set_xlabel("斜距 (km)")
    ax.set_ylabel("斜距误差 (km)")
    ax.set_title("(c) 误差随距离的变化", fontsize=10.5)
    ax.legend(loc="best", fontsize=7.8, markerscale=2.0)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 10
def fig10(D):
    mo_all = D["radar_output"]["motion"]
    groups = by_target(mo_all)
    n = len(groups)

    fig = plt.figure(figsize=(18.2, 3.35 * n + 1.35))
    gs = fig.add_gridspec(n, 5, width_ratios=[0.07, 1, 1, 1, 1])
    heads = ["速度–高度相图", "速度与加速度时序", "目标类别判定", "速度分布"]
    title_block(fig, "图 10  目标运动特征：速度–高度与机动判别（按目标分行）",
                f"{n} 个目标（{'、'.join(f'目标 {t}' for t, _ in groups)}）；"
                f"每行的速度、加速度与类别判定只属于该目标")

    for i, (tid, g) in enumerate(groups):
        g = g.sort_values("t")
        t = g["t"].to_numpy(float)
        sp = g["speed"].to_numpy(float)
        alt = g["alt"].to_numpy(float) / 1e3
        acc = g["acc"].to_numpy(float)
        ballistic = g["cls"].astype(str).eq("弹道目标").to_numpy()
        lab = fig.add_subplot(gs[i, 0])
        lab.axis("off")
        lab.text(0.5, 0.5, target_title(tid, g["track"]), transform=lab.transAxes,
                 rotation=90, va="center", ha="center", fontsize=10.0,
                 fontweight="bold", color="#2c3e50")
        row = [fig.add_subplot(gs[i, j]) for j in range(1, 5)]
        if i == 0:
            for ax, h in zip(row, heads):
                ax.set_title(h, fontsize=10.5)

        ax = row[0]
        sc = ax.scatter(sp, alt, c=t, s=8, cmap="viridis", linewidths=0)
        cb = fig.colorbar(sc, ax=ax, pad=0.02)
        cb.set_label("时间 (s)", fontsize=7.5)
        cb.ax.tick_params(labelsize=7)
        ax.set_xlabel("速度 (m/s)")
        ax.set_ylabel("高度 (km)")
        ax.annotate(f"速度 {sp.min():.0f}–{sp.max():.0f} m/s\n"
                    f"高度 {alt.min():.0f}–{alt.max():.0f} km",
                    xy=(0.03, 0.96), xycoords="axes fraction", fontsize=7.4,
                    va="top", bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb",
                                        ec="#c8d0d8"))
        style_axes(ax)

        ax = row[1]
        ax.plot(t, sp, color=C["accent1"], lw=1.0, label="速度")
        ax.set_xlabel("仿真时间 (s)")
        ax.set_ylabel("速度 (m/s)", color=C["accent1"])
        ax.tick_params(axis="y", colors=C["accent1"])
        ax2 = ax.twinx()
        ax2.plot(t, acc, color=C["warn"], lw=0.8, alpha=0.75, label="加速度")
        ax2.set_ylabel("加速度 (m/s²)", color=C["warn"])
        ax2.tick_params(axis="y", colors=C["warn"])
        ax2.set_ylim(min(0.0, float(acc.min())) - 2.0, float(acc.max()) * 1.1 + 1.0)
        ax2.grid(False)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8.0)
        style_axes(ax)
        for s in ("top", "right"):
            ax2.spines[s].set_visible(False)

        ax = row[2]
        ax.fill_between(t, 0, ballistic.astype(float), step="post",
                        color=C["accent3"], alpha=0.55, lw=0)
        ax.plot(t, ballistic.astype(float), drawstyle="steps-post",
                color=C["accent3"], lw=1.2)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(["未知", "弹道目标"])
        ax.set_ylim(-0.1, 1.1)
        ax.set_xlabel("仿真时间 (s)")
        ax.annotate(f"弹道判定 {int(ballistic.sum())}/{len(ballistic)} 帧",
                    xy=(0.03, 0.88), xycoords="axes fraction", fontsize=7.6,
                    va="top", bbox=dict(boxstyle="round,pad=0.3", fc="#f7f9fb",
                                        ec="#c8d0d8"))
        ax.grid(axis="y", visible=False)
        style_axes(ax)

        ax = row[3]
        ax.hist(sp, bins=40, color=C["accent1"], alpha=0.85)
        ax.set_xlabel("速度 (m/s)")
        ax.set_ylabel("帧数")
        style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 11
def fig11(D):
    to = D["radar_ext"]["track_out"].sort_values("t")
    lost = D["radar_int"]["trk_lost"]
    res = D["radar_ext"]["res"].sort_values("t")

    tids = sorted(int(v) for v in to["tid"].unique())
    color_of = {tid: CYCLE[k % len(CYCLE)] for k, tid in enumerate(tids)}
    pairs = sorted({(int(r.tid), int(r.track)) for r in to[["tid", "track"]].itertuples()})

    fig, axes = plt.subplots(2, 1, figsize=(13.4, 7.2), sharex=True,
                             gridspec_kw={"height_ratios": [1.45, 1.0]})
    cap = int((res["used"] + res["free"]).max())       # 资源池容量由 used+free 推出
    title_block(fig, "图 11  航迹生命周期与雷达资源占用（按目标）",
                f"航迹输出 {len(to)} 条记录（{len(pairs)} 条航迹 / {len(tids)} 个目标），"
                f"特征链路丢失记录 {len(lost)} 条，资源池 {cap} 槽位")

    ax = axes[0]
    for k, (tid, trk) in enumerate(pairs):
        g = to[(to["tid"] == tid) & (to["track"] == trk)].sort_values("t")
        t = g["t"].to_numpy(float)
        ok = g["state"].astype(str).eq("确认").to_numpy()
        for a, b in _runs(t, ok):
            ax.plot([a, b], [k, k], lw=8, solid_capstyle="butt",
                    color=color_of[tid], alpha=0.9)
        m = ~ok
        if m.any():
            ax.plot(t[m], np.full(m.sum(), k), lw=8, solid_capstyle="butt",
                    color=C["warn"], alpha=0.45)
    ax.set_yticks(range(len(pairs)))
    ax.set_yticklabels([f"目标 {tid}\n航迹 {trk}" for tid, trk in pairs])
    ax.set_ylabel("目标 / 航迹")
    ax.set_title("(a) 各目标航迹的确认（目标色）/ 非确认（浅红）时段", fontsize=10.5)
    ax.set_ylim(-0.7, len(pairs) - 0.3)
    t_lo, t_hi = float(to["t"].min()), float(to["t"].max())
    if "t" in lost.columns and len(lost):
        t_lo = min(t_lo, float(lost["t"].min()))
        t_hi = max(t_hi, float(lost["t"].max()))
    t_pad = 0.02 * max(t_hi - t_lo, 1.0)
    ax.set_xlim(t_lo - t_pad, t_hi + t_pad)
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
    ymax = float(max(res["used"].max(), res["n_conf"].max(), res["n_lost"].max()))
    ax.set_ylim(-0.2, max(ymax * 1.15, 1.0))
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
                f"脉压增益 {_range_txt(gn['pc'], ' dB')}；MTI 增益 {mti:.1f} dB"
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
    js = db10(mg["p_jam"].to_numpy(float) / mg["p_echo"].to_numpy(float))
    ax.scatter(js, mg["snr_out"], s=11, color=C["accent4"], alpha=0.65,
               linewidths=0)
    ax.set_xlabel("干信比 J/S (dB)")
    ax.set_ylabel("检测 SNR (dB)")
    ax.set_title("(b) 输出 SNR 随干信比的变化", fontsize=10.5)
    ok = np.isfinite(js)
    rho = float(np.corrcoef(js[ok], mg["snr_out"].to_numpy(float)[ok])[0, 1])
    ax.annotate(f"Pearson r = {rho:.3f}，{len(mg)} 个处理周期\n"
                f"J/S 中位 {np.nanmedian(js):.0f} dB",
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
    ("08", "fig08_track_states", "图 8 雷达多目标轨迹三态", fig08),
    ("09", "fig09_meas_error", "图 9 量测误差时序", fig09),
    ("10", "fig10_speed_altitude", "图 10 速度–高度与机动判别", fig10),
    ("11", "fig11_track_gantt", "图 11 航迹生命周期", fig11),
    ("16", "fig16_power_ledger", "图 16 处理链功率账本", fig16),
]
