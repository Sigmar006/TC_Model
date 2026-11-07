# -*- coding: utf-8 -*-
"""图 40–43：UKF 协方差与轨道、融合状态精度、宽场覆盖重访、落点/发射点精度。

坐标口径
--------
* 方差、σ、长度与速度一律换算成 dB（方差 10·lg，σ/长度/速度 20·lg）后线性绘制，
  不使用对数坐标轴；
* 本批 `ukf` 表没有目标号，同一时刻常有两行，图 40 按高度把两条支路分开画，
  不强行给两条支路编号；
* `msf` 每历元只有 1 个信息源，图 41 不宣称“多源融合增益”；
* `pos_err` 是卫星自身定轨误差，`msf` 的 cov 是目标状态协方差，两者不混用。
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from .common import (C, CYCLE, all_obs_span, color_map, db10, db20, needs,
                     obs_clip, obs_note, style_axes, title_block)


def _rng_txt(v, unit="", nd=2):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if not v.size:
        return "未记录"
    if v.min() == v.max():
        return f"{v.min():.{nd}f}{unit}"
    return f"{v.min():.{nd}f}–{v.max():.{nd}f}{unit}"


def _cdf(ax, data, label, color, **kw):
    s = np.sort(np.asarray(data, dtype=float))
    s = s[np.isfinite(s)]
    if not s.size:
        return
    p = np.arange(1, s.size + 1) / s.size
    ax.plot(s, p, lw=1.5, color=color, label=label, **kw)


def _parse3(series):
    """把 "a,b,c" 字符串列解析成 (n, 3) 数组。"""
    out = []
    for v in series:
        try:
            parts = [float(x) for x in str(v).split(",")]
        except ValueError:
            parts = []
        out.append(parts[:3] if len(parts) >= 3 else [np.nan] * 3)
    return np.asarray(out, dtype=float)


# ---------------------------------------------------------------- 图 40
@needs(("sat", "ukf", "UKF滤波"),
       title="图 40 UKF 协方差与轨道状态")
def fig40(D):
    uk_all = D["sat"]["ukf"].sort_values("t")
    # ukf 表没有目标键，只能取全批观测时段的并集；超出的外推段默认不画。
    lo_obs, hi_obs = all_obs_span(D)
    keep, n_cut = obs_clip(uk_all["t"].to_numpy(float), lo_obs, hi_obs)
    if not keep.any():
        keep, n_cut = np.ones(len(uk_all), dtype=bool), 0
    uk = uk_all[keep].reset_index(drop=True)
    lla = _parse3(uk["lla"])
    t = uk["t"].to_numpy(float)
    covtr = uk["covtr"].to_numpy(float)
    h = lla[:, 2] / 1e3
    lat, lon = lla[:, 0], lla[:, 1]

    # 本表没有目标号：按高度把高轨支（≈3.5 万 km）与低轨支（≈1–3 千 km）分开，
    # 只在图上标明两支的样本量，不臆造编号。
    hi = h > 1.0e4
    lo = ~hi & np.isfinite(h)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    clip_txt = obs_note(n_cut, hi_obs, "UKF")
    title_block(fig, "图 40  UKF 协方差迹与轨道高度、星下点",
                f"UKF 记录 {len(uk)} 条，{uk['t'].nunique()} 个时刻；"
                f"高轨支 {int(hi.sum())} 条（{_rng_txt(h[hi], ' km', 0)}），"
                f"低轨支 {int(lo.sum())} 条（{_rng_txt(h[lo], ' km', 0)}）；"
                f"同一时刻常有两行，表中无目标号"
                + (f"；{clip_txt}" if clip_txt else ""))

    ax = axes[0, 0]
    ax.plot(t[hi], h[hi], ".", ms=2.8, color=C["pred"], alpha=0.75,
            label=f"高轨支 {int(hi.sum())} 条", zorder=3)
    ax.plot(t[lo], h[lo], ".", ms=2.2, color=C["accent1"], alpha=0.4,
            label=f"低轨支 {int(lo.sum())} 条")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("轨道高度 (km)")
    ax.set_ylim(-1200, 38000)
    ax.set_title("(a) 高度时序：两条支路并存", fontsize=10.5)
    ax.annotate(f"高轨支中位 {float(np.median(h[hi])):.0f} km\n"
                f"低轨支中位 {float(np.median(h[lo])):.0f} km",
                xy=(0.35, 0.62), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="center right", fontsize=8, markerscale=4)
    style_axes(ax)

    ax = axes[0, 1]
    cov_db = db10(covtr)
    ax.plot(t[hi], cov_db[hi], ".", ms=2.5, color=C["pred"], alpha=0.6,
            label="高轨支", zorder=3)
    ax.plot(t[lo], cov_db[lo], ".", ms=2.2, color=C["accent1"], alpha=0.35,
            label="低轨支")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("协方差迹 $\\mathrm{tr}(P)$ (dB re 1 m²)")
    ax.set_title("(b) 协方差迹时序", fontsize=10.5)
    ax.annotate(f"高轨支中位 {float(np.median(cov_db[hi])):.1f} dB\n"
                f"低轨支中位 {float(np.median(cov_db[lo])):.1f} dB\n"
                f"低轨 5% 分位 {float(np.nanpercentile(cov_db[lo], 5)):.1f} dB",
                xy=(0.30, 0.62), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8, markerscale=4)
    style_axes(ax)

    ax = axes[1, 0]
    sc = ax.scatter(lon, lat, c=db20(np.clip(h, 1e-3, None)), s=5,
                    cmap="viridis", alpha=0.5, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label("轨道高度 (dB re 1 km)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xticks(np.arange(-180, 181, 60))
    ax.set_yticks(np.arange(-90, 91, 30))
    ax.grid(True, color=C["grid"], lw=0.6, alpha=0.7)
    ax.set_xlabel("经度 (°)")
    ax.set_ylabel("纬度 (°)")
    ax.set_title("(c) 星下点分布（离线，无底图）", fontsize=10.5)
    ax.annotate(f"高轨支经度中位 {float(np.median(lon[hi])):.1f}°\n"
                f"（跨度 {float(np.ptp(lon[hi])):.1f}°，近似定点）",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.0,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    style_axes(ax, minor=False)

    ax = axes[1, 1]
    _cdf(ax, cov_db[lo], "低轨支", C["accent1"])
    _cdf(ax, cov_db[hi], "高轨支", C["pred"])
    for v, lb in ((float(np.median(cov_db[lo])), "低轨中位"),
                  (float(np.median(cov_db[hi])), "高轨中位")):
        ax.axvline(v, color=C["warn"], ls="--", lw=0.9, alpha=0.8)
        ax.annotate(f"{lb} {v:.1f} dB", xy=(v, 0.55), xytext=(4, 0),
                    textcoords="offset points", rotation=90, fontsize=7.4,
                    color=C["warn"], va="center")
    ax.set_xlabel("协方差迹 $\\mathrm{tr}(P)$ (dB re 1 m²)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title("(d) 协方差迹分布", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 41
@needs(("sat", "msf", "多传感器目标跟踪功能测试"),
       title="图 41 融合状态精度与信息源")
def fig41(D):
    m = D["sat"]["msf"].sort_values("t")
    # cov 是 6 个逗号分隔的方差（前 3 位置、后 3 速度）；位置 σ 取三轴方差均值再开方。
    varr = np.asarray([[float(v) for v in s.split(",")[:3]]
                       for s in m["cov"].astype(str)], dtype=float)
    pos_sig = np.sqrt(np.clip(varr.mean(axis=1), 0.0, None))
    vel_sig = m["vel_sigma"].to_numpy(float)
    pos_db = db20(pos_sig)
    vel_db = db20(vel_sig)
    t = m["t"].to_numpy(float)
    keys = sorted(int(v) for v in m["key"].dropna().unique())
    kcmap = color_map(keys)
    srcs = m["srcs"].astype(str)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 41  融合状态位置/速度 σ 与信息源构成（按目标键）",
                f"融合记录 {len(m)} 条，{len(keys)} 个目标键"
                f"（{'、'.join(str(k) for k in keys)}）；"
                f"位置 σ {_rng_txt(pos_sig, ' m', 1)}，"
                f"速度 σ {_rng_txt(vel_sig, ' m/s', 1)}；颜色按目标键区分")

    ax = axes[0, 0]
    for key in keys:
        s = m["key"].to_numpy(float) == key
        ax.plot(t[s], pos_db[s], ".", ms=2.0,
                color=kcmap[key], alpha=0.45,
                label=f"目标键 {key}")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("位置 σ (dB re 1 m)")
    ax.set_title("(a) 位置 σ 时序", fontsize=10.5)
    ax.annotate(f"中位 {float(np.median(pos_db)):.1f} dB（{float(np.median(pos_sig)):.1f} m）\n"
                f"95% {float(np.percentile(pos_db, 95)):.1f} dB"
                f"（{float(np.percentile(pos_sig, 95)):.0f} m）\n"
                f"最大 {float(np.max(pos_db)):.1f} dB"
                f"（{float(np.max(pos_sig)):.2e} m）",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=7.8,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper center", fontsize=7.8, markerscale=2.5, ncol=2)
    style_axes(ax)

    ax = axes[0, 1]
    for key in keys:
        s = m["key"].to_numpy(float) == key
        ax.plot(t[s], vel_db[s], ".", ms=2.0,
                color=kcmap[key], alpha=0.45,
                label=f"目标键 {key}")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("速度 σ (dB re 1 m/s)")
    ax.set_title("(b) 速度 σ 时序", fontsize=10.5)
    ax.annotate(f"中位 {float(np.median(vel_db)):.1f} dB（{float(np.median(vel_sig)):.2f} m/s）\n"
                f"95% {float(np.percentile(vel_db, 95)):.1f} dB"
                f"（{float(np.percentile(vel_sig, 95)):.1f} m/s）",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=7.8,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="center left", fontsize=7.8, markerscale=2.5)
    style_axes(ax)

    ax = axes[1, 0]
    uniq = [s for s in sorted(srcs.unique())]
    x = np.arange(len(uniq))
    w = 0.38
    for k, key in enumerate(keys):
        s_all = m["key"].to_numpy(float) == key
        cnt = [int((s_all & (srcs == u).to_numpy()).sum()) for u in uniq]
        ax.bar(x + (k - 0.5) * w, cnt, width=w,
               color=kcmap[key], alpha=0.85, label=f"目标键 {key}")
    ax.set_xticks(x)
    ax.set_xticklabels(uniq, fontsize=8, rotation=12)
    ax.set_ylabel("融合记录数")
    ax.set_ylim(0, max(int((srcs == u).sum()) for u in uniq) * 1.28)
    ax.set_title("(c) 信息源构成（每条记录仅 1 个源）", fontsize=10.5)
    ax.annotate("每历元仅 1 个信息源，本批数据不含\n"
                "多源同时刻融合的对比记录，不能据此\n"
                "宣称“多源融合增益”",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=7.8,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#fdf6ec", ec="#e6c79c"))
    ax.legend(loc="upper right", fontsize=8)
    style_axes(ax)

    ax = axes[1, 1]
    for key in keys:
        s = m["key"].to_numpy(float) == key
        _cdf(ax, pos_db[s], f"目标键 {key}", kcmap[key])
    ax.axvline(60.0, color=C["neutral"], ls=":", lw=0.9,
               label="1 km = 60 dB")
    ax.set_xlabel("位置 σ (dB re 1 m)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title("(d) 位置 σ 累积分布", fontsize=10.5)
    ax.annotate(f"{int((pos_sig > 1e3).sum())}/{len(pos_sig)} 条超过 1 km\n"
                f"地面站：{m['ground'].iloc[0] if len(m) else '无'}",
                xy=(0.03, 0.34), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 42
@needs(("sat", "wfov", "宽视场扫描探测功能测试"),
       title="图 42 宽场覆盖与重访")
def fig42(D):
    wf = D["sat"]["wfov"].sort_values("t")
    ok_all = wf["center"].astype(str).str.contains(",", regex=False)
    sats = sorted(int(v) for v in wf["sat"].dropna().unique())
    scmap = color_map(sats)

    # 每颗卫星自己有 1° 网格：单元计数与重访间隔都按卫星分别统计，
    # 不把五颗星的覆盖次数加在同一格上（那样会虚增单格覆盖数）。
    stats = {}
    for s in sats:
        g = wf[wf["sat"].to_numpy(float) == s]
        ok = g["center"].astype(str).str.contains(",", regex=False)
        c = g[ok]
        lat = c["center"].str.split(",").str[0].astype(float).to_numpy()
        lon = c["center"].str.split(",").str[1].astype(float).to_numpy()
        ct = c["t"].to_numpy(float)
        cells, inv, counts = np.unique(
            np.column_stack([np.round(lat).astype(int),
                             np.round(lon).astype(int)]), axis=0,
            return_inverse=True, return_counts=True)
        gaps = []
        order = np.argsort(inv, kind="stable")
        inv_s, t_s = inv[order], ct[order]
        for cid in range(len(cells)):
            tt = np.unique(t_s[inv_s == cid])
            if tt.size > 1:
                gaps.extend(np.diff(tt).tolist())
        stats[s] = dict(lat=lat, lon=lon, cells=cells, counts=counts,
                        gaps=np.asarray(gaps, dtype=float),
                        n=int(ok.sum()), tot=len(g))

    gaps_all = np.concatenate([stats[s]["gaps"] for s in sats]) \
        if any(stats[s]["gaps"].size for s in sats) else np.asarray([])
    lat_all = np.concatenate([stats[s]["lat"] for s in sats])
    n_cells = sum(len(stats[s]["cells"]) for s in sats)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 42  宽场覆盖单元与重访间隔（按卫星）",
                f"扫描记录 {len(wf)} 条，有效中心 {int(ok_all.sum())} 条"
                f"（{ok_all.mean()*100:.1f}%）；"
                f"1° 网格单元合计 {n_cells} 个，重访样本 {len(gaps_all)} 个；"
                f"颜色＝卫星")

    ax = axes[0, 0]
    for s in sats:
        st = stats[s]
        if not len(st["cells"]):
            continue
        ax.scatter(st["cells"][:, 1], st["cells"][:, 0],
                   s=8 + 2.0 * st["counts"], color=scmap[s], alpha=0.55,
                   linewidths=0, label=f"卫星 {s}（{len(st['cells'])} 单元）")
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xticks(np.arange(-180, 181, 60))
    ax.set_yticks(np.arange(-90, 91, 30))
    ax.grid(True, color=C["grid"], lw=0.6, alpha=0.7)
    ax.set_xlabel("经度 (°)")
    ax.set_ylabel("纬度 (°)")
    ax.set_title("(a) 覆盖单元分布（颜色＝卫星，点大小∝单元覆盖数）", fontsize=9.6)
    ax.annotate(f"纬度 {_rng_txt(lat_all, '°', 1)}\n经度覆盖全圈（±180° 回绕）",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.0,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=6.8)
    style_axes(ax, minor=False)

    ax = axes[0, 1]
    hit = [stats[s]["n"] for s in sats]
    miss = [stats[s]["tot"] - stats[s]["n"] for s in sats]
    x = np.arange(len(sats))
    ax.bar(x, miss, width=0.6, color=C["neutral"], alpha=0.45, label="中心无效")
    for xi, s in zip(x, sats):
        ax.bar(xi, stats[s]["n"], width=0.6, bottom=stats[s]["tot"] - stats[s]["n"],
               color=scmap[s], alpha=0.9, label=f"卫星 {s} 有效")
    ax.set_xticks(x)
    ax.set_xticklabels([f"卫星 {s}" for s in sats], fontsize=8)
    ax.set_ylabel("扫描记录数")
    ax.set_ylim(0, max(np.add(hit, miss)) * 1.38)
    ax.set_title("(b) 有效中心占比", fontsize=10.5)
    for xi, s, h_, m_ in zip(x, sats, hit, miss):
        ax.annotate(f"{h_/(h_+m_)*100:.1f}%", xy=(xi, h_ + m_),
                    xytext=(0, 3), textcoords="offset points",
                    ha="center", fontsize=7.2, color=scmap[s])
    ax.legend(loc="upper left", fontsize=6.6, ncol=2)
    style_axes(ax)

    ax = axes[1, 0]
    allc = np.concatenate([stats[s]["counts"] for s in sats])
    edges = np.arange(0.5, float(np.max(allc)) + 1.5, 1.0)
    for s in sats:
        ax.hist(stats[s]["counts"], bins=edges, histtype="step", lw=1.3,
                color=scmap[s], label=f"卫星 {s}（中位 {float(np.median(stats[s]['counts'])):.0f} 次）")
    ax.set_xlabel("单个 1° 单元的覆盖记录数")
    ax.set_ylabel("单元数")
    ax.set_title("(c) 单元覆盖次数分布（按卫星叠加）", fontsize=10.5)
    ax.annotate(f"五颗星单元合计 {n_cells} 个，单格最高 {int(np.max(allc))} 次",
                xy=(0.40, 0.58), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=6.6)
    style_axes(ax)

    ax = axes[1, 1]
    for s in sats:
        g = stats[s]["gaps"]
        if g.size >= 3:
            _cdf(ax, db10(g), f"卫星 {s}（{len(g)} 个）", scmap[s])
    if gaps_all.size:
        # 本批重访间隔高度量化，50% 与 90% 分位常常同为 40 s，重复画线会叠字。
        qs = {}
        for q in (50, 90, 99):
            qs.setdefault(float(np.percentile(gaps_all, q)), []).append(q)
        for v, ql in sorted(qs.items()):
            ax.axvline(float(db10(v)), color=C["neutral"], ls="-.", lw=0.9,
                       alpha=0.8)
            txt = "/".join(f"{q}%" for q in ql)
            ax.annotate(f"{txt} {v:.0f} s", xy=(float(db10(v)), 0.45),
                        xytext=(4, 0), textcoords="offset points",
                        fontsize=7.4, color=C["neutral"], ha="left",
                        va="center")
    ax.set_xlabel("同一单元相邻访问间隔 (dB re 1 s)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title(f"(d) 重访间隔累积分布（全体 {len(gaps_all)} 个样本）", fontsize=10.5)
    ax.legend(loc="lower right", fontsize=6.8)
    style_axes(ax)
    return fig


# ---------------------------------------------------------------- 图 43
@needs(("sat", "impact", "落点预报与信息分发功能测试"),
       ("sat", "launch", "发射点预测功能测试"),
       ("sat", "pos_err", "卫星自身定位误差功能测试"),
       title="图 43 落点、发射点与定轨精度")
def fig43(D):
    im = D["sat"]["impact"].sort_values("t")
    la = D["sat"]["launch"].sort_values("t")
    pe = D["sat"]["pos_err"]

    # impact / launch 只有 key（99、109），没有卫星号；pos_err 有 sat（103–107）。
    # 目标键与卫星分别由 color_map 统一配色，跨图同名实体同色。
    keys = sorted({int(v) for v in im["key"].dropna().unique()}
                  | {int(v) for v in la["key"].dropna().unique()})
    kcmap = color_map(keys)
    sats = sorted(int(v) for v in pe["sat"].dropna().unique())
    scmap = color_map(sats)

    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.4))
    title_block(fig, "图 43  落点/发射点预报误差与卫星定轨误差（按目标键/卫星）",
                f"落点 {len(im)} 条、发射点 {len(la)} 条、定轨 {len(pe)} 条；"
                f"落点 1σ {_rng_txt(im['r1'], ' m', 0)}，"
                f"定轨误差 {_rng_txt(pe['err'], ' m', 2)}；"
                f"落点/发射点按目标键 {'、'.join(str(k) for k in keys)} 分色，"
                f"定轨按卫星 {'、'.join(str(s) for s in sats)} 分色")

    ax = axes[0, 0]
    for key in keys:
        s = im["key"].to_numpy(float) == key
        ax.plot(im["t"].to_numpy(float)[s], db20(im["r1"].to_numpy(float)[s]),
                ".", ms=2.2, color=kcmap[key], alpha=0.4,
                label=f"目标键 {key}（{int(s.sum())} 条）")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("落点误差范围 1σ (dB re 1 m)")
    ax.set_title("(a) 落点 1σ 误差时序（颜色＝目标键）", fontsize=10.5)
    q50, q95 = np.percentile(im["r1"].to_numpy(float), [50, 95])
    ax.annotate(f"置信度 {im['conf'].iloc[0]:.2f}\n"
                f"中位 {q50:.0f} m，95% {q95:.0f} m",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.2,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=8, markerscale=4)
    style_axes(ax)

    ax = axes[0, 1]
    ea = la["ea"].to_numpy(float)
    eb = la["eb"].to_numpy(float)
    ea_db, eb_db = db20(ea), db20(eb)
    bad = ea > 1e6
    for key in keys:
        s = la["key"].to_numpy(float) == key
        ax.plot(eb_db[s & ~bad], ea_db[s & ~bad], ".", ms=3.2, color=kcmap[key],
                alpha=0.35, label=f"目标键 {key}（{int((s & ~bad).sum())} 条）")
        if (s & bad).any():
            ax.plot(eb_db[s & bad], ea_db[s & bad], "x", ms=5, color=kcmap[key],
                    alpha=0.85, label=f"目标键 {key} 半长轴异常 {int((s & bad).sum())} 条")
    lo = float(np.nanmin(np.concatenate([ea_db, eb_db]))) - 3.0
    hi = float(np.nanmax(np.concatenate([ea_db, eb_db]))) + 3.0
    ax.plot([lo, hi], [lo, hi], color=C["neutral"], ls="--", lw=0.9, label="1:1")
    ax.set_xlabel("误差椭圆半短轴 $e_b$ (dB re 1 m)")
    ax.set_ylabel("误差椭圆半长轴 $e_a$ (dB re 1 m)")
    ax.set_title("(b) 发射点误差椭圆长短轴（颜色＝目标键）", fontsize=10.5)
    ax.annotate(f"$e_a$ 中位 {float(np.median(ea)):.0f} m\n"
                f"$e_a$ 最大 {float(np.max(ea)):.2e} m\n"
                f"异常大 {int(bad.sum())} 条（× 标记）",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.0,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=6.8)
    style_axes(ax)

    ax = axes[1, 0]
    eaz = la["eaz"].to_numpy(float)
    edges = np.histogram_bin_edges(eaz, bins=40)
    for key in keys:
        s = la["key"].to_numpy(float) == key
        v = eaz[s]
        ax.hist(v, bins=edges, histtype="step", lw=1.4, color=kcmap[key],
                label=f"目标键 {key}（{len(v)} 条，中位 {float(np.median(v)):+.1f}°）")
        ax.axvline(float(np.median(v)), color=kcmap[key], ls="--", lw=0.9)
    ax.axvline(0.0, color=C["neutral"], ls=":", lw=0.9)
    ax.set_xlabel("椭圆方位角 (°)")
    ax.set_ylabel("发射点记录数")
    ax.set_title("(c) 误差椭圆方位角分布（按目标键叠加）", fontsize=10.5)
    ax.annotate(f"{_rng_txt(eaz, '°', 1)}\n"
                f"全体中位 {float(np.median(eaz)):+.1f}°",
                xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8.2,
                va="top", bbox=dict(boxstyle="round,pad=0.35",
                                    fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="upper right", fontsize=6.8)
    style_axes(ax)

    ax = axes[1, 1]
    for s_ in sats:
        sub = pe[pe["sat"] == s_]["err"].to_numpy(float)
        _cdf(ax, db20(np.clip(sub, 1e-3, None)),
             f"卫星 {s_}（{len(sub)} 条，中位 {float(np.median(sub)):.2f} m）",
             scmap[s_])
    ax.set_xlabel("卫星自身定位误差 (dB re 1 m)")
    ax.set_ylabel("累积概率")
    ax.set_ylim(0, 1.02)
    ax.set_title(f"(d) 定轨误差累积分布（{len(sats)} 颗卫星）", fontsize=10.5)
    ax.annotate(f"零误差 {int((pe['err'] <= 0).sum())} 条（置 $10^{{-3}}$ m = −60 dB）\n"
                f"中位 {float(np.median(pe['err'])):.2f} m，"
                f"99% {float(np.percentile(pe['err'], 99)):.2f} m",
                xy=(0.03, 0.06), xycoords="axes fraction", fontsize=8.0,
                va="bottom", bbox=dict(boxstyle="round,pad=0.35",
                                       fc="#f7f9fb", ec="#c8d0d8"))
    ax.legend(loc="lower right", fontsize=6.8)
    style_axes(ax)
    return fig


FIGURES = [
    ("40", "fig40_ukf_covariance", "图 40 UKF 协方差与轨道", fig40),
    ("41", "fig41_msf_accuracy", "图 41 融合状态精度与信息源", fig41),
    ("42", "fig42_wfov_coverage", "图 42 宽场覆盖与重访", fig42),
    ("43", "fig43_impact_launch", "图 43 落点与定轨精度", fig43),
]
