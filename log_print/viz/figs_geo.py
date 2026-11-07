# -*- coding: utf-8 -*-
"""图 44–54：雷达目标几何时序、卫星星下点与宽场扫描轨迹。

坐标口径
--------
* 高度、斜距、ECEF 分量都是千米级长度量，量级集中在 10³–10⁴ km，按 km 线性绘制，
  不转 dB（与图 40 的轨道高度同口径）；dB 只用于跨数量级的量。
* 一张图只画一个实体：44–49 按目标序位分幅，50–54 按卫星序位分幅。标题里的编号
  取自当次日志，序位只是图号占位；本批不足该序位时改画说明卡，不臆造曲线。
* “探测高度”取航迹关联/落点预测同源的 LLA 高度（`assoc.h`），并与滤波航迹
  ECEF→LLA 的高度并列，两者同源不同环节，便于核对；“运动特征”里的 `高度`
  是站心 ENU 的 u 分量（= R·sin el），不是海拔高度，不在此图混用。
* 卫星图为 GEO（本批 103–107），星下点漂移 < 0.1°，轨迹缩成一个点，故以标记点
  表示，不把首拍与稳态连成跨半球的长线；经度用“相对星下点经度”，避免全盘覆盖
  跨 ±180° 时断成两段。
  `wfov.quad` 是“视场矩形 ∩ 地球视盘”的经纬度包络框（源码
  `SbirsPipeline.cpp` ComputeGroundCoverageEnvelope），四角顺序为西北/东北/
  东南/西南，故按经纬度矩形连线；t=1 的轨道初值帧（300 km）与稳态不连续，剔除。
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from .common import (C, by_target, color_map, ecef_to_lla, empty_fig, enu_to_ecef,
                     needs, style_axes, target_title, title_block)


def _rng_txt(v, unit="", nd=0):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if not v.size:
        return "未记录"
    if float(v.min()) == float(v.max()):
        return f"{v.min():.{nd}f}{unit}"
    # 负值区间用 “~” 分隔，避免 “-4862–-3440” 这类读不出边界的写法。
    sep = " ~ " if float(v.min()) < 0 else "–"
    return f"{v.min():.{nd}f}{sep}{v.max():.{nd}f}{unit}"


def _note(ax, text, xy=(0.03, 0.96), va="top", ha="left", size=8.2):
    ax.annotate(text, xy=xy, xycoords="axes fraction", fontsize=size,
                va=va, ha=ha,
                bbox=dict(boxstyle="round,pad=0.35", fc="#f7f9fb", ec="#c8d0d8"))


def _slot_target(D, rank):
    """按目标序位取 (目标号, 航迹表)；不足该序位返回 (None, None)。"""
    kf = D["radar_output"]["kf"]
    if kf is None or len(kf) == 0:
        return None, None
    groups = by_target(kf)
    if rank >= len(groups):
        return None, None
    tid, g = groups[rank]
    return tid, g.sort_values("t")


def _site_ecef():
    x, y, z = enu_to_ecef(0.0, 0.0, 0.0)
    return float(x), float(y), float(z)


def _kf_lla(g):
    return ecef_to_lla(g["x"].to_numpy(float), g["y"].to_numpy(float),
                       g["z"].to_numpy(float))


def _kf_range_km(g):
    x0, y0, z0 = _site_ecef()
    return np.hypot(np.hypot(g["x"].to_numpy(float) - x0,
                             g["y"].to_numpy(float) - y0),
                    g["z"].to_numpy(float) - z0) / 1e3


# ---------------------------------------------------------------- 图 44–45
@needs(("radar_output", "kf", "跟踪滤波功能测试"),
       title="图 44–45 探测高度时序")
def _altitude(D, rank, num):
    tid, g = _slot_target(D, rank)
    if tid is None:
        return empty_fig(f"图 {num}  探测高度时序",
                         f"本批日志只解析到 {len(by_target(D['radar_output']['kf']))} 个目标，"
                         f"没有第 {rank + 1} 个目标")
    t = g["t"].to_numpy(float)
    _, _, h = _kf_lla(g)
    h = h / 1e3
    assoc = D["radar_output"]["assoc"]
    a = assoc[assoc["tid"] == tid].sort_values("t") if "tid" in assoc.columns \
        else assoc.iloc[0:0]
    col = color_map([tid])[tid]

    fig, ax = plt.subplots(figsize=(11.4, 5.2))
    ax.plot(t, h, color=col, lw=1.8,
            label=f"滤波航迹高度（ECEF→LLA，{len(g)} 点）")
    if len(a):
        ax.plot(a["t"].to_numpy(float), a["h"].to_numpy(float) / 1e3, "o", ms=2.8,
                color=C["meas"], alpha=0.75, mew=0,
                label=f"关联时刻探测高度（{len(a)} 点）")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("海拔高度 (km)")
    ax.set_title("(a) 目标高度–时间", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2)

    hi, lo = float(h.max()), float(h.min())
    span = max(hi - lo, 1e-6)
    ax.set_ylim(lo - 0.10 * span, hi + 0.18 * span)
    _note(ax, f"{target_title(tid)}；时间 {_rng_txt(t, ' s')}\n"
              f"高度 {_rng_txt(h, ' km')}；首末 {h[0]:.0f} → {h[-1]:.0f} km"
              f"（{h[-1] - h[0]:+.0f} km）")
    style_axes(ax)
    title_block(fig, f"图 {num}  {target_title(tid)} 探测高度时序",
                f"高度取航迹关联同源的 LLA 高度；本图只画目标 {int(tid)}，"
                f"{len(g)} 条滤波记录、{len(a)} 条关联记录")
    return fig


def fig44(D):
    return _altitude(D, 0, "44")


def fig45(D):
    return _altitude(D, 1, "45")


# ---------------------------------------------------------------- 图 46–47
@needs(("radar_ext", "plot", "雷达对外目标信息输出接口测试-点迹量测"),
       ("radar_output", "kf", "跟踪滤波功能测试"),
       title="图 46–47 斜距时序")
def _range(D, rank, num):
    tid, g = _slot_target(D, rank)
    if tid is None:
        return empty_fig(f"图 {num}  斜距时序",
                         f"本批日志只解析到 {len(by_target(D['radar_output']['kf']))} 个目标，"
                         f"没有第 {rank + 1} 个目标")
    plot = D["radar_ext"]["plot"]
    p = plot[plot["tid"] == tid].sort_values("t") if "tid" in plot.columns \
        else plot.sort_values("t")
    rng_kf = _kf_range_km(g)
    col = color_map([tid])[tid]

    fig, ax = plt.subplots(figsize=(11.4, 5.2))
    ax.plot(g["t"].to_numpy(float), rng_kf, color=C["filter"], lw=1.5, ls="--",
            label=f"滤波航迹反算斜距（{len(g)} 点）")
    if len(p):
        ax.plot(p["t"].to_numpy(float), p["rng"].to_numpy(float) / 1e3, "-o",
                ms=2.6, mew=0, lw=1.4, color=col, alpha=0.9,
                label=f"雷达量测斜距（{len(p)} 点）")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("雷达–目标斜距 (km)")
    ax.set_title("(a) 斜距–时间", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2)

    vals = p["rng"].to_numpy(float) / 1e3 if len(p) else rng_kf
    if len(p):
        _note(ax, f"{target_title(tid)}；时间 {_rng_txt(p['t'].to_numpy(float), ' s')}\n"
                  f"量测斜距 {_rng_txt(vals, ' km')}；首末 {vals[0]:.0f} → {vals[-1]:.0f} km"
                  f"（{vals[-1] - vals[0]:+.0f} km）",
              xy=(0.03, 0.04), va="bottom")
    else:
        _note(ax, f"{target_title(tid)}；本批无点迹记录，仅画滤波反算斜距\n"
                  f"斜距 {_rng_txt(vals, ' km')}",
              xy=(0.03, 0.04), va="bottom")
    style_axes(ax)
    title_block(fig, f"图 {num}  {target_title(tid)} 雷达–目标斜距时序",
                f"量测斜距取雷达点迹输出（radar_ext.plot），滤波斜距由 ECEF 航迹"
                f"与站址反算；本图只画目标 {int(tid)}")
    return fig


def fig46(D):
    return _range(D, 0, "46")


def fig47(D):
    return _range(D, 1, "47")


# ---------------------------------------------------------------- 图 48–49
@needs(("radar_output", "kf", "跟踪滤波功能测试"),
       title="图 48–49 ECEF 坐标时序")
def _ecef(D, rank, num):
    tid, g = _slot_target(D, rank)
    if tid is None:
        return empty_fig(f"图 {num}  ECEF 坐标时序",
                         f"本批日志只解析到 {len(by_target(D['radar_output']['kf']))} 个目标，"
                         f"没有第 {rank + 1} 个目标")
    t = g["t"].to_numpy(float)
    x = g["x"].to_numpy(float) / 1e3
    y = g["y"].to_numpy(float) / 1e3
    z = g["z"].to_numpy(float) / 1e3
    col = color_map([tid])[tid]

    fig, ax = plt.subplots(figsize=(11.4, 5.2))
    ax.plot(t, x, color=col, lw=1.6, label=f"x（{_rng_txt(x, ' km')}）")
    ax.plot(t, y, color=C["meas"], lw=1.6, label=f"y（{_rng_txt(y, ' km')}）")
    ax.plot(t, z, color=C["filter"], lw=1.6, label=f"z（{_rng_txt(z, ' km')}）")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("ECEF 坐标 (km)")
    ax.set_title("(a) ECEF 三分量–时间", fontsize=10.5)
    ax.legend(loc="center right", fontsize=8.2)

    # x/y/z 共用同一纵轴，上下各留 8% 余量，避免曲线顶到面板标题、也避免被图例压住。
    lo = float(np.nanmin([x.min(), y.min(), z.min()]))
    hi = float(np.nanmax([x.max(), y.max(), z.max()]))
    span = max(hi - lo, 1.0)
    ax.set_ylim(lo - 0.08 * span, hi + 0.08 * span)

    rr = np.hypot(np.hypot(x, y), z)
    _note(ax, f"{target_title(tid)}；{len(g)} 条滤波记录，时间 {_rng_txt(t, ' s')}\n"
              f"地心距 |r| {_rng_txt(rr, ' km')}；首末 {rr[0]:.0f} → {rr[-1]:.0f} km")
    style_axes(ax)
    title_block(fig, f"图 {num}  {target_title(tid)} 目标 ECEF 坐标时序",
                f"坐标取跟踪滤波输出的 ECEF 位置（radar_output.kf）；本图只画目标 "
                f"{int(tid)}")
    return fig


def fig48(D):
    return _ecef(D, 0, "48")


def fig49(D):
    return _ecef(D, 1, "49")


# ---------------------------------------------------------------- 图 50–54
def _wrap180(d):
    """把角度差折算到 [-180, 180)，用于“相对星下点经度”。"""
    return (np.asarray(d, dtype=float) + 180.0) % 360.0 - 180.0


def _parse_quad(text):
    """'lat,lon;...' → [(lon, lat) × 4]；含 miss 时返回 None。"""
    pts = []
    for tok in str(text).split(";"):
        parts = tok.split(",")
        if len(parts) != 2:
            return None
        try:
            lat, lon = float(parts[0]), float(parts[1])
        except ValueError:
            return None
        pts.append((lon, lat))
    return pts if len(pts) == 4 else None


@needs(("sat", "pos_err", "卫星自身定位误差功能测试"),
       ("sat", "wfov", "宽视场扫描探测功能测试"),
       title="图 50–54 星下点与宽场扫描轨迹")
def _sat_track(D, rank, num):
    pos = D["sat"]["pos_err"]
    wf = D["sat"]["wfov"]
    sats = sorted(int(v) for v in pos["sat"].dropna().unique())
    if rank >= len(sats):
        return empty_fig(f"图 {num}  星下点与宽场扫描轨迹",
                         f"本批日志只解析到 {len(sats)} 颗卫星，没有第 {rank + 1} 颗")
    sat = sats[rank]
    g = pos[pos["sat"].to_numpy(float) == sat].sort_values("t")
    w = wf[wf["sat"].to_numpy(float) == sat].sort_values("t")

    t_all = g["t"].to_numpy(float)
    lat, lon, h = ecef_to_lla(g["ap0"].to_numpy(float), g["ap1"].to_numpy(float),
                              g["ap2"].to_numpy(float))
    # 首拍是轨道初值（本批 t=1 只有 300 km），与 GEO 稳态差两个量级：它的星下点和
    # 宽场四角落在地球另一侧，混进来会把坐标范围拉到 ±170°，所以整帧剔除。
    steady = h > 6.0e6
    if not np.any(steady):
        steady = np.ones(h.shape, dtype=bool)
    init = ~steady
    t0 = float(t_all[steady].min())
    w = w[w["t"].to_numpy(float) >= t0]

    lat_s, lon_s = lat[steady], lon[steady]
    lon_ref = float(np.nanmedian(lon_s))
    lat_ref = float(np.nanmedian(lat_s))
    # 经度取“相对星下点”，GEO 全盘覆盖跨 ±180° 时轨迹才不会断成两段。
    dlon_s = _wrap180(lon_s - lon_ref)

    ok = w["center"].astype(str).str.contains(",", regex=False)
    c = w[ok]
    clat = c["center"].str.split(",").str[0].astype(float).to_numpy()
    dlon_c = _wrap180(c["center"].str.split(",").str[1].astype(float).to_numpy() - lon_ref)
    ct = c["t"].to_numpy(float)

    quads = []
    for row in w.itertuples():
        q = _parse_quad(row.quad)
        if q is not None:
            quads.append((float(row.t),
                          [(_wrap180(lo_ - lon_ref), la_) for lo_, la_ in q]))

    col = color_map(sats)[sat]
    cmap = plt.get_cmap("viridis")
    norm = plt.Normalize(float(np.nanmin(ct)), float(np.nanmax(ct))) if len(ct) \
        else plt.Normalize(0.0, 1.0)

    fig, ax = plt.subplots(figsize=(11.8, 6.2))
    if len(c):
        ax.scatter(dlon_c, clat, c=ct, s=6, cmap=cmap, norm=norm, alpha=0.70,
                   linewidths=0, label=f"扫描单元中心（{len(c)} 个）", zorder=2)

    # 宽视场四角是整幅覆盖范围（GEO 下几乎覆盖整个可见半球），只画轮廓并按时间
    # 着色；不再填充，否则十个面叠在一起糊成一片。
    pick = []
    if quads:
        pick = np.unique(np.linspace(0, len(quads) - 1, 10).round().astype(int))
        for idx in pick:
            tq, q = quads[idx]
            ax.plot([p[0] for p in q] + [q[0][0]],
                    [p[1] for p in q] + [q[0][1]],
                    color=cmap(norm(tq)), lw=1.0, alpha=0.85, zorder=1)

    ax.plot(dlon_s, lat_s, "-o", ms=2.2, mew=0, lw=1.2, color=col, zorder=4,
            label=f"星下点（{int(steady.sum())} 拍，GEO 近似静止）")

    handles, labels = ax.get_legend_handles_labels()
    if quads:
        handles.append(Line2D([], [], color="#55595e", lw=1.1))
        labels.append(f"宽场覆盖轮廓（有效 {len(quads)} 个，抽 {len(pick)} 个）")
    ax.legend(handles, labels, loc="best", fontsize=7.6, markerscale=1.8)

    ax.set_xlabel("相对星下点经度 Δλ (°)")
    ax.set_ylabel("纬度 (°)")
    ax.set_title("(a) 星下点与宽场瞬时覆盖（颜色＝时间）", fontsize=10.5)

    xs = list(dlon_s) + list(dlon_c)
    ys = list(lat_s) + list(clat)
    for _, q in quads:
        xs.extend(p[0] for p in q)
        ys.extend(p[1] for p in q)
    x0, x1 = float(np.nanmin(xs)), float(np.nanmax(xs))
    y0, y1 = float(np.nanmin(ys)), float(np.nanmax(ys))
    pad_x = max(0.05 * (x1 - x0), 1.0)
    pad_y = max(0.05 * (y1 - y0), 1.0)
    ax.set_xlim(x0 - pad_x, x1 + pad_x)
    ax.set_ylim(y0 - pad_y, y1 + pad_y)

    cb = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, pad=0.015)
    cb.set_label("仿真时间 (s)", fontsize=8.5)
    cb.ax.tick_params(labelsize=7.5)

    ns = "N" if lat_ref >= 0 else "S"
    ew = "E" if lon_ref >= 0 else "W"
    _note(ax, f"卫星 {sat}；共 {len(g)} 拍（稳态 {int(steady.sum())} 拍），"
              f"{_rng_txt(t_all, ' s')}\n"
              f"星下点 {abs(lat_ref):.2f}°{ns}、{abs(lon_ref):.2f}°{ew}；"
              f"纬度漂移 {_rng_txt(lat_s, '°', 2)}\n"
              f"高度中位 {np.nanmedian(h[steady])/1e3:.0f} km（GEO 稳态），"
              f"星下点经度漂移 {_rng_txt(dlon_s, '°', 3)}")

    init_note = ""
    if np.any(init):
        init_note = (f"t={t_all[init].min():.0f} s 的轨道初值帧（高度 "
                     f"{h[init].min()/1e3:.0f} km）与 GEO 稳态不连续，未画入；")
    title_block(fig, f"图 {num}  卫星 {sat} 星下点轨迹与宽场扫描范围",
                f"扫描单元 {len(w)} 条记录（有效中心 {len(c)} 条、有效四角 {len(quads)} 条）；"
                f"横轴为相对星下点经度；{init_note}本图只画卫星 {sat}")
    style_axes(ax)
    return fig


def fig50(D):
    return _sat_track(D, 0, "50")


def fig51(D):
    return _sat_track(D, 1, "51")


def fig52(D):
    return _sat_track(D, 2, "52")


def fig53(D):
    return _sat_track(D, 3, "53")


def fig54(D):
    return _sat_track(D, 4, "54")


FIGURES = [
    ("44", "fig44_geo_altitude_tg1", "图 44 目标探测高度时序（目标序位 1）", fig44),
    ("45", "fig45_geo_altitude_tg2", "图 45 目标探测高度时序（目标序位 2）", fig45),
    ("46", "fig46_geo_range_tg1", "图 46 雷达–目标斜距时序（目标序位 1）", fig46),
    ("47", "fig47_geo_range_tg2", "图 47 雷达–目标斜距时序（目标序位 2）", fig47),
    ("48", "fig48_geo_ecef_tg1", "图 48 目标 ECEF 坐标时序（目标序位 1）", fig48),
    ("49", "fig49_geo_ecef_tg2", "图 49 目标 ECEF 坐标时序（目标序位 2）", fig49),
    ("50", "fig50_geo_sat_track1", "图 50 星下点与宽场扫描轨迹（卫星序位 1）", fig50),
    ("51", "fig51_geo_sat_track2", "图 51 星下点与宽场扫描轨迹（卫星序位 2）", fig51),
    ("52", "fig52_geo_sat_track3", "图 52 星下点与宽场扫描轨迹（卫星序位 3）", fig52),
    ("53", "fig53_geo_sat_track4", "图 53 星下点与宽场扫描轨迹（卫星序位 4）", fig53),
    ("54", "fig54_geo_sat_track5", "图 54 星下点与宽场扫描轨迹（卫星序位 5）", fig54),
]
