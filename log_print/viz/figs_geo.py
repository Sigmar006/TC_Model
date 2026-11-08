# -*- coding: utf-8 -*-
"""图 44–54：雷达目标几何时序、卫星星下点与宽场扫描轨迹。

坐标口径
--------
* 高度、斜距、ECEF 分量都是千米级长度量，量级集中在 10³–10⁴ km，按 km 线性绘制，
  不转 dB（与图 40 的轨道高度同口径）；dB 只用于跨数量级的量。
* 一张图只画一个实体：44–49 按目标序位分幅，50–54 按卫星序位分幅。标题里的编号
  取自当次日志，序位只是图号占位；本批不足该序位时改画说明卡，不臆造曲线。
* 图 44–49 每条量都画三份：场景真值（`radar_ext.scene` 的 `位置ENU`，源码注释
  写明为场景真值）、点迹量测（`radar_ext.plot` 的斜距/方位/俯仰，高度用关联行的
  LLA）、滤波航迹（`radar_output.kf` 的 ECEF）。三者按 (仿真时间, 目标ID) 对齐；
  真值不外推，只截到与滤波同段的时间窗；滤波航迹裁到该目标的观测窗（点迹首末
  时刻），观测停止后的外推段默认不画，`--show-extrapolated` 可关闭裁剪；
  量测点少于滤波点时按点打标记。本批点迹由场景 ENU 直接换算，量测与真值基本重合；
  残差不再在图内另起注释块，裁剪说明并入副标题，免得压住曲线。
* “探测高度”取航迹关联/落点预测同源的 LLA 高度（`assoc.h`），并与滤波航迹
  ECEF→LLA 的高度并列，两者同源不同环节，便于核对；“运动特征”里的 `高度`
  是站心 ENU 的 u 分量（= R·sin el），不是海拔高度，不在此图混用。
* 卫星图为 GEO（本批 103–107），星下点漂移 < 0.1°，轨迹缩成一个点，故以标记点
  表示，不把首拍与稳态连成跨半球的长线；经度用“相对星下点经度”，避免全盘覆盖
  跨 ±180° 时断成两段。
  `wfov.quad` 是“视场矩形 ∩ 地球视盘”的经纬度包络框（源码
  `SbirsPipeline.cpp` ComputeGroundCoverageEnvelope），四角顺序为西北/东北/
  东南/西南，故按经纬度矩形连线；t=1 的轨道初值帧（300 km）与稳态不连续，剔除。
  该星实际观测到的目标（`ir_ext.ir_obs`，退回 `ir_int.ir_trk`）按目标画出地面
  轨迹，位置取场景真值；扫描单元中心只是波位、不等于目标位置，不再单独画。
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from .common import (C, by_target, color_map, ecef_to_lla, empty_fig, enu_to_ecef,
                     finish, needs, obs_clip, obs_note, radar_obs_span, style_axes,
                     target_title, title_block)


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


# ---------------------------------------------------------------- 真值 / 量测
def _scene_truth(D, tid):
    """场景真值：`radar_ext.scene` 的 `位置ENU` → ECEF / LLA / 斜距，统一换成 km。

    返回按 t 升序的 dict（t/x/y/z/h/rng）；本批没有该目标的场景行时返回 None。
    """
    ext = D.get("radar_ext") or {}
    sc = ext.get("scene")
    if sc is None or len(sc) < 2 or "tid" not in sc.columns:
        return None
    s = sc[sc["tid"] == tid].sort_values("t")
    if len(s) < 2:
        return None
    e0 = s["e0"].to_numpy(float)
    e1 = s["e1"].to_numpy(float)
    e2 = s["e2"].to_numpy(float)
    x, y, z = enu_to_ecef(e0, e1, e2)
    _, _, h = ecef_to_lla(x, y, z)
    return {"t": s["t"].to_numpy(float), "x": x / 1e3, "y": y / 1e3,
            "z": z / 1e3, "h": h / 1e3,
            "rng": np.hypot(np.hypot(e0, e1), e2) / 1e3}


def _scene_lla(D, tid, t0, t1):
    """场景真值的地面位置：ENU→ECEF→LLA，只取观测时段 [t0, t1]。"""
    tr = _scene_truth(D, tid)
    if tr is None:
        return None
    m = (tr["t"] >= t0) & (tr["t"] <= t1)
    if not np.any(m):
        return None
    lat, lon, h = ecef_to_lla(tr["x"][m] * 1e3, tr["y"][m] * 1e3, tr["z"][m] * 1e3)
    return {"t": tr["t"][m], "lat": lat, "lon": lon, "h": h}


def _truth_window(tr, lo, hi):
    """把真值截到 [lo, hi]：与滤波同段才好逐点比对，也不让发射段压扁纵轴。"""
    if tr is None:
        return None
    m = (tr["t"] >= lo) & (tr["t"] <= hi)
    if not np.any(m):
        return None
    return {k: v[m] for k, v in tr.items()}


def _plot_ecef_km(p):
    """点迹（斜距/方位/俯仰）→ ENU → ECEF（km）。

    方位口径与源码 `TryLookPolarFromEnuM` 一致：az = atan2(北, 东)，即自东向逆时针，
    故 e = r·cos(el)·cos(az)、n = r·cos(el)·sin(az)、u = r·sin(el)。
    """
    r = p["rng"].to_numpy(float)
    az = np.radians(p["az"].to_numpy(float))
    el = np.radians(p["el"].to_numpy(float))
    x, y, z = enu_to_ecef(r * np.cos(el) * np.cos(az),
                          r * np.cos(el) * np.sin(az),
                          r * np.sin(el))
    return x / 1e3, y / 1e3, z / 1e3


# ---------------------------------------------------------------- 图 44–45
@needs(("radar_output", "kf", "跟踪滤波功能测试"),
       ("radar_ext", "scene", "目标场景接口测试"),
       title="图 44–45 探测高度时序")
def _altitude(D, rank, num):
    tid, g = _slot_target(D, rank)
    if tid is None:
        return empty_fig(f"图 {num}  探测高度时序",
                         f"本批日志只解析到 {len(by_target(D['radar_output']['kf']))} 个目标，"
                         f"没有第 {rank + 1} 个目标")
    t = g["t"].to_numpy(float)
    _, _, hk_all = _kf_lla(g)
    t_lo, t_hi = radar_obs_span(D, tid)
    keep, n_cut = obs_clip(t, t_lo, t_hi)
    if not keep.any():
        keep, n_cut = np.ones(t.shape, dtype=bool), 0
    t = t[keep]
    hk = hk_all[keep] / 1e3
    tr = _scene_truth(D, tid)
    tw = _truth_window(tr, float(t.min()), float(t.max()))
    assoc = D["radar_output"]["assoc"]
    a = assoc[assoc["tid"] == tid].sort_values("t") if "tid" in assoc.columns \
        else assoc.iloc[0:0]
    ta = a["t"].to_numpy(float)
    ha = a["h"].to_numpy(float) / 1e3

    fig, ax = plt.subplots(figsize=(11.4, 5.2))
    if tw is not None:
        ax.plot(tw["t"], tw["h"], color=C["truth"], lw=3.2, alpha=0.45,
                solid_capstyle="round", zorder=2, label="场景真值")
    ax.plot(t, hk, color=C["filter"], lw=1.6, zorder=3, label="滤波航迹")
    if len(a):
        ax.plot(ta, ha, "o", ms=3.0, mew=0, color=C["meas"], alpha=0.85, zorder=4,
                label="点迹量测")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("海拔高度 (km)")
    ax.set_title("(a) 目标高度–时间（真值 / 量测 / 滤波）", fontsize=10.5)
    ax.legend(loc="best", fontsize=8.2)

    vals = [hk]
    if tw is not None:
        vals.append(tw["h"])
    if len(a):
        vals.append(ha)
    lo = float(np.nanmin([v.min() for v in vals]))
    hi = float(np.nanmax([v.max() for v in vals]))
    span = max(hi - lo, 1e-6)
    ax.set_ylim(lo - 0.10 * span, hi + 0.18 * span)

    style_axes(ax)
    clip_txt = obs_note(n_cut, t_hi, "滤波航迹")
    title_block(fig, f"图 {num}  {target_title(tid)} 探测高度时序",
                f"真值取场景接口的 位置ENU（radar_ext.scene），量测取关联 LLA，"
                f"滤波取 ECEF 航迹；{len(t)} 条滤波记录、{len(a)} 条量测记录，"
                f"本图只画目标 {int(tid)}"
                + (f"；{clip_txt}" if clip_txt else ""))
    return fig


def fig44(D):
    return _altitude(D, 0, "44")


def fig45(D):
    return _altitude(D, 1, "45")


# ---------------------------------------------------------------- 图 46–47
@needs(("radar_ext", "plot", "雷达对外目标信息输出接口测试-点迹量测"),
       ("radar_ext", "scene", "目标场景接口测试"),
       ("radar_output", "kf", "跟踪滤波功能测试"),
       title="图 46–47 斜距时序")
def _range(D, rank, num):
    tid, g = _slot_target(D, rank)
    if tid is None:
        return empty_fig(f"图 {num}  斜距时序",
                         f"本批日志只解析到 {len(by_target(D['radar_output']['kf']))} 个目标，"
                         f"没有第 {rank + 1} 个目标")
    t = g["t"].to_numpy(float)
    plot = D["radar_ext"]["plot"]
    p = plot[plot["tid"] == tid].sort_values("t") if "tid" in plot.columns \
        else plot.sort_values("t")
    rng_all = _kf_range_km(g)
    t_lo, t_hi = radar_obs_span(D, tid)
    keep, n_cut = obs_clip(t, t_lo, t_hi)
    if not keep.any():
        keep, n_cut = np.ones(t.shape, dtype=bool), 0
    t = t[keep]
    rng_kf = rng_all[keep]
    tr = _scene_truth(D, tid)
    tw = _truth_window(tr, float(t.min()), float(t.max()))
    tp = p["t"].to_numpy(float)
    rp = p["rng"].to_numpy(float) / 1e3

    fig, ax = plt.subplots(figsize=(11.4, 5.2))
    if tw is not None:
        ax.plot(tw["t"], tw["rng"], color=C["truth"], lw=3.2, alpha=0.45,
                solid_capstyle="round", zorder=2, label="场景真值")
    ax.plot(t, rng_kf, color=C["filter"], lw=1.5, ls="--", zorder=3,
            label="滤波航迹反算斜距")
    if len(p):
        ax.plot(tp, rp, "-o", ms=2.6, mew=0, lw=1.2, color=C["meas"], alpha=0.9,
                zorder=4, label="点迹量测斜距")
    ax.set_xlabel("仿真时间 (s)")
    ax.set_ylabel("雷达–目标斜距 (km)")
    ax.set_title("(a) 斜距–时间（真值 / 量测 / 滤波）", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.2)

    vals = [rng_kf]
    if tw is not None:
        vals.append(tw["rng"])
    if len(p):
        vals.append(rp)
    lo = float(np.nanmin([v.min() for v in vals]))
    hi = float(np.nanmax([v.max() for v in vals]))
    span = max(hi - lo, 1e-6)
    ax.set_ylim(lo - 0.08 * span, hi + 0.18 * span)

    style_axes(ax)
    clip_txt = obs_note(n_cut, t_hi, "滤波航迹")
    title_block(fig, f"图 {num}  {target_title(tid)} 雷达–目标斜距时序",
                f"真值取场景 位置ENU 的模，量测取雷达点迹输出（radar_ext.plot），"
                f"滤波斜距由 ECEF 航迹与站址反算；本图只画目标 {int(tid)}"
                + (f"；{clip_txt}" if clip_txt else ""))
    return fig


def fig46(D):
    return _range(D, 0, "46")


def fig47(D):
    return _range(D, 1, "47")


# ---------------------------------------------------------------- 图 48–49
@needs(("radar_output", "kf", "跟踪滤波功能测试"),
       ("radar_ext", "scene", "目标场景接口测试"),
       title="图 48–49 ECEF 坐标时序")
def _ecef(D, rank, num):
    tid, g = _slot_target(D, rank)
    if tid is None:
        return empty_fig(f"图 {num}  ECEF 坐标时序",
                         f"本批日志只解析到 {len(by_target(D['radar_output']['kf']))} 个目标，"
                         f"没有第 {rank + 1} 个目标")
    t = g["t"].to_numpy(float)
    kf_all = {"x": g["x"].to_numpy(float) / 1e3, "y": g["y"].to_numpy(float) / 1e3,
              "z": g["z"].to_numpy(float) / 1e3}
    t_lo, t_hi = radar_obs_span(D, tid)
    keep, n_cut = obs_clip(t, t_lo, t_hi)
    if not keep.any():
        keep, n_cut = np.ones(t.shape, dtype=bool), 0
    t = t[keep]
    kf = {k: v[keep] for k, v in kf_all.items()}
    tr = _scene_truth(D, tid)
    tw = _truth_window(tr, float(t.min()), float(t.max()))

    ext = D.get("radar_ext") or {}
    plot = ext.get("plot")
    p = plot[plot["tid"] == tid].sort_values("t") if plot is not None \
        and "tid" in plot.columns else plot
    if p is not None and len(p):
        tp = p["t"].to_numpy(float)
        mx, my, mz = _plot_ecef_km(p)
        meas = {"x": mx, "y": my, "z": mz}
    else:
        tp = np.empty(0)
        meas = {k: np.empty(0) for k in "xyz"}

    fig, axs = plt.subplots(3, 1, figsize=(11.4, 8.8), sharex=True)
    for ax, tag, key, name in zip(axs, "abc", "xyz", ("x", "y", "z")):
        if tw is not None:
            ax.plot(tw["t"], tw[key], color=C["truth"], lw=3.2, alpha=0.45,
                    solid_capstyle="round", zorder=2, label="场景真值")
        ax.plot(t, kf[key], color=C["filter"], lw=1.5, zorder=3, label="滤波航迹")
        if len(tp):
            ax.plot(tp, meas[key], "o", ms=2.6, mew=0, color=C["meas"], alpha=0.85,
                    zorder=4, label="点迹量测")
        ax.set_ylabel(f"{name} (km)")
        ax.set_title(f"({tag}) {name} 分量（真值 / 量测 / 滤波）", fontsize=10.5)

        vals = [kf[key]]
        if tw is not None:
            vals.append(tw[key])
        if len(tp):
            vals.append(meas[key])
        lo = float(np.nanmin([v.min() for v in vals]))
        hi = float(np.nanmax([v.max() for v in vals]))
        span = max(hi - lo, 1.0)
        ax.set_ylim(lo - 0.08 * span, hi + 0.13 * span)
        style_axes(ax)

    axs[0].legend(loc="best", fontsize=8.2)
    axs[-1].set_xlabel("仿真时间 (s)")

    clip_txt = obs_note(n_cut, t_hi, "滤波航迹")
    title_block(fig, f"图 {num}  {target_title(tid)} 目标 ECEF 坐标时序",
                f"真值取场景 位置ENU 经站心→ECEF，量测由点迹斜距/方位/俯仰反算 ENU 后转 "
                f"ECEF，滤波取跟踪输出（radar_output.kf）；本图只画目标 {int(tid)}"
                + (f"；{clip_txt}" if clip_txt else ""))
    finish(fig)
    return fig


def fig48(D):
    return _ecef(D, 0, "48")


def fig49(D):
    return _ecef(D, 1, "49")


# ---------------------------------------------------------------- 图 50–54
def _wrap180(d):
    """把角度差折算到 [-180, 180)，用于“相对星下点经度”。"""
    return (np.asarray(d, dtype=float) + 180.0) % 360.0 - 180.0


def _segments(x, y, jump=90.0):
    """按经度环绕处断线：相邻点 Δx 超过 jump 就是跨了 ±180°，拆段后分别画，
    免得轨迹被连成一条横贯全图的长线。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 2:
        return [(x, y)]
    cut = np.flatnonzero(np.abs(np.diff(x)) > jump) + 1
    return list(zip(np.split(x, cut), np.split(y, cut)))


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

    # 该星实际观测到的目标：`ir_obs`（退回 `ir_trk`）给出（卫星, 目标）的观测时段，
    # 目标地面位置取场景真值（radar_ext.scene → LLA）。扫描单元中心只是波位，
    # 与目标真实位置不是一回事，不再单独画。
    tgt = []
    obs = (D.get("ir_ext") or {}).get("ir_obs")
    if obs is None or len(obs) == 0:
        obs = (D.get("ir_int") or {}).get("ir_trk")
    if obs is not None and len(obs) and {"sat", "tid"} <= set(obs.columns):
        o = obs[obs["sat"].to_numpy(float) == sat]
        for tid, og in o.groupby("tid"):
            t_o = og["t"].to_numpy(float)
            lo_t, hi_t = float(np.nanmin(t_o)), float(np.nanmax(t_o))
            tgt.append(dict(tid=int(tid), t0=lo_t, t1=hi_t, n=len(og),
                            trk=_scene_lla(D, int(tid), lo_t, hi_t)))

    quads = []
    for row in w.itertuples():
        q = _parse_quad(row.quad)
        if q is not None:
            quads.append((float(row.t),
                          [(_wrap180(lo_ - lon_ref), la_) for lo_, la_ in q]))

    col = color_map(sats)[sat]
    cmap = plt.get_cmap("viridis")
    qt = np.asarray([tq for tq, _ in quads], dtype=float)
    norm = plt.Normalize(float(qt.min()), float(qt.max())) if len(qt) \
        else plt.Normalize(0.0, 1.0)

    fig, ax = plt.subplots(figsize=(11.8, 6.2))
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
            label="星下点")

    # 目标地面轨迹按目标配色（与其它图同号同色），首点用空心圈标出首次观测位置。
    tcol = color_map(list(sats) + [o["tid"] for o in tgt])
    for o in tgt:
        trk = o["trk"]
        if trk is None:
            continue
        dl = _wrap180(trk["lon"] - lon_ref)
        for i, (sx, sy) in enumerate(_segments(dl, trk["lat"])):
            ax.plot(sx, sy, "-", lw=1.8, color=tcol[o["tid"]], zorder=5,
                    label=f"目标 {o['tid']} 地面轨迹" if i == 0 else None)
        ax.plot(dl[:1], trk["lat"][:1], "o", ms=5.5, mfc="none", mew=1.3,
                color=tcol[o["tid"]], zorder=6)

    handles, labels = ax.get_legend_handles_labels()
    if quads:
        handles.append(Line2D([], [], color="#55595e", lw=1.1))
        labels.append("宽场覆盖轮廓")
    ax.legend(handles, labels, loc="best", fontsize=7.6, markerscale=1.8)

    ax.set_xlabel("相对星下点经度 Δλ (°)")
    ax.set_ylabel("纬度 (°)")
    ax.set_title("(a) 星下点与宽场瞬时覆盖（颜色＝时间）", fontsize=10.5)

    xs = list(dlon_s)
    ys = list(lat_s)
    for _, q in quads:
        xs.extend(p[0] for p in q)
        ys.extend(p[1] for p in q)
    for o in tgt:
        if o["trk"] is None:
            continue
        xs.extend(_wrap180(o["trk"]["lon"] - lon_ref).tolist())
        ys.extend(np.asarray(o["trk"]["lat"], dtype=float).tolist())
    x0, x1 = float(np.nanmin(xs)), float(np.nanmax(xs))
    y0, y1 = float(np.nanmin(ys)), float(np.nanmax(ys))
    pad_x = max(0.05 * (x1 - x0), 1.0)
    pad_y = max(0.05 * (y1 - y0), 1.0)
    ax.set_xlim(x0 - pad_x, x1 + pad_x)
    ax.set_ylim(y0 - pad_y, y1 + pad_y)

    if quads:
        cb = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax,
                          pad=0.015)
        cb.set_label("仿真时间 (s)", fontsize=8.5)
        cb.ax.tick_params(labelsize=7.5)

    ns = "N" if lat_ref >= 0 else "S"
    ew = "E" if lon_ref >= 0 else "W"
    note = [f"卫星 {sat}；共 {len(g)} 拍（稳态 {int(steady.sum())} 拍），"
            f"{_rng_txt(t_all, ' s')}",
            f"星下点 {abs(lat_ref):.2f}°{ns}、{abs(lon_ref):.2f}°{ew}；"
            f"纬度漂移 {_rng_txt(lat_s, '°', 2)}",
            f"高度中位 {np.nanmedian(h[steady])/1e3:.0f} km（GEO 稳态），"
            f"星下点经度漂移 {_rng_txt(dlon_s, '°', 3)}"]
    if tgt:
        note.append("观测目标 " + "、".join(
            f"{o['tid']}（{o['t0']:.0f}–{o['t1']:.0f} s）" for o in tgt))
    # 卫星注释放到图脚：星下点与目标轨迹常压在上半区，放图内会盖住曲线。
    fig.text(0.012, 0.008, "\n".join(note), fontsize=8.0, ha="left", va="bottom",
             color="#3a4550", linespacing=1.45)

    if tgt:
        tgt_txt = (f"；观测目标 {'、'.join(str(o['tid']) for o in tgt)}，"
                   f"地面位置取场景真值（radar_ext.scene）")
    else:
        tgt_txt = "；本批该星未观测到目标"
    init_note = ""
    if np.any(init):
        init_note = (f"t={t_all[init].min():.0f} s 的轨道初值帧（高度 "
                     f"{h[init].min()/1e3:.0f} km）与 GEO 稳态不连续，未画入；")
    title_block(fig, f"图 {num}  卫星 {sat} 星下点轨迹与宽场扫描范围",
                f"宽场覆盖 {len(w)} 帧（有效四角 {len(quads)} 帧）；"
                f"横轴为相对星下点经度{tgt_txt}；{init_note}本图只画卫星 {sat}")
    style_axes(ax)
    finish(fig, bottom=0.105)
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
