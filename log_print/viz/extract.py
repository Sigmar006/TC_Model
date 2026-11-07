# -*- coding: utf-8 -*-
"""日志解析：每个文件只扫一遍，结果缓存为 pickle。

实现要点
--------
1. 每个日志文件一次顺序读取，按"消息类型"分派到对应模式；
2. 所有正则统一用 **命名分组**，解析时只取 `groupdict()`，
   从根本上避免"第几个括号"这类错位错误；
3. 结果按（文件名, 大小, 修改时间, 版本号）缓存到 viz/.cache。
"""
from __future__ import annotations

import pickle
import re
import time
from pathlib import Path

import numpy as np
import pandas as pd

from . import common

CACHE_VERSION = 11


# ------------------------------------------------------------------ 读取
# 日志可能按 GBK/GB18030 落盘（中文 Windows 的默认编码）。这里按内容探测编码，
# 避免内网日志不是 UTF-8 时整份解析为空、程序却不报错。
_ENC: dict[str, str] = {}
_SAMPLE = 16 << 20


def _penalty(raw: bytes, enc: str) -> float:
    """按解码错误密度打分：0 表示干净，接近 0.3 表示整段乱码。"""
    try:
        raw.decode(enc)
        return 0.0
    except UnicodeDecodeError:
        pass
    txt = raw.decode(enc, errors="replace")
    return txt.count("\ufffd") / max(len(txt), 1)


def detect_encoding(path: Path) -> str:
    key = str(path)
    hit = _ENC.get(key)
    if hit:
        return hit
    with path.open("rb") as fh:
        head = fh.read(_SAMPLE)
    if head.startswith(b"\xef\xbb\xbf"):
        enc = "utf-8-sig"
    else:
        cut = len(head)
        while len(head) - cut < 4 and cut > 0 and head[cut - 1] > 0x7F:
            cut -= 1                                 # 去掉采样末尾被截断的多字节
        probe = head[:cut]
        u = _penalty(probe, "utf-8")
        # 实测：真 GBK 日志按 UTF-8 解码的错误密度 ≈0.32，带少量坏字节的 UTF-8 ≈0.02。
        # 因此只有"按 UTF-8 几乎解不动"时才改判 GB18030，宁可少量替换字符也不整篇乱码。
        if u < 0.1 or not any(b > 0x7F for b in probe):
            enc = "utf-8"
        else:
            enc = "gb18030" if _penalty(probe, "gb18030") < u else "utf-8"
    _ENC[key] = enc
    return enc


def open_text(path: Path):
    """按探测到的编码打开日志。"""
    return path.open("r", encoding=detect_encoding(path), errors="replace")


def resolve_log(fname: str) -> Path:
    """在日志目录（含子目录）里找同名文件，内网按日期/批次分子目录时同样可用。"""
    base = Path(common.LOG_DIR)
    direct = base / fname
    if direct.is_file():
        return direct
    hits = [q for q in base.rglob(fname) if q.is_file()] if base.is_dir() else []
    if not hits:
        raise FileNotFoundError(
            f"日志目录里找不到 {fname}：{base}\n"
            f"  请把日志放入该目录，或用 --log-dir / 环境变量 LOG_PRINT_LOG_DIR 指定。")
    if len(hits) > 1:
        hits.sort(key=lambda q: q.stat().st_mtime, reverse=True)
        print(f"    注意：{fname} 有 {len(hits)} 份，取最新一份 {hits[0]}")
    return hits[0]

HEAD = re.compile(r"^仿真时间=([\d.]+)s 仿真周期=(\d+) \[([^\]]+)\] ")

_N = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"


# ------------------------------------------------------------------ 模式构造
def f(label: str, name: str) -> str:
    """数值标量字段：  名称=123.4"""
    return rf"{re.escape(label)}=(?P<{name}>{_N})"


def s(label: str, name: str, pat: str = r"\S+") -> str:
    """文本字段。"""
    return rf"{re.escape(label)}=(?P<{name}>{pat})"


def v3(label: str, name: str) -> str:
    """三元组字段：  名称=(1,2,3)"""
    return (rf"{re.escape(label)}=\((?P<{name}0>{_N}),(?P<{name}1>{_N}),"
            rf"(?P<{name}2>{_N})\)")


def lst(label: str, name: str) -> str:
    """方括号列表字段： 名称=[a;b;c]"""
    return rf"{re.escape(label)}=\[(?P<{name}>[^\]]*)\]"


def fc(name: str) -> str:
    """特征置信度字段（2026-11-08 由“综合识别加权贡献”改名，老日志仍要认）。"""
    return rf"(?:综合识别加权贡献|置信度)=(?P<{name}>{_N})"


def J(*parts: str) -> re.Pattern:
    return re.compile(" ".join(parts))


def conv(g: dict, *extra: dict) -> dict:
    """把 groupdict 转成数值/文本混合的 dict。"""
    out: dict = {}
    for k, v in g.items():
        if v is None:
            continue
        try:
            out[k] = float(v)
        except (TypeError, ValueError):
            out[k] = v
    for e in extra:
        out.update(e)
    return out


class Rows:
    def __init__(self) -> None:
        self.d: dict[str, list[dict]] = {}

    def add(self, name: str, row: dict) -> None:
        self.d.setdefault(name, []).append(row)

    def frames(self) -> dict[str, pd.DataFrame]:
        return {k: pd.DataFrame(v) for k, v in self.d.items() if v}


# ================================================================== 雷达输出
P_BEAM = J(
    r"雷达ID=(?P<rid>\d+)",
    s("驻留种类", "kind"),
    rf"波位序号=(?P<seq>\d+)",
    rf"指向=\((?P<az>{_N}),(?P<el>{_N})\)",
)
P_BEAM_NOSEQ = J(
    r"雷达ID=(?P<rid>\d+)",
    s("驻留种类", "kind"),
    rf"指向=\((?P<az>{_N}),(?P<el>{_N})\)",
)
P_BEAMSUM = J(r"雷达ID=(?P<rid>\d+)", rf"本周期搜索驻留数=(?P<n>\d+)")
P_SEARCH = J(
    r"雷达ID=(?P<rid>\d+)",
    rf"搜索=(?P<n_search>\d+)",
    rf"指定=(?P<n_desig>\d+)",
    rf"驻留=(?P<n_dwell>\d+)",
    rf"预算/已耗时=(?P<budget>{_N})/(?P<used>{_N})s",
)
P_SCHED = re.compile(
    r"\[搜索数(?P<n_search>\d+),指定数(?P<n_desig>\d+),外部跟踪数(?P<n_ext>\d+),"
    r"库内跟踪数(?P<n_lib>\d+),确认航迹数(?P<n_conf>\d+)\]")
P_JAMPOW = J(
    rf"干扰源ID=(?P<src>\d+)",
    f("到达雷达功率", "power") + "W",
    f("到达功率密度", "pden") + r"W/m\^2",
)
P_GAIN1 = J(r"目标ID=(?P<tid>\d+)", f("脉压增益", "pc") + "dB")
P_GAIN2 = J(r"目标ID=(?P<tid>\d+)", f("MTI增益", "mti") + "dB")
P_GAIN3 = J(r"目标ID=(?P<tid>\d+)", f("MTD增益", "mtd") + "dB")
P_GAIN4 = J(
    r"目标ID=(?P<tid>\d+)",
    f("处理后回波功率", "p_after") + "W",
    s("处理链路", "chain"),
    f("处理前回波功率", "p_before") + "W",
    f("回波功率", "p_echo") + "W",
)
P_NOISE1 = J(r"目标ID=(?P<tid>\d+)", f("脉冲压缩后的噪声功率", "p_pc") + "W")
P_NOISE2 = J(r"目标ID=(?P<tid>\d+)", lst("各多普勒滤波器通道噪声功率", "ch") + "W")
P_JG1 = J(r"目标ID=(?P<tid>\d+)", f("MTI处理后干扰剩余功率", "p_mti") + "W")
P_JG2 = J(r"目标ID=(?P<tid>\d+)", lst("MTD各多普勒通道干扰功率分布", "ch") + "W")
P_JG3 = J(r"目标ID=(?P<tid>\d+)", f("MTD处理后的干扰总抑制比", "suppr") + "dB")
P_ECHO = J(
    r"目标ID=(?P<tid>\d+)",
    f("斜距", "rng") + "m",
    f("目标RCS", "rcs") + r"m\^2",
    f("回波功率", "p_echo") + "W",
    f("噪声功率", "p_noise") + "W",
    f("干扰功率", "p_jam") + "W",
    f("回波功率密度", "p_den") + r"W/m\^2",
)
P_CFAR1 = J(r"目标ID=(?P<tid>\d+)", f("统计检测门限", "thr") + "dB")
P_CFAR2 = J(r"目标ID=(?P<tid>\d+)", f("信噪比", "snr") + "dB")
P_CFAR3 = J(r"目标ID=(?P<tid>\d+)",
            rf"检测概率=(?P<pd90>\d+)%的最大探测距离=(?P<r90>{_N})m",
            f("依据RCS", "rcs90") + r"m\^2")
P_RCSDET = J(
    r"目标ID=(?P<tid>\d+)",
    f("本周期RCS", "rcs") + r"m\^2",
    rf"定位信息=斜距=(?P<rng>{_N})m",
    rf"方位/俯仰=\((?P<az>{_N}),(?P<el>{_N})\)",
    s("来源", "src"),
)
P_RCSMOT = J(
    r"目标ID=(?P<tid>\d+)",
    rf"运动参数=速度=(?P<speed>{_N})m/s",
    rf"航向=(?P<heading>{_N})",
    rf"加速度=\((?P<acc0>{_N}),(?P<acc1>{_N}),(?P<acc2>{_N})\)m/s\^2",
    s("来源", "src"),
)
P_KF = J(
    rf"航迹=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    rf"位置（ECEF）=\((?P<x>{_N}),(?P<y>{_N}),(?P<z>{_N})\)m",
    rf"速度=\((?P<vx>{_N}),(?P<vy>{_N}),(?P<vz>{_N})\)m/s",
    rf"加速度=\((?P<ax>{_N}),(?P<ay>{_N}),(?P<az>{_N})\)m/s\^2",
)
P_MTT1 = J(
    rf"航迹=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    rf"定位信息=斜距=(?P<rng>{_N})m",
    rf"方位/俯仰=\((?P<az>{_N}),(?P<el>{_N})\)",
    s("来源", "src"),
)
P_MTT2 = J(
    rf"航迹=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    rf"运动参数=速度=(?P<speed>{_N})m/s",
    rf"航向=(?P<heading>{_N})",
    rf"加速度=\((?P<ax>{_N}),(?P<ay>{_N}),(?P<az>{_N})\)m/s\^2",
    s("来源", "src"),
)
P_MEAS = J(
    r"目标ID=(?P<tid>\d+)",
    f("斜距", "rng") + "m",
    rf"方位/俯仰=\((?P<az>{_N}),(?P<el>{_N})\)",
)
P_MERR = J(
    r"目标ID=(?P<tid>\d+)",
    rf"目标量测误差统计=斜距误差=(?P<dr>{_N})m",
    rf"方位/俯仰误差=\((?P<daz>{_N}),(?P<del>{_N})\)",
)
P_ASSOC = J(
    r"目标ID=(?P<tid>\d+)",
    rf"位置LLA=\((?P<lat>{_N}),(?P<lon>{_N}),(?P<h>{_N})\)",
    f("检测概率", "pd"),
    s("关联结果", "result", r".+$"),
)
P_MOTION = re.compile(
    rf"航迹=(?P<track>\d+) 目标ID=(?P<tid>\d+) 速度=(?P<speed>{_N})m/s "
    rf"高度=(?P<alt>{_N})m 加速度=(?P<acc>{_N})m/s\^2 "
    rf"近似直线=(?P<straight>\S+) 速度来源=(?P<v_src>\S+) "
    rf"加速度来源=(?P<a_src>\S+)"
    # 2026-11-06 起该行中间多了特征置信度字段（原“综合识别加权贡献”），老日志没有，两者都要认。
    rf"(?: {fc('contrib')})? "
    rf"识别类别=(?P<cls>\S+)")
P_RCSF = J(
    rf"航迹=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    rf"RCS均值/标准差=(?P<mean>{_N})/(?P<std>{_N})dBsm",
    f("平均RCS归一化值", "norm"),
    f("RCS波动系数", "fluct"),
    fc("contrib"),
    s("识别类别", "cls"),
)
P_POL = {
    "det":   J(rf"航迹=(?P<track>\d+)", rf"目标ID=(?P<tid>\d+)",
               rf"极化散射矩阵行列式均值/标准差=(?P<m>{_N})°?/(?P<s>{_N})°?"),
    "span":  J(rf"航迹=(?P<track>\d+)", rf"目标ID=(?P<tid>\d+)",
               rf"功率迹（Span）均值/标准差=(?P<m>{_N})°?/(?P<s>{_N})°?"),
    "depol": J(rf"航迹=(?P<track>\d+)", rf"目标ID=(?P<tid>\d+)",
               rf"去极化系数均值/标准差=(?P<m>{_N})°?/(?P<s>{_N})°?"),
    "ori":   J(rf"航迹=(?P<track>\d+)", rf"目标ID=(?P<tid>\d+)",
               rf"本征极化方向角均值/标准差=(?P<m>{_N})°?/(?P<s>{_N})°?"),
    "ell":   J(rf"航迹=(?P<track>\d+)", rf"目标ID=(?P<tid>\d+)",
               rf"本征极化椭圆率均值/标准差=(?P<m>{_N})°?/(?P<s>{_N})°?"),
}
P_POLHEAD = J(rf"航迹=(?P<track>\d+)", rf"目标ID=(?P<tid>\d+)")
P_POLX = J(fc("contrib"))
P_HRRP = J(
    rf"航迹=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    lst("散射中心", "centers"),
    f("长度", "length") + "m",
    rf"峰数=(?P<npeaks>\d+)",
    fc("contrib"),
    s("识别类型", "cls"),
)
P_CLS = J(rf"航迹=(?P<track>\d+)", rf"目标ID=(?P<tid>\d+)", s("目标类别", "cls"))
P_SCALE = J(
    rf"集群目标数量=(?P<n_cluster>\d+)",
    rf"在跟=(?P<n_track>\d+)",
    rf"待确认=(?P<n_pending>\d+)",
    rf"丢失=(?P<n_lost>\d+)",
    lst("确认目标ID", "ids"),
    s("在跟列表", "track_list", r".+$"),
)
P_HORIZON = J(rf"雷达ID=(?P<rid>\d+)",
              rf"方位/俯仰指向信息=\((?P<az>{_N}),(?P<el>{_N})\)")


def scan_radar_output(path: Path) -> dict[str, pd.DataFrame]:
    rows = Rows()
    gain: dict[tuple, dict] = {}
    noise: dict[tuple, dict] = {}
    jg: dict[tuple, dict] = {}
    cfar: dict[tuple, dict] = {}
    rcsdet: dict[tuple, dict] = {}
    mtt: dict[tuple, dict] = {}
    meas: dict[tuple, dict] = {}
    pol: dict[tuple, dict] = {}

    def key_of(t, cyc, k=109):
        return (t, cyc, k)

    with open_text(path) as fh:
        for line in fh:
            m = HEAD.match(line)
            if not m:
                continue
            t = float(m.group(1)); cyc = int(m.group(2)); msg = m.group(3)
            s_ = line[m.end():]
            base = dict(t=t, cyc=cyc)

            if msg == "波束扫描功能测试":
                mm = P_BEAM.search(s_)
                if mm:
                    rows.add("beam", conv(mm.groupdict(), base))
                else:
                    mm = P_BEAM_NOSEQ.search(s_)
                    if mm:
                        rows.add("beam", conv(mm.groupdict(), base))
                    else:
                        mm = P_BEAMSUM.search(s_)
                        if mm:
                            rows.add("beam_sum", conv(mm.groupdict(), base))

            elif msg == "空域自主搜索功能测试":
                mm = P_SEARCH.search(s_)
                if mm:
                    rows.add("search", conv(mm.groupdict(), base))

            elif msg == "调度策略功能测试":
                mm = P_SCHED.search(s_)
                if mm:
                    rows.add("sched", conv(mm.groupdict(), base))

            elif msg == "干扰功率计算功能测试":
                mm = P_JAMPOW.search(s_)
                if mm:
                    rows.add("jam_power", conv(mm.groupdict(), base))

            elif msg == "目标信号增益功能测试":
                k = key_of(t, cyc)
                row = gain.setdefault(k, dict(base))
                for pat in (P_GAIN1, P_GAIN2, P_GAIN3, P_GAIN4):
                    mm = pat.search(s_)
                    if mm:
                        row.update(conv(mm.groupdict()))

            elif msg == "噪声增益功能测试":
                k = key_of(t, cyc)
                row = noise.setdefault(k, dict(base))
                for pat in (P_NOISE1, P_NOISE2):
                    mm = pat.search(s_)
                    if mm:
                        row.update(conv(mm.groupdict()))

            elif msg == "干扰信号处理增益功能测试":
                k = key_of(t, cyc)
                row = jg.setdefault(k, dict(base))
                for pat in (P_JG1, P_JG2, P_JG3):
                    mm = pat.search(s_)
                    if mm:
                        row.update(conv(mm.groupdict()))

            elif msg == "回波功率计算功能测试":
                mm = P_ECHO.search(s_)
                if mm:
                    rows.add("echo", conv(mm.groupdict(), base))

            elif msg == "恒虚警检测功能测试":
                k = (t, cyc)
                row = cfar.setdefault(k, dict(base))
                for pat in (P_CFAR1, P_CFAR2, P_CFAR3):
                    mm = pat.search(s_)
                    if mm:
                        row.update(conv(mm.groupdict()))

            elif msg == "根据目标RCS实时计算目标探测结果功能测试":
                mm = P_RCSDET.search(s_)
                if mm:
                    g = conv(mm.groupdict())
                    k = (t, cyc, int(g.pop("tid")))
                    rcsdet.setdefault(k, dict(base)).update(g)
                else:
                    mm = P_RCSMOT.search(s_)
                    if mm:
                        g = conv(mm.groupdict())
                        k = (t, cyc, int(g.pop("tid")))
                        rcsdet.setdefault(k, dict(base)).update(g)

            elif msg == "跟踪滤波功能测试":
                mm = P_KF.search(s_)
                if mm:
                    rows.add("kf", conv(mm.groupdict(), base))

            elif msg == "多目标跟踪功能测试":
                mm = P_MTT1.search(s_)
                if mm:
                    g = conv(mm.groupdict())
                    k = (t, cyc, int(g.pop("track")))
                    mtt.setdefault(k, dict(base)).update(g)
                else:
                    mm = P_MTT2.search(s_)
                    if mm:
                        g = conv(mm.groupdict())
                        k = (t, cyc, int(g.pop("track")))
                        mtt.setdefault(k, dict(base)).update(g)

            elif msg == "对目标的角度和距离进行测量功能测试":
                mm = P_MEAS.search(s_)
                if mm:
                    g = conv(mm.groupdict())
                    k = (t, cyc, int(g.pop("tid")))
                    meas.setdefault(k, dict(base)).update(g)
                else:
                    mm = P_MERR.search(s_)
                    if mm:
                        g = conv(mm.groupdict())
                        k = (t, cyc, int(g.pop("tid")))
                        meas.setdefault(k, dict(base)).update(g)

            elif msg == "航迹关联功能测试":
                mm = P_ASSOC.search(s_)
                if mm:
                    rows.add("assoc", conv(mm.groupdict(), base))

            elif msg == "运动特征处理功能测试":
                mm = P_MOTION.search(s_)
                if mm:
                    rows.add("motion", conv(mm.groupdict(), base))

            elif msg == "RCS统计特征处理功能测试":
                mm = P_RCSF.search(s_)
                if mm:
                    rows.add("rcs_feat", conv(mm.groupdict(), base))

            elif msg == "极化特征解算功能测试":
                # 同一周期里多个目标各发若干行，必须按 (t, cyc, 目标) 归并，
                # 否则后一个目标会覆盖前一个目标的极化特征。
                mh = P_POLHEAD.search(s_)
                if mh is None:
                    continue
                head = conv(mh.groupdict())
                k = (t, cyc, int(head["tid"]))
                row = pol.setdefault(k, dict(base, track=int(head["track"]),
                                             tid=int(head["tid"])))
                for name, pat in P_POL.items():
                    mm = pat.search(s_)
                    if mm:
                        g = conv(mm.groupdict())
                        row["track"] = int(g["track"])
                        row["tid"] = int(g["tid"])
                        row[f"{name}_m"] = g["m"]
                        row[f"{name}_s"] = g["s"]
                mm = P_POLX.search(s_)
                if mm:
                    row.update(conv(mm.groupdict()))

            elif msg == "宽带一维像特征解算功能测试":
                mm = P_HRRP.search(s_)
                if mm:
                    rows.add("hrrp", conv(mm.groupdict(), base))

            elif msg == "综合特征识别功能测试":
                mm = P_CLS.search(s_)
                if mm:
                    rows.add("cls", conv(mm.groupdict(), base))

            elif msg == "规模目标识别功能测试":
                mm = P_SCALE.search(s_)
                if mm:
                    rows.add("scale", conv(mm.groupdict(), base))

            elif msg == "地平线搜索功能测试":
                mm = P_HORIZON.search(s_)
                if mm:
                    rows.add("horizon", conv(mm.groupdict(), base))

    for name, store in (("gain", gain), ("noise", noise), ("jam_gain", jg),
                        ("cfar", cfar), ("rcs_detect", rcsdet), ("mtt", mtt),
                        ("meas", meas), ("pol_feat", pol)):
        if store:
            rows.d[name] = [dict(v) for v in store.values()]
    return rows.frames()


# ================================================================== 雷达对外接口
P_SCENE = J(
    rf"目标序号=(?P<idx>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("目标名称", "name"),
    v3("位置ENU", "e") + "m",
    v3("速度ENU", "v") + "m/s",
    f("RCS", "rcs") + r"m\^2",
    rf"Swerling=(?P<swerling>\d+)",
)
P_RES = J(
    rf"已用跟踪容量=(?P<used>\d+)",
    rf"可用跟踪容量=(?P<free>\d+)",
    rf"航迹状态候选/确认/丢失=(?P<n_cand>\d+)/(?P<n_conf>\d+)/(?P<n_lost>\d+)",
    rf"波束任务数=(?P<n_beam>\d+)",
    rf"搜索/指定/外部跟踪/库内跟踪任务=(?P<n_search>\d+)/(?P<n_desig>\d+)/"
    rf"(?P<n_ext>\d+)/(?P<n_lib>\d+)",
    lst("波束任务队列", "queue"),
    rf"当前目标数量=(?P<n_target>\d+)",
    lst("当前目标列表", "targets"),
    rf"告警数量=(?P<n_alarm>\d+)",
)
P_JAMN = J(rf"干扰数量=(?P<n>\d+)", rf"世界周期=(?P<wc>\d+)",
           rf"接收时间窗=(?P<win>{_N})/(?P<win_dt>{_N})s")
P_JAMENT = J(
    rf"干扰源ID=(?P<src>\d+)",
    rf"发射ID=(?P<txid>\d+)",
    v3("位置ECEF", "p") + "m",
    v3("速度ECEF", "v") + "m/s",
    s("波形", "wave"),
    rf"活动起点/持续时间=(?P<t0>{_N})/(?P<dur>{_N})s",
    f("中心频率", "fc") + "Hz",
    f("带宽", "bw") + "Hz",
    f("发射功率", "ptx") + "W",
    f("峰值增益", "gpeak") + "dBi",
    rf"极化=(?P<pol>\d+)",
)
P_PLOT = J(
    r"数据层级=点迹量测",
    rf"目标ID=(?P<tid>\d+)",
    rf"点迹ID=(?P<pid>\d+)",
    s("检测标志", "det"),
    f("距离", "rng") + "m",
    f("方位", "az") + "deg",
    f("俯仰", "el") + "deg",
    f("径向速度(远离为正)", "vr") + "m/s",
    f("SNR", "snr") + "dB",
    f("检测概率", "pd"),
)
P_TOUT = J(
    r"数据层级=航迹识别",
    rf"航迹ID=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("生命周期状态", "state"),
    v3("目标状态ENU_位置", "e") + "m",
    v3("速度", "v") + "m/s",
    s("坐标系", "frame"),
    v3("加速度", "a") + r"m/s\^2",
    r"[^ ]*",                      # 状态协方差ENU[...]=[...]
    s("目标类别", "cls"),
    s("目标型号", "model"),
)
P_TOUT_Q = J(
    rf"RCS特征摘要=均值(?P<rcs_mean>{_N})dBsm/标准差(?P<rcs_std>{_N})dB/"
    rf"质量(?P<rcs_q>{_N})",
    rf"极化特征摘要=能量差(?P<pol_de>{_N})dB/相对差(?P<pol_rel>{_N})dB/"
    rf"质量(?P<pol_q>{_N})/Span均值(?P<pol_span_m>{_N})/Span标准差(?P<pol_span_s>{_N})",
    rf"HRRP特征摘要=长度(?P<hrrp_len>{_N})m/峰数(?P<hrrp_peaks>\d+)/"
    rf"能量集中率(?P<hrrp_conc>{_N})/分辨率(?P<hrrp_res>{_N})m/质量(?P<hrrp_q>{_N})",
)


def scan_radar_ext(path: Path) -> dict[str, pd.DataFrame]:
    rows = Rows()
    with open_text(path) as fh:
        for line in fh:
            m = HEAD.match(line)
            if not m:
                continue
            t = float(m.group(1)); cyc = int(m.group(2)); msg = m.group(3)
            s_ = line[m.end():]
            base = dict(t=t, cyc=cyc)

            if msg == "目标场景接口测试":
                mm = P_SCENE.search(s_)
                if mm:
                    rows.add("scene", conv(mm.groupdict(), base))

            elif msg == "雷达资源状态与日志接口测试":
                mm = P_RES.search(s_)
                if mm:
                    rows.add("res", conv(mm.groupdict(), base))

            elif msg == "外部干扰输入雷达接口测试":
                mm = P_JAMN.search(s_)
                if mm:
                    rows.add("jam_n", conv(mm.groupdict(), base))
                else:
                    mm = P_JAMENT.search(s_)
                    if mm:
                        rows.add("jam_in", conv(mm.groupdict(), base))

            elif msg == "雷达对外目标信息输出接口测试-点迹量测":
                mm = P_PLOT.search(s_)
                if mm:
                    rows.add("plot", conv(mm.groupdict(), base))

            elif msg == "雷达对外目标信息输出接口测试-航迹识别":
                mm = P_TOUT.search(s_)
                if mm:
                    row = conv(mm.groupdict(), base)
                    mmq = P_TOUT_Q.search(s_)
                    if mmq:
                        row.update(conv(mmq.groupdict()))
                    rows.add("track_out", row)
    return rows.frames()


# ================================================================== 雷达内部接口
P_ESP = J(
    rf"雷达ID=(?P<rid>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    f("斜距", "rng") + "m",
    f("RCS", "rcs") + r"m\^2",
    f("波束离轴增益", "g_off") + "dBi",
    f("回波计算输出功率", "p_out") + "W",
    f("处理入口回波功率", "p_in") + "W",
    f("回波计算输出时延", "delay_out") + "s",
    f("处理入口回波时延", "delay_in") + "s",
    f("回波计算输出双程多普勒", "dop_out") + "Hz",
    f("处理入口双程多普勒", "dop_in") + "Hz",
    f("脉压增益", "pc_gain") + "dB",
    f("带宽", "bw") + "Hz",
    f("PRF", "prf") + "Hz",
    rf"有效脉冲数=(?P<npulse>\d+)",
)
P_DETECT = J(
    rf"目标ID=(?P<tid>\d+)",
    f("处理输出SNR", "snr_out") + "dB",
    f("检测器输入SNR", "snr_in") + "dB",
    f("积累SNR", "snr_int") + "dB",
    f("总噪声功率", "p_noise") + "W",
    rf"脉压/MTI/MTD增益=(?P<g_pc>{_N})/(?P<g_mti>{_N})/(?P<g_mtd>{_N})dB",
    f("统计门限", "thr") + "dB",
    f("Pd", "pd"),
    s("检测标志", "det"),
)
P_MTRK = J(
    rf"源目标索引=(?P<idx>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    v3("量测位置ENU", "e") + "m",
    v3("速度种子", "v") + "m/s",
    f("Pd", "pd"),
    f("RCS", "rcs") + r"m\^2",
    r"[^ ]*",                      # 量测协方差=[...]
    rf"关联键=(?P<key>\d+)",
    s("消费结果", "result"),
)
P_TF = J(
    rf"航迹=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("航迹状态", "state"),
    v3("位置ENU", "e") + "m",
    v3("速度ENU", "v") + "m/s",
    s("坐标系", "frame"),
    f("SNR", "snr") + "dB",
    rf"特征有效掩码=(?P<mask>\d+)",
    rf"特征来源周期=(?P<feat_cyc>\d+)",
    rf"特征批次=(?P<feat_batch>\d+)",
    rf"识别来源周期=(?P<rec_cyc>\d+)",
    rf"识别批次=(?P<rec_batch>\d+)",
    s("类别", "cls"),
    s("型号", "model"),
    f("综合分", "s1"),
    f("次优分", "s2"),
    f("边际", "margin"),
    rf"有效维度数=(?P<ndim>\d+)",
    rf"积累观测数=(?P<nobs>\d+)",
)
P_TF_LOST = J(
    rf"航迹=(?P<track>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("航迹状态", "state"),
)


def scan_radar_int(path: Path) -> dict[str, pd.DataFrame]:
    rows = Rows()
    with open_text(path) as fh:
        for line in fh:
            m = HEAD.match(line)
            if not m:
                continue
            t = float(m.group(1)); cyc = int(m.group(2)); msg = m.group(3)
            s_ = line[m.end():]
            base = dict(t=t, cyc=cyc)

            if msg == "雷达回波与信号处理交互接口测试":
                mm = P_ESP.search(s_)
                if mm:
                    rows.add("echo_sp", conv(mm.groupdict(), base))

            elif msg == "雷达信号处理与检测判决交互接口测试":
                mm = P_DETECT.search(s_)
                if mm:
                    rows.add("det", conv(mm.groupdict(), base))

            elif msg == "雷达检测量测与航迹处理交互接口测试":
                mm = P_MTRK.search(s_)
                if mm:
                    rows.add("meas_trk", conv(mm.groupdict(), base))

            elif msg == "雷达航迹与特征识别交互接口测试":
                mm = P_TF.search(s_)
                if mm:
                    rows.add("trk_feat", conv(mm.groupdict(), base))
                else:
                    mm = P_TF_LOST.search(s_)
                    if mm:
                        rows.add("trk_lost", conv(mm.groupdict(), base))
    return rows.frames()


# ================================================================== 红外内部接口
P_IGEOM = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("阶段", "stage"),
    f("斜距", "rng") + "m",
    r".*?",
    rf"几何输出角\(ECI\)=\((?P<g_az>{_N}),(?P<g_el>{_N})\)deg",
    rf"视场门输入角\(传感器系\)=\((?P<fov_az>{_N}),(?P<fov_el>{_N})\)deg",
    rf"扫描中心=\((?P<c_az>{_N}),(?P<c_el>{_N})\)deg",
    s("视场门结果", "fov_res"),
    s("门控结果", "gate_res"),
)
P_IGEOM_OCC = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("门控阶段", "stage"),
    s("门控结果", "gate_res"),
    s("排除码", "code"),
    f("遮挡余量", "margin") + "m",
)
P_IRAD = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("阶段", "stage"),
    f("目标辐射强度", "intensity") + "W/sr",
    f("大气透过率", "tau"),
    f("斜距", "rng") + "m",
    f("接收孔径", "aperture") + "m",
    f("光学透过率", "tau_opt"),
    f("量子效率", "qe"),
    f("积分时间", "tint") + "s",
    f("背景辐射", "bg") + r"W/\(sr·m\^2\)",
    f("接收功率", "p_rx") + "W",
    f("信号能量", "e_sig") + "J",
    f("光子噪声", "e_ph") + "J",
    r"[^ ]*",
    r"[^ ]*",
    f("有效噪声能量", "e_noise") + "J",
    f("辐射噪声模块输出SNR", "snr"),
    f("判决模块输入SNR", "snr_in"),
    f("门限", "thr"),
    s("SNR门结果", "snr_res"),
    s("视场门结果", "fov_res"),
    r"[^ ]*",
    s("综合准入结果", "admit"),
)
P_ICAND = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    f("候选量测时刻", "tmeas") + "s",
    rf"候选测角=\((?P<az>{_N}),(?P<el>{_N})\)deg",
    f("SNR", "snr"),
    rf"连续命中=(?P<hits>\d+)",
    r"[^ ]*",
    s("候选来源", "source"),
    s("调度结果", "sched"),
    rf"指令方位/俯仰=\((?P<cmd_az>{_N}),(?P<cmd_el>{_N})\)deg",
    s("候选状态", "state"),
    s("捕获失败原因", "fail"),
)
P_ITRK = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    f("量测时刻", "tmeas") + "s",
    rf"预测角=\((?P<az>{_N}),(?P<el>{_N})\)deg",
    rf"角速度=\((?P<w_az>{_N}),(?P<w_el>{_N})\)deg/s",
    f("视线角速率", "los_rate") + "deg/s",
    s("输出阶段", "stage"),
    rf"帧数=(?P<frames>\d+)",
    rf"单帧/融合角噪声=(?P<sig1>{_N})/(?P<sig2>{_N})deg",
    r"[^ ]*",                      # 滤波器量测输入=(...)
    r"[^ ]*",                      # 量测协方差=[...]rad^2
    rf"滤波输出角=\((?P<fov_az>{_N}),(?P<fov_el>{_N})\)rad",
    f("NIS", "nis"),
    s("NIS门结果", "nis_res"),
    s("跟踪状态", "trk"),
)


def scan_ir_int(path: Path) -> dict[str, pd.DataFrame]:
    rows = Rows()
    with open_text(path) as fh:
        for line in fh:
            m = HEAD.match(line)
            if not m:
                continue
            t = float(m.group(1)); cyc = int(m.group(2)); msg = m.group(3)
            s_ = line[m.end():]
            base = dict(t=t, cyc=cyc)

            if msg == "红外场景几何与视场门控交互接口测试":
                mm = P_IGEOM.search(s_)
                if mm:
                    rows.add("ir_geom", conv(mm.groupdict(), base))
                else:
                    mm = P_IGEOM_OCC.search(s_)
                    if mm:
                        rows.add("ir_geom", conv(mm.groupdict(), base))

            elif msg == "红外辐射噪声与检测判决交互接口测试":
                mm = P_IRAD.search(s_)
                if mm:
                    rows.add("ir_rad", conv(mm.groupdict(), base))

            elif msg == "红外宽场候选与窄场调度交互接口测试":
                mm = P_ICAND.search(s_)
                if mm:
                    rows.add("ir_cand", conv(mm.groupdict(), base))

            elif msg == "红外量测与跟踪滤波交互接口测试":
                mm = P_ITRK.search(s_)
                if mm:
                    rows.add("ir_trk", conv(mm.groupdict(), base))
    return rows.frames()


# ================================================================== 红外对外接口
P_IENV = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"目标数量=(?P<n_target>\d+)",
    f("背景辐射亮度", "bg") + r"W/\(sr·m\^2\)",
    f("基础大气透过率", "tau"),
    f("环境温度", "temp") + "C",
    f("相对湿度", "humid") + "%",
    f("探测器温度", "det_t") + "K",
)
P_IOBS = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"观测ID=(?P<oid>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("检测标志", "det"),
    f("方位(ECI)", "az") + "rad",
    f("俯仰(ECI)", "el") + "rad",
    f("IR_SNR", "snr"),
    s("观测阶段", "stage"),
    s("引导估计距离", "dist", r"[^ ]+"),
)


def scan_ir_ext(path: Path) -> dict[str, pd.DataFrame]:
    rows = Rows()
    with open_text(path) as fh:
        for line in fh:
            m = HEAD.match(line)
            if not m:
                continue
            t = float(m.group(1)); cyc = int(m.group(2)); msg = m.group(3)
            s_ = line[m.end():]
            base = dict(t=t, cyc=cyc)

            if msg == "目标与红外环境场景输入接口测试":
                mm = P_IENV.search(s_)
                if mm:
                    rows.add("ir_env", conv(mm.groupdict(), base))

            elif msg == "红外观测与航迹输出接口测试":
                mm = P_IOBS.search(s_)
                if mm:
                    rows.add("ir_obs", conv(mm.groupdict(), base))
    return rows.frames()


# ================================================================== 卫星
P_WFOV = J(
    rf"卫星ID=(?P<sat>\d+)",
    lst("覆盖四角经纬", "quad"),
    rf"中心=\((?P<center>[^)]*)\)",
    f("驻留时间", "dwell") + "s",
)
P_POS = J(
    rf"卫星ID=(?P<sat>\d+)",
    v3("真值位置ECEF", "tp") + "m",
    v3("实际位置ECEF", "ap") + "m",
    v3("定位误差ECEF", "pe") + "m",
    v3("真值速度ECEF", "tv") + "m/s",
    v3("实际速度ECEF", "av") + "m/s",
    v3("速度误差ECEF", "ve") + "m/s",
    rf"\|定位误差\|=(?P<err>{_N})m",
)
P_ANG = J(
    rf"相对卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    rf"测角残差方位/俯仰=\((?P<az>{_N}),(?P<el>{_N})\)arcsec",
)
P_MSF = J(
    rf"航迹键=(?P<key>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    lst("传感器信息源名称", "srcs"),
    r"[^ ]*",
    s("测角信息融合卫星地面站名称", "ground"),
    rf"协方差对角=\((?P<cov>[^)]*)\)",
    f("速度精度1σ", "vel_sigma") + "m/s",
)
P_MSF_SAT = J(
    rf"相对卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    rf"目标方位/俯仰\(ECI\)=\((?P<az>{_N}),(?P<el>{_N})\)rad",
    s("红外可探测", "detect"),
)
P_UKF = J(
    rf"位置LLA=\((?P<lla>[^)]*)\)",
    rf"ECEF位置m=\((?P<ecef>[^)]*)\)",
    rf"速度m/s=\((?P<vel>[^)]*)\)",
    rf"加速度m/s\^2=\((?P<acc>[^)]*)\)",
    f("协方差迹", "covtr"),
)
P_IMP = J(
    rf"目标键=(?P<key>\d+)",
    rf"预测落点=\((?P<lat>{_N}),(?P<lon>{_N}),(?P<h>{_N})\)",
    f("落点误差范围1σ", "r1") + "m",
    f("置信度", "conf"),
)
P_LAU = J(
    rf"目标键=(?P<key>\d+)",
    rf"预测发射点地理位置=\((?P<lat>{_N}),(?P<lon>{_N}),(?P<h>{_N})\)",
    f("位置误差1σ", "r1") + "m",
    rf"误差椭圆半长/半短/方位=\((?P<ea>{_N})m,(?P<eb>{_N})m,(?P<eaz>{_N})",
)
P_SHT = J(rf"目标键=(?P<key>\d+)", s("关机时刻", "t_shut"),
          s("关机点经纬高", "point"))
P_NFOV = J(
    rf"卫星ID=(?P<sat>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    rf"脱靶量m=\((?P<mx>{_N}),(?P<my>{_N})\)",
    f("脱靶量模", "miss") + "m",
    rf"脱靶量像素=\((?P<px>{_N}),(?P<py>{_N})\)",
    r"[^ ]*",
    rf"窄视场视线角误差方位/俯仰=\((?P<az_err>{_N}),(?P<el_err>{_N})\)arcsec",
    s("跟踪状态", "trk"),
    f("信号能量", "e_sig") + "J",
    f("SNR", "snr") + "dB",
    r".*?",                        # 线性判定=(...) 内含空格
    f("跟踪位置误差", "pos_err") + "m",
    r"[^ ]*",
    rf"跟踪位置协方差1σ=\((?P<cov>[^)]*)\)",
    f("位置误差/σ", "esig"),
    rf"帧数=(?P<frames>\d+)",
    s("帧门结果", "gate"),
)
P_EVT = J(
    s("事件", "event"),
    rf"卫星ID=(?P<sat>\d+)",
    rf"等级=(?P<level>\d+)",
    rf"目标ID=(?P<tid>\d+)",
    s("阶段", "stage"),
    f("时间", "t_event") + "s",
)
P_RMAX = J(rf"相对卫星ID=(?P<sat>\d+)", rf"目标ID=(?P<tid>\d+)",
           f("相对卫星最大探测距离", "rmax") + "m")


def scan_sat(path: Path) -> dict[str, pd.DataFrame]:
    rows = Rows()
    with open_text(path) as fh:
        for line in fh:
            m = HEAD.match(line)
            if not m:
                continue
            t = float(m.group(1)); cyc = int(m.group(2)); msg = m.group(3)
            s_ = line[m.end():]
            base = dict(t=t, cyc=cyc)

            if msg == "宽视场扫描探测功能测试":
                mm = P_WFOV.search(s_)
                if mm:
                    rows.add("wfov", conv(mm.groupdict(), base))

            elif msg == "卫星自身定位误差功能测试":
                mm = P_POS.search(s_)
                if mm:
                    rows.add("pos_err", conv(mm.groupdict(), base))

            elif msg == "红外载荷角误差功能测试":
                mm = P_ANG.search(s_)
                if mm:
                    rows.add("ang_err", conv(mm.groupdict(), base))

            elif msg == "多传感器目标跟踪功能测试":
                mm = P_MSF.search(s_)
                if mm:
                    rows.add("msf", conv(mm.groupdict(), base))
                else:
                    mm = P_MSF_SAT.search(s_)
                    if mm:
                        rows.add("ir_detect", conv(mm.groupdict(), base))

            elif msg == "UKF滤波":
                mm = P_UKF.search(s_)
                if mm:
                    rows.add("ukf", conv(mm.groupdict(), base))

            elif msg == "落点预报与信息分发功能测试":
                mm = P_IMP.search(s_)
                if mm:
                    rows.add("impact", conv(mm.groupdict(), base))

            elif msg == "发射点预测功能测试":
                mm = P_LAU.search(s_)
                if mm:
                    rows.add("launch", conv(mm.groupdict(), base))

            elif msg == "关机点预测功能测试":
                mm = P_SHT.search(s_)
                if mm:
                    rows.add("shutdown", conv(mm.groupdict(), base))

            elif msg == "窄视场跟踪探测功能测试":
                mm = P_NFOV.search(s_)
                if mm:
                    rows.add("nfov", conv(mm.groupdict(), base))

            elif msg == "特殊事件监测与提示功能测试":
                mm = P_EVT.search(s_)
                if mm:
                    rows.add("event", conv(mm.groupdict(), base))

            elif msg == "最大探测距离计算功能测试":
                mm = P_RMAX.search(s_)
                if mm:
                    rows.add("maxrange", conv(mm.groupdict(), base))
    return rows.frames()


# ================================================================== 缓存
SCANNERS = {
    "radar_output": ("RadarOutput.log", scan_radar_output),
    "radar_ext": ("RadarExternalInterface.log", scan_radar_ext),
    "radar_int": ("RadarInternalInterface.log", scan_radar_int),
    "ir_int": ("IRInternalInterface.log", scan_ir_int),
    "ir_ext": ("IRExternalInterface.log", scan_ir_ext),
    "sat": ("SatelliteTarget.log", scan_sat),
}

_MEM: dict[str, dict[str, pd.DataFrame]] = {}


def _cache_file(tag: str, path: Path) -> Path:
    st = path.stat()
    return common.CACHE_DIR / f"{tag}_{st.st_size}_{int(st.st_mtime)}_v{CACHE_VERSION}.pkl"


def load(tag: str, *, refresh: bool = False, verbose: bool = True):
    if tag in _MEM and not refresh:
        return _MEM[tag]
    fname, fn = SCANNERS[tag]
    path = resolve_log(fname)
    common.CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cf = _cache_file(tag, path)
    if cf.exists() and not refresh:
        with cf.open("rb") as fh:
            data = pickle.load(fh)
    else:
        if verbose:
            t0 = time.time()
            print(f"    · 解析 {fname} ({detect_encoding(path)}) ...", end="", flush=True)
        data = fn(path)
        if verbose:
            print(f" {time.time() - t0:.1f}s, "
                  f"{sum(len(v) for v in data.values())} 条")
        with cf.open("wb") as fh:
            pickle.dump(data, fh, protocol=4)
    _MEM[tag] = data
    return data


# ================================================================== CSV
def antenna_pattern():
    """(az_deg, el_deg, gain_dB[n_az, n_el])"""
    path = resolve_log("RadarAntennaGainPatterndata.csv")
    st = path.stat()
    common.CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cf = common.CACHE_DIR / f"antenna_{st.st_size}_{int(st.st_mtime)}_v1.npz"
    if cf.exists():
        d = np.load(cf)
        return d["az"], d["el"], d["g"]
    df = pd.read_csv(path, usecols=["az_off_deg", "el_off_deg", "gain_dbi"],
                     encoding=detect_encoding(path))
    az = np.unique(df["az_off_deg"].to_numpy())
    el = np.unique(df["el_off_deg"].to_numpy())
    g = df["gain_dbi"].to_numpy().reshape(az.size, el.size)
    np.savez(cf, az=az, el=el, g=g)
    return az, el, g


def scan_pattern() -> pd.DataFrame:
    path = resolve_log("Rir_scan_pattern.csv")
    return pd.read_csv(path, encoding=detect_encoding(path))

