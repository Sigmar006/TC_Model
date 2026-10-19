#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Scan one simulation round and judge it against the acceptance criteria.

The output is organised **by acceptance item**, not by whatever numbers happen
to be convenient: every row names an indicator exactly as
docs/review/functional_acceptance_test_criteria_ocr.md states it, quotes the
criterion's own requirement in the 判据 column, then shows this round's evidence
and a verdict. Interpretations and root-cause hints go into the 说明 column so
they never get confused with the criterion itself.

Items covered are the ones the fix plan touches: 46, 58, 62, 65, 67, 75, 76, 77.
Each log is read once, streaming; the logs are hundreds of MB.

Usage:
    python tools/acceptance_baseline_scan.py
    python tools/acceptance_baseline_scan.py --log-dir <dir> --out <md> --json <json>
"""

import argparse
import json
import math
import os
import re
import sys

DEFAULT_LOG_DIR = r"C:\software\workspace\1qfosim\FOSimStudio\log"

LOGS = {
    "ir_internal": "IRInternalInterface.log",
    "ir_external": "IRExternalInterface.log",
    "radar_external": "RadarExternalInterface.log",
    "radar_output": "RadarOutput.log",
}

ITEM_TITLES = {
    "46": "极化特征解算功能测试",
    "58": "目标与红外环境场景输入接口测试",
    "62": "目标场景接口测试",
    "65": "雷达对外目标信息输出接口测试（航迹识别）",
    "67": "红外观测与航迹输出接口测试",
    "75": "红外辐射噪声与检测判决交互接口测试",
    "76": "红外宽场候选与窄场调度交互接口测试",
    "77": "红外量测与跟踪滤波交互接口测试",
}

ITEM_FILES = {
    "46": "radar_output",
    "58": "ir_external",
    "62": "radar_external",
    "65": "radar_external",
    "67": "ir_external",
    "75": "ir_internal",
    "76": "ir_internal",
    "77": "ir_internal",
}

RE_FLOAT = r"([-\d.eE+]+)"


def rx(pattern):
    return re.compile(pattern)


RE_MEAS_COV = rx(r"量测协方差=\[" + RE_FLOAT + "," + RE_FLOAT + ";" + RE_FLOAT + "," + RE_FLOAT
                 + r"\]rad\^2")
RE_SINGLE_FUSED = rx(r"单帧/融合角噪声=" + RE_FLOAT + "/" + RE_FLOAT + "deg")
RE_FRAMES = rx(r"帧数=(\d+)")
RE_NIS = rx(r"NIS=" + RE_FLOAT)
RE_NIS_GATE = rx(r"NIS门结果=(\S+)")
RE_THRESHOLD = rx(r"门限=" + RE_FLOAT)
RE_SNR = rx(r"判决模块输入SNR=" + RE_FLOAT)
RE_STAGE = rx(r"阶段=(\S+)")
RE_GATE = rx(r"SNR门结果=(\S+)")
RE_VIEW_GATE = rx(r"视场门结果=(\S+)")
RE_SETTLED_GATE = rx(r"稳定帧门结果=(\S+)")
RE_ADMIT = rx(r"综合准入结果=(\S+)")
RE_ENERGY = rx(r"信号能量=" + RE_FLOAT + "J")
RE_NOISE_TOTAL = rx(r"有效噪声能量=" + RE_FLOAT + "J")
RE_ANGLE_RATE = rx(r"角速度=(\S+)")
RE_TRACK_STATE = rx(r"跟踪状态=(\S+)")
RE_POINTING_STATE = rx(r"指向状态=(\S+)")
RE_CANDIDATE_STATE = rx(r"候选状态=(\S+)")
RE_RESIDUAL = rx(r"剩余角=" + RE_FLOAT + "deg")
RE_HITS = rx(r"连续命中=(\d+)")
RE_CUE_SOURCE = rx(r"引导来源卫星ID=(-?\d+)")
RE_SCHED_RESULT = rx(r"调度结果=(\S+)")
RE_RCS = rx(r"RCS=" + RE_FLOAT + r"m\^2")
RE_SWERLING = rx(r"Swerling=(\d+)")
RE_CONFIDENCE = rx(r"置信度=" + RE_FLOAT)
RE_CLASS = rx(r"目标类别=(\S+)")
RE_COUNT = rx(r"目标数量=(\d+)")
RE_OBS_COUNT = rx(r"观测数量=(\d+)")
RE_RADIANCE = rx(r"红外辐射强度=" + RE_FLOAT)
RE_POL_WEIGHT = rx(r"综合识别加权贡献=" + RE_FLOAT)
RE_TRANSMITTANCE = rx(r"大气透过率=" + RE_FLOAT)
RE_PHOTON = rx(r"光子噪声=" + RE_FLOAT)
RE_THERMAL = rx(r"热噪声=" + RE_FLOAT)
RE_READOUT = rx(r"读出噪声=" + RE_FLOAT)
RE_OUT_STAGE = rx(r"输出阶段=(\S+)")
RE_CYCLE = rx(r"仿真周期=(\d+)")


def num(pattern, line, cast=float):
    m = pattern.search(line)
    if m is None:
        return None
    try:
        return cast(m.group(1))
    except (TypeError, ValueError):
        return None


def iter_lines(path):
    with open(path, "rb") as handle:
        for raw in handle:
            try:
                yield raw.decode("utf-8")
            except UnicodeDecodeError:
                yield raw.decode("gbk", errors="replace")


class Stats(object):
    def __init__(self):
        self.min = None
        self.max = None
        self._seen = set()

    def add(self, value):
        if value is None:
            return
        self.min = value if self.min is None else min(self.min, value)
        self.max = value if self.max is None else max(self.max, value)
        self._seen.add(value)

    @property
    def distinct(self):
        return len(self._seen)


def f(value):
    if value is None:
        return "-"
    if isinstance(value, float):
        if value == 0.0:
            return "0"
        if abs(value) >= 1.0e5 or abs(value) < 1.0e-3:
            return "%.3e" % value
        return "%.3f" % value
    return str(value)


def pct(part, whole):
    if not whole:
        return "-"
    return "%.1f%%" % (100.0 * part / whole)


def note_cycle(it, line):
    """Track the simulated cycle range so rounds of different length stay comparable."""
    m = RE_CYCLE.search(line)
    if m is None:
        return
    cycle = int(m.group(1))
    run = it["_run"]
    run["min_cycle"] = cycle if run["min_cycle"] is None else min(run["min_cycle"], cycle)
    run["max_cycle"] = cycle if run["max_cycle"] is None else max(run["max_cycle"], cycle)


def new_items():
    return {
        "_run": {"min_cycle": None, "max_cycle": None},
        "46": {"rows": 0, "sub": {}, "weight_bad": 0, "weight_rows": 0,
               "class_counts": {}},
        "58": {"header_rows": 0, "target_rows": 0, "fields": {}, "counts": {},
               "radiance": Stats()},
        "62": {"rows": 0, "header_rows": 0, "target_rows": 0, "fields": {},
               "rcs": Stats(), "swerling": {}},
        "65": {"rows": 0, "header_rows": 0, "track_rows": 0, "fields": {},
               "confidence": Stats(), "classes": {}},
        "67": {"header_rows": 0, "obs_rows": 0, "track_rows": 0, "fields": {},
               "obs_counts": {}},
        "75": {"rows": 0, "fields": {}, "stage_threshold": {}, "snr_mismatch": 0,
               "admit_mismatch": 0, "admit_rows": 0, "invalid_admit": 0,
               "snr": Stats(), "thresholds": Stats(), "transmittance": Stats(),
               "photon": Stats(), "thermal": Stats(), "readout": Stats(),
               "noise_total": Stats(), "stage_rows": {}},
        "76": {"sched_rows": 0, "point_rows": 0, "fields": {}, "hits": {},
               "cue_sources": {}, "sched_results": {}, "candidate_states": {},
               "pointing_states": {}, "residual": Stats(),
               "residual_zero_stable": 0, "stable_rows": 0,
               "residual_zero_turning": 0, "turning_rows": 0,
               "turning_but_captured": 0, "captured_in_turning": 0},
        "77": {"rows": 0, "fields": {}, "rate_na": 0, "rate_zero": 0,
               "rate_numeric": 0, "rate_reasons": {}, "rates": Stats(),
               "zero_frames": 0, "zero_frames_tracking": 0,
               "nis_gate_mismatch": 0, "nis_rows": 0, "nis": Stats(),
               "cov_mismatch": 0, "cov_rows": 0, "stages": {}},
    }


def scan_ir_internal(path, it):
    d75, d76, d77 = it["75"], it["76"], it["77"]
    required_75 = ["目标辐射强度=", "斜距=", "接收孔径=", "光学透过率=", "大气透过率=",
                   "量子效率=", "积分时间=", "背景辐射=", "光子噪声=", "热噪声=",
                   "读出噪声=", "有效噪声能量=", "接收功率=", "信号能量=",
                   "辐射噪声模块输出SNR=", "判决模块输入SNR=", "门限=", "SNR门结果=",
                   "视场门结果=", "稳定帧门结果=", "综合准入结果="]
    required_76 = ["目标ID=", "候选量测时刻=", "候选测角=", "连续命中=", "引导来源卫星ID=",
                   "调度结果=", "指令方位/俯仰=", "候选状态=", "当前服务目标=",
                   "指向状态=", "剩余角=", "捕获失败原因="]
    required_77 = ["目标ID=", "量测时刻=", "预测角=", "角速度=", "输出阶段=", "帧数=",
                   "单帧/融合角噪声=", "滤波器量测输入=", "量测协方差=", "滤波输出角=",
                   "NIS=", "NIS门结果=", "跟踪状态="]

    for line in iter_lines(path):
        if "有效噪声能量=" in line:
            note_cycle(it, line)
            d75["rows"] += 1
            for marker in required_75:
                if marker in line:
                    d75["fields"][marker] = d75["fields"].get(marker, 0) + 1
            stage = num(RE_STAGE, line, str)
            threshold = num(RE_THRESHOLD, line)
            snr = num(RE_SNR, line)
            d75["snr"].add(snr)
            d75["thresholds"].add(threshold)
            d75["transmittance"].add(num(RE_TRANSMITTANCE, line))
            d75["photon"].add(num(RE_PHOTON, line))
            d75["thermal"].add(num(RE_THERMAL, line))
            d75["readout"].add(num(RE_READOUT, line))
            d75["noise_total"].add(num(RE_NOISE_TOTAL, line))
            if stage:
                d75["stage_rows"][stage] = d75["stage_rows"].get(stage, 0) + 1
                d75["stage_threshold"].setdefault(stage, set()).add(threshold)
            gate = num(RE_GATE, line, str)
            if gate and snr is not None and threshold is not None:
                if (gate == "通过") != (snr >= threshold):
                    d75["snr_mismatch"] += 1
            view = num(RE_VIEW_GATE, line, str)
            settled = num(RE_SETTLED_GATE, line, str)
            admit = num(RE_ADMIT, line, str)
            if view and gate and admit:
                settled_ok = True if stage == "WFOV" else settled == "通过"
                expected = (view == "通过") and (gate == "通过") and settled_ok
                d75["admit_rows"] += 1
                if expected != (admit == "通过"):
                    d75["admit_mismatch"] += 1
            energy = num(RE_ENERGY, line)
            total = num(RE_NOISE_TOTAL, line)
            if admit == "通过" and ((energy is not None and energy <= 0.0) or
                                    (total is not None and total <= 0.0)):
                d75["invalid_admit"] += 1
        elif "连续命中=" in line:
            note_cycle(it, line)
            d76["sched_rows"] += 1
            for marker in required_76:
                if marker in line:
                    d76["fields"][marker] = d76["fields"].get(marker, 0) + 1
            hits = num(RE_HITS, line, int)
            if hits is not None:
                d76["hits"][hits] = d76["hits"].get(hits, 0) + 1
            cue = num(RE_CUE_SOURCE, line, int)
            if cue is not None:
                d76["cue_sources"][cue] = d76["cue_sources"].get(cue, 0) + 1
            sched = num(RE_SCHED_RESULT, line, str)
            if sched:
                d76["sched_results"][sched] = d76["sched_results"].get(sched, 0) + 1
            cand = num(RE_CANDIDATE_STATE, line, str)
            if cand:
                d76["candidate_states"][cand] = d76["candidate_states"].get(cand, 0) + 1
        elif "剩余角=" in line:
            note_cycle(it, line)
            d76["point_rows"] += 1
            for marker in required_76:
                if marker in line:
                    d76["fields"][marker] = d76["fields"].get(marker, 0) + 1
            residual = num(RE_RESIDUAL, line)
            d76["residual"].add(residual)
            status = num(RE_POINTING_STATE, line, str)
            if status:
                d76["pointing_states"][status] = d76["pointing_states"].get(status, 0) + 1
            cand = num(RE_CANDIDATE_STATE, line, str)
            if cand:
                d76["candidate_states"][cand] = d76["candidate_states"].get(cand, 0) + 1
            if cand == "已捕获跟踪":
                d76["captured_in_turning"] += 1
            if status == "已稳定":
                d76["stable_rows"] += 1
                if residual == 0.0:
                    d76["residual_zero_stable"] += 1
            if status == "转动中":
                d76["turning_rows"] += 1
                if residual == 0.0:
                    d76["residual_zero_turning"] += 1
                if cand == "已捕获跟踪":
                    d76["turning_but_captured"] += 1
        elif "角速度=" in line and "NIS=" in line:
            note_cycle(it, line)
            d77["rows"] += 1
            for marker in required_77:
                if marker in line:
                    d77["fields"][marker] = d77["fields"].get(marker, 0) + 1
            rate = num(RE_ANGLE_RATE, line, str) or ""
            if rate.startswith("不适用"):
                d77["rate_na"] += 1
                d77["rate_reasons"][rate] = d77["rate_reasons"].get(rate, 0) + 1
            elif rate.startswith("(0.000000,0.000000)"):
                d77["rate_zero"] += 1
            else:
                d77["rate_numeric"] += 1
                d77["rates"].add(num(rx(r"角速度=\(" + RE_FLOAT), line))
            stage = num(RE_OUT_STAGE, line, str)
            if stage:
                d77["stages"][stage] = d77["stages"].get(stage, 0) + 1
            frames = num(RE_FRAMES, line, int)
            if frames == 0:
                d77["zero_frames"] += 1
                if num(RE_TRACK_STATE, line, str) == "跟踪":
                    d77["zero_frames_tracking"] += 1
            nis = num(RE_NIS, line)
            gate = num(RE_NIS_GATE, line, str)
            if nis is not None and gate:
                d77["nis_rows"] += 1
                d77["nis"].add(nis)
                if (gate == "超门") != (nis > 5.991):
                    d77["nis_gate_mismatch"] += 1
            single_fused = RE_SINGLE_FUSED.search(line)
            cov = RE_MEAS_COV.search(line)
            if single_fused and cov and frames:
                single_sigma = float(single_fused.group(1))
                fused_sigma = float(single_fused.group(2))
                cov00 = float(cov.group(1))
                d77["cov_rows"] += 1
                expected_sigma = single_sigma / math.sqrt(frames)
                if abs(fused_sigma - expected_sigma) > 1.0e-3 * max(expected_sigma, 1.0e-12):
                    d77["cov_mismatch"] += 1
                elif abs(cov00 - (math.radians(fused_sigma)) ** 2) > 0.25 * (
                        math.radians(fused_sigma) ** 2):
                    d77["cov_mismatch"] += 1


def scan_ir_external(path, it):
    d58, d67 = it["58"], it["67"]
    for line in iter_lines(path):
        if "背景辐射亮度=" in line:
            note_cycle(it, line)
            d58["header_rows"] += 1
            for marker in ("目标数量=", "背景辐射亮度=", "基础大气透过率=", "环境温度=",
                           "相对湿度=", "探测器温度="):
                if marker in line:
                    d58["fields"][marker] = d58["fields"].get(marker, 0) + 1
            count = num(RE_COUNT, line, int)
            if count is not None:
                d58["counts"][count] = d58["counts"].get(count, 0) + 1
        elif "红外辐射强度=" in line:
            d58["target_rows"] += 1
            d58["radiance"].add(num(RE_RADIANCE, line))
            for marker in ("目标状态=", "位置ECEF=", "速度ECEF="):
                if marker in line:
                    d58["fields"][marker] = d58["fields"].get(marker, 0) + 1
        elif "观测数量=" in line:
            d67["header_rows"] += 1
            count = num(RE_OBS_COUNT, line, int)
            if count is not None:
                d67["obs_counts"][count] = d67["obs_counts"].get(count, 0) + 1
            if "目标数量=" in line:
                d67["fields"]["目标数量="] = d67["fields"].get("目标数量=", 0) + 1
        elif "检测标志=" in line:
            d67["obs_rows"] += 1
            for marker in ("观测ID=", "目标ID=", "检测标志=", "方位(ECI)=", "俯仰(ECI)=",
                           "IR_SNR=", "观测阶段="):
                if marker in line:
                    d67["fields"][marker] = d67["fields"].get(marker, 0) + 1
            if "引导估计距离=" in line:
                d67["fields"]["引导估计距离="] = d67["fields"].get("引导估计距离=", 0) + 1
                if "(归属诊断,非被动红外直接测距)" in line:
                    d67["fields"]["距离标注="] = d67["fields"].get("距离标注=", 0) + 1
        elif "状态估计来源=" in line:
            d67["track_rows"] += 1
            for marker in ("目标ID=", "状态估计来源=", "状态向量=", "状态协方差="):
                if marker in line:
                    d67["fields"][marker] = d67["fields"].get(marker, 0) + 1


def scan_radar_external(path, it):
    d62, d65 = it["62"], it["65"]
    required_65 = ["航迹数量=", "集群目标数量=", "批次=", "航迹ID=", "目标ID=",
                   "生命周期状态=", "目标状态ENU_位置=", "速度=", "加速度=",
                   "状态协方差ENU", "目标类别=", "目标型号=", "置信度=",
                   "RCS特征摘要=", "极化特征摘要=", "HRRP特征摘要="]
    for line in iter_lines(path):
        if "目标场景接口测试" in line:
            note_cycle(it, line)
            d62["rows"] += 1
            if "目标ID=" in line:
                d62["target_rows"] += 1
            elif "目标数量=" in line:
                d62["header_rows"] += 1
            for marker in ("目标数量=", "目标状态=", "RCS=", "Swerling=", "视角RCS=",
                           "极化数据=", "HRRP散射中心="):
                if marker in line:
                    d62["fields"][marker] = d62["fields"].get(marker, 0) + 1
            d62["rcs"].add(num(RE_RCS, line))
            swerling = num(RE_SWERLING, line, int)
            if swerling is not None:
                d62["swerling"][swerling] = d62["swerling"].get(swerling, 0) + 1
        elif "航迹识别" in line:
            note_cycle(it, line)
            d65["rows"] += 1
            if "航迹ID=" in line:
                d65["track_rows"] += 1
            elif "航迹数量=" in line:
                d65["header_rows"] += 1
            for marker in required_65:
                if marker in line:
                    d65["fields"][marker] = d65["fields"].get(marker, 0) + 1
            d65["confidence"].add(num(RE_CONFIDENCE, line))
            cls = num(RE_CLASS, line, str)
            if cls:
                d65["classes"][cls] = d65["classes"].get(cls, 0) + 1


def scan_radar_output(path, it):
    d46 = it["46"]
    sub_items = (
        ("极化散射矩阵行列式", "极化散射矩阵行列式均值/标准差="),
        ("功率迹（Span）", "功率迹（Span）均值/标准差="),
        ("去极化系数", "去极化系数均值/标准差="),
        ("本征极化方向角", "本征极化方向角均值/标准差="),
        ("本征极化椭圆率", "本征极化椭圆率均值/标准差="),
        ("目标类型", "目标类型="),
        ("综合识别加权贡献", "综合识别加权贡献="),
    )
    for line in iter_lines(path):
        if "极化特征解算功能测试" not in line:
            continue
        note_cycle(it, line)
        d46["rows"] += 1
        for name, marker in sub_items:
            if marker in line:
                stats = d46["sub"].setdefault(name, Stats())
                value = num(rx(re.escape(marker) + RE_FLOAT), line)
                if value is not None:
                    stats.add(value)
        cls = num(rx(r"目标类型=(\S+)"), line, str)
        if cls:
            d46["class_counts"][cls] = d46["class_counts"].get(cls, 0) + 1
        weight = num(RE_POL_WEIGHT, line)
        if weight is not None:
            d46["weight_rows"] += 1
            if weight < 0.0 or weight > 1.0:
                d46["weight_bad"] += 1


SCANNERS = {
    "ir_internal": scan_ir_internal,
    "ir_external": scan_ir_external,
    "radar_external": scan_radar_external,
    "radar_output": scan_radar_output,
}


def build_rows(it):
    rows = []
    add = rows.append

    d = it["46"]
    required_sub = ["极化散射矩阵行列式", "功率迹（Span）", "去极化系数", "本征极化方向角",
                    "本征极化椭圆率"]
    if d["rows"] == 0:
        add(("46", "极化散射矩阵行列式 / 功率迹 / 去极化系数 / 本征方向角 / 本征椭圆率 / 目标类型",
             "能够记录即通过", "0 行", "无样本",
             "本轮无 `极化特征解算功能测试` 行；规范要求缺样本时省略该行，故不能判通过也不能判失败"))
    else:
        for name in required_sub:
            stats = d["sub"].get(name, Stats())
            add(("46", name, "能够记录即通过",
                 "%d 行，数值 distinct=%d" % (d["rows"], stats.distinct),
                 "通过" if d["sub"].get(name) is not None else "不通过",
                 "该项为常量（distinct=1）" if stats.distinct == 1 else ""))
        add(("46", "目标类型", "能够记录目标类型即通过",
             "%d 行；类别分布 %s" % (d["rows"],
                                 json.dumps(d["class_counts"], ensure_ascii=False)),
             "通过" if d["class_counts"] else "不通过", ""))
        add(("46", "综合识别加权贡献", "形成候选匹配后记录，且数值有限并位于 [0,1]",
             "%d 行，越界 %d 行" % (d["weight_rows"], d["weight_bad"]),
             "通过" if d["weight_rows"] and d["weight_bad"] == 0 else
             ("不通过" if d["weight_rows"] else "无样本"), ""))

    d = it["58"]
    for marker, label in (("目标数量=", "目标数量"), ("背景辐射亮度=", "背景辐射亮度"),
                          ("基础大气透过率=", "基础大气透过率"), ("环境温度=", "环境温度"),
                          ("相对湿度=", "相对湿度"), ("探测器温度=", "探测器温度")):
        covered = d["fields"].get(marker, 0)
        add(("58", label, "记录目标数量、背景辐射和大气透过率",
             "%d / %d 头行（%s）" % (covered, d["header_rows"], pct(covered, d["header_rows"])),
             "通过" if covered == d["header_rows"] and d["header_rows"] else "不通过", ""))
    add(("58", "目标状态数组（目标状态/位置ECEF/速度ECEF）", "记录目标状态数组",
         "目标行 %d；目标状态 %d · 位置 %d · 速度 %d" % (
             d["target_rows"], d["fields"].get("目标状态=", 0),
             d["fields"].get("位置ECEF=", 0), d["fields"].get("速度ECEF=", 0)),
         "通过" if d["target_rows"] and d["fields"].get("目标状态=", 0) == d["target_rows"]
         else "不通过", ""))
    add(("58", "红外辐射特征（红外辐射强度）", "记录红外辐射特征",
         "%d 行，distinct=%d，最大值 %s" % (d["target_rows"], d["radiance"].distinct,
                                         f(d["radiance"].max)),
         "通过" if d["target_rows"] else "无样本",
         "全部目标同值属**场景输入**问题（C1），本接口项仍判通过"))
    add(("58", "目标数量取值分布", "逐周期记录目标数量",
         json.dumps({str(k): v for k, v in sorted(d["counts"].items())[:6]}, ensure_ascii=False),
         "通过" if d["counts"] else "无样本", "仅列前 6 个取值"))

    d = it["62"]
    for marker, label, denominator, note in (
            ("目标数量=", "目标数量", "header_rows", ""),
            ("目标状态=", "目标状态数组", "target_rows",
             "★ 本项明列「目标状态数组」，本轮该字段完全缺失"),
            ("RCS=", "目标 RCS", "target_rows", ""),
            ("Swerling=", "Swerling 型", "target_rows",
             "验收标准未单列 Swerling，此处作 RCS 配套说明"),
            ("视角RCS=", "视角 RCS", "target_rows", "属 RCS 的视角展开"),
            ("极化数据=", "极化数据", "target_rows", ""),
            ("HRRP散射中心=", "HRRP 数据", "target_rows", "")):
        covered = d["fields"].get(marker, 0)
        whole = d[denominator]
        add(("62", label, "记录目标数量、目标状态数组、目标 RCS、极化数据和 HRRP 数据",
             "%d / %d 行（%s）" % (covered, whole, pct(covered, whole)),
             "通过" if whole and covered == whole else "不通过", note))
    add(("62", "RCS 取值分布 / Swerling", "逐目标记录 RCS（与 Swerling）",
         "distinct=%d（%s~%s）；Swerling %s" % (
             d["rcs"].distinct, f(d["rcs"].min), f(d["rcs"].max),
             json.dumps({str(k): v for k, v in sorted(d["swerling"].items())})),
         "通过" if d["rcs"].distinct > 0 else "无样本",
         "全部目标同为 0.1 m² 属**场景数据**问题（C2），本接口项仍判通过"))

    d = it["65"]
    for marker, label, denominator in (
            ("航迹数量=", "航迹数量", "header_rows"),
            ("集群目标数量=", "集群目标数量", "header_rows"),
            ("批次=", "批次", "header_rows"),
            ("航迹ID=", "航迹标识", "track_rows"),
            ("目标ID=", "目标标识", "track_rows"),
            ("生命周期状态=", "生命周期状态", "track_rows"),
            ("目标状态ENU_位置=", "目标状态（ENU 位置）", "track_rows"),
            ("速度=", "目标速度", "track_rows"),
            ("加速度=", "目标加速度", "track_rows"),
            ("状态协方差ENU", "状态协方差（6×6 `[x,vx,y,vy,z,vz]`）", "track_rows"),
            ("目标类别=", "目标类别", "track_rows"),
            ("目标型号=", "目标型号", "track_rows"),
            ("置信度=", "置信度", "track_rows"),
            ("RCS特征摘要=", "RCS 特征摘要", "track_rows"),
            ("极化特征摘要=", "极化特征摘要", "track_rows"),
            ("HRRP特征摘要=", "HRRP 特征摘要", "track_rows")):
        covered = d["fields"].get(marker, 0)
        whole = d[denominator]
        add(("65", label, "记录航迹数量、航迹标识、目标状态、状态协方差、集群目标数量、"
             "目标类别以及 RCS/极化/HRRP 特征摘要",
             "%d / %d 行（%s）" % (covered, whole, pct(covered, whole)),
             "通过" if whole and covered == whole else
             ("无样本" if not whole else "不通过"), ""))
    add(("65", "取值为物理量（非字段存在性）", "（无独立判据）",
         "航迹行 %d；类别分布 %s；置信度 max %s" % (
             d["rows"], json.dumps(d["classes"], ensure_ascii=False), f(d["confidence"].max)),
         "-", "字段齐全 ≠ 数值可信：ENU 越界与置信度上不去属 P4/P5 的物理问题"))

    d = it["67"]
    add(("67", "观测数量 / 目标数量", "记录观测数量与目标数量；无输出不出日志",
         "头行 %d；观测数量取值 %s" % (
             d["header_rows"],
             json.dumps({str(k): v for k, v in sorted(d["obs_counts"].items())[:5]})),
         "通过" if d["header_rows"] and 0 not in d["obs_counts"] else "不通过",
         "本轮无 `观测数量=0` 行 ⇒ 未落空投影"))
    for marker, label in (("观测ID=", "目标/观测标识"), ("检测标志=", "检测标志"),
                          ("方位(ECI)=", "方位"), ("俯仰(ECI)=", "俯仰"),
                          ("IR_SNR=", "IR SNR"), ("观测阶段=", "观测阶段")):
        covered = d["fields"].get(marker, 0)
        add(("67", label, "记录目标/航迹标识、检测标志、方位、俯仰、IR SNR",
             "%d / %d 观测行（%s）" % (covered, d["obs_rows"], pct(covered, d["obs_rows"])),
             "通过" if covered == d["obs_rows"] and d["obs_rows"] else "不通过", ""))
    add(("67", "状态估计 / 状态协方差", "记录状态估计与状态协方差（与原始观测分层）",
         "跟踪行 %d；状态估计来源 %d · 状态向量 %d · 状态协方差 %d" % (
             d["track_rows"], d["fields"].get("状态估计来源=", 0),
             d["fields"].get("状态向量=", 0), d["fields"].get("状态协方差=", 0)),
         "通过" if d["track_rows"] and d["fields"].get("状态协方差=", 0) == d["track_rows"]
         else "不通过", ""))
    add(("67", "归属诊断距离标注", "可附带归属/引导诊断距离，但必须明确标注其不是被动红外直接测距",
         "引导估计距离 %d 行，带标注 %d 行" % (
             d["fields"].get("引导估计距离=", 0), d["fields"].get("距离标注=", 0)),
         "通过" if d["fields"].get("引导估计距离=", 0) and
         d["fields"].get("距离标注=", 0) == d["fields"].get("引导估计距离=", 0) else "不通过", ""))

    d = it["75"]
    required_75 = ["目标辐射强度=", "斜距=", "接收孔径=", "光学透过率=", "大气透过率=",
                   "量子效率=", "积分时间=", "背景辐射=", "光子噪声=", "热噪声=",
                   "读出噪声=", "有效噪声能量=", "接收功率=", "信号能量=",
                   "判决模块输入SNR=", "门限=", "SNR门结果=", "视场门结果=",
                   "稳定帧门结果=", "综合准入结果="]
    missing = [m.rstrip("=") for m in required_75 if not d["fields"].get(m)]
    add(("75", "交换数据齐全性（20 项）", "交换数据必须齐全",
         "缺：%s" % ("、".join(missing) if missing else "无"),
         "通过" if not missing else "不通过",
         "含 `辐射噪声模块输出SNR`（辐射噪声模块输出）与 `判决模块输入SNR`（判决入口消费）双口径"))
    stage_thresholds = {k: sorted(v) for k, v in d["stage_threshold"].items()}
    distinct_across = len({tuple(v) for v in stage_thresholds.values()}) > 1
    add(("75", "WFOV/NFOV 各自门限", "WFOV/NFOV 使用各自门限",
         json.dumps({k: [f(x) for x in v] for k, v in stage_thresholds.items()},
                    ensure_ascii=False),
         "通过" if distinct_across else "不通过",
         "两阶段同为 0.001 ⇒ 未使用各自门限（B4）"))
    add(("75", "SNR 门结果与门限比较一致", "SNR 门结果必须与门限比较一致",
         "%d / %d 行不一致" % (d["snr_mismatch"], d["rows"]),
         "通过" if d["snr_mismatch"] == 0 else "不通过", ""))
    add(("75", "综合准入 = 视场 ∩ SNR ∩（NFOV 稳定帧）", "综合准入结果必须同时反映视场、SNR，"
         "以及 NFOV 稳定帧条件",
         "%d / %d 行不一致" % (d["admit_mismatch"], d["admit_rows"]),
         "通过" if d["admit_mismatch"] == 0 else "不通过", ""))
    add(("75", "非法/非有限能量不得形成检测记录", "非法或非有限能量不得形成检测记录",
         "%d 行" % d["invalid_admit"], "通过" if d["invalid_admit"] == 0 else "不通过", ""))
    add(("75", "噪声分量（光子/热/读出）", "（无独立判据，作判读依据）",
         "光子 %s~%s（distinct %d）· 热 %s~%s（distinct %d）· 读出 %s~%s" % (
             f(d["photon"].min), f(d["photon"].max), d["photon"].distinct,
             f(d["thermal"].min), f(d["thermal"].max), d["thermal"].distinct,
             f(d["readout"].min), f(d["readout"].max)),
         "-", "光子噪声不随大气透过率 %s~%s 变化 ⇒ B1；热噪声压过光子噪声 ⇒ B2" % (
             f(d["transmittance"].min), f(d["transmittance"].max))))
    add(("75", "判决模块输入 SNR 与门限", "（无独立判据，作判读依据）",
         "SNR %s~%s，门限 %s" % (f(d["snr"].min), f(d["snr"].max), f(d["thresholds"].max)),
         "-", "SNR 为能量比且 −31~−19 dB ⇒ 与「检测成立」不自洽（P2/P3）"))

    d = it["76"]
    required_76 = ["目标ID=", "候选量测时刻=", "候选测角=", "连续命中=", "引导来源卫星ID=",
                   "调度结果=", "指令方位/俯仰=", "候选状态=", "当前服务目标=",
                   "指向状态=", "剩余角=", "捕获失败原因="]
    missing = [m.rstrip("=") for m in required_76 if not d["fields"].get(m)]
    add(("76", "交换数据齐全性（12 项）", "交换数据必须齐全",
         "缺：%s" % ("、".join(missing) if missing else "无"),
         "通过" if not missing else "不通过", ""))
    add(("76", "连续命中状态", "只有满足宽场候选条件的目标可进入调度",
         "调度行 %d；取值分布 %s" % (
             d["sched_rows"], json.dumps({str(k): v for k, v in sorted(d["hits"].items())})),
         "不通过" if len(d["hits"]) == 1 and 1 in d["hits"] else
         ("无样本" if not d["hits"] else "通过"),
         "恒为 1 ⇒ 计数器从不累加（B9）"))
    add(("76", "引导来源卫星 ID", "跨星 cue 按来源 ID 和量测时间可追踪",
         json.dumps({str(k): v for k, v in sorted(d["cue_sources"].items())}),
         "通过" if any(k >= 0 for k in d["cue_sources"]) else "不通过",
         "恒 −1 ⇒ 本轮跨星 cue 无证据（需确认场景是否触发）"))
    add(("76", "ATP 未稳定不得伪装成 NFOV 已捕获", "未被选择或 ATP 未稳定的目标不得伪装成 "
         "NFOV 已捕获输出",
         "指向行 %d；候选状态=已捕获跟踪 %d；其中指向状态=转动中 %d" % (
             d["point_rows"], d["captured_in_turning"], d["turning_but_captured"]),
         "通过" if d["turning_but_captured"] == 0 else "不通过",
         "本项最硬的一条：残差 149.85° 仍报已捕获即违反"))
    add(("76", "剩余角（指向残差）", "指向稳定状态与候选状态必须自洽",
         "0 值 %d / %d 行；已稳定 %d/%d 为 0，转动中 %d/%d 为 0；max %s" % (
             d["residual_zero_stable"] + d["residual_zero_turning"], d["point_rows"],
             d["residual_zero_stable"], d["stable_rows"], d["residual_zero_turning"],
             d["turning_rows"], f(d["residual"].max)),
         "-", "「已稳定却恒 0」+「转动中残差 149.85°」⇒ 残差口径疑为指令−指令（B8）"))

    d = it["77"]
    required_77 = ["目标ID=", "量测时刻=", "预测角=", "角速度=", "输出阶段=", "帧数=",
                   "单帧/融合角噪声=", "滤波器量测输入=", "量测协方差=", "滤波输出角=",
                   "NIS=", "NIS门结果=", "跟踪状态="]
    missing = [m.rstrip("=") for m in required_77 if not d["fields"].get(m)]
    add(("77", "交换数据齐全性（13 项）", "交换数据必须齐全",
         "缺：%s" % ("、".join(missing) if missing else "无"),
         "通过" if not missing else "不通过", ""))
    add(("77", "角速度", "记录后验角速度",
         "不适用 %d · 零 %d · 有值 %d；不适用理由 %s；有值行方位分量 %s~%s deg/s" % (
             d["rate_na"], d["rate_zero"], d["rate_numeric"],
             json.dumps(d["rate_reasons"], ensure_ascii=False),
             f(d["rates"].min), f(d["rates"].max)),
         "通过" if d["rate_numeric"] else "不通过",
         "有值行的量级应与同行 `视线角速率` 同阶；恒 0 或恒不适用均为未派生"))
    add(("77", "多帧协方差按有效帧数缩放", "多帧协方差按有效帧数缩放",
         "%d / %d 行不满足（融合σ=单帧σ/√N，且 σ² 落入 2×2 协方差）" % (
             d["cov_mismatch"], d["cov_rows"]),
         "通过" if d["cov_rows"] and d["cov_mismatch"] == 0 else
         ("无样本" if not d["cov_rows"] else "不通过"), ""))
    add(("77", "NIS 门结果与 NIS 数值一致", "NIS 连续超门后释放锁定",
         "%d / %d 行不一致" % (d["nis_gate_mismatch"], d["nis_rows"]),
         "通过" if d["nis_gate_mismatch"] == 0 else "不通过",
         "门判决与数值自相矛盾 ⇒ B5，须先加诊断再改门限"))
    add(("77", "缺量测周期只能预测/滑行", "缺量测周期只能预测/滑行",
         "帧数=0 行 %d，其中仍判 `跟踪` %d 行" % (d["zero_frames"],
                                              d["zero_frames_tracking"]),
         "无样本" if d["zero_frames"] == 0 else
         ("通过" if d["zero_frames_tracking"] == 0 else "不通过"),
         "本轮无 帧数=0 样本（短轮次），B7 不可据此判定"))
    add(("77", "输出阶段分布", "（无独立判据）", json.dumps(d["stages"], ensure_ascii=False),
         "-", "上轮审计称恒 `NFOV跟踪`；本轮见分布"))

    return rows


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR)
    parser.add_argument("--out", default=None, help="write markdown to this file (UTF-8)")
    parser.add_argument("--json", default=None, help="write raw metrics JSON")
    args = parser.parse_args(argv)

    present = {}
    for key, name in LOGS.items():
        path = os.path.join(args.log_dir, name)
        if os.path.exists(path):
            present[key] = path
        else:
            sys.stderr.write("missing log: %s\n" % path)

    it = new_items()
    for key, path in present.items():
        SCANNERS[key](path, it)

    lines = []

    def emit(text):
        print(text)
        lines.append(text)

    emit("# 验收项基线（%s）" % args.log_dir)
    emit("")
    run = it["_run"]
    emit("本轮仿真周期区间：%s ~ %s（周期数 %s）" % (
        run["min_cycle"], run["max_cycle"],
        (run["max_cycle"] - run["min_cycle"] + 1) if run["max_cycle"] else "-"))
    emit("")
    emit("口径：指标名与判据取自 `docs/review/functional_acceptance_test_criteria_ocr.md`；"
         "「说明」列是解释与根因线索，不属判据；「-」表示该项无独立判据、仅作判读依据。")
    emit("")
    all_rows = build_rows(it)
    for item in ("46", "58", "62", "65", "67", "75", "76", "77"):
        emit("## 第 %s 项 %s（`%s`）" % (item, ITEM_TITLES[item], LOGS[ITEM_FILES[item]]))
        emit("")
        emit("| 判定指标（验收标准） | 判据要点 | 本轮证据 | 判读 | 说明 |")
        emit("|---|---|---|---|---|")
        for row_item, indicator, criterion, evidence, result, note in all_rows:
            if row_item == item:
                emit("| %s | %s | %s | %s | %s |" % (indicator, criterion, evidence, result,
                                                     note))
        emit("")
        verdicts = [r[4] for r in all_rows if r[0] == item]
        emit("小结：通过 %d · 不通过 %d · 无样本 %d · 仅解释 %d" % (
            verdicts.count("通过"), verdicts.count("不通过"), verdicts.count("无样本"),
            verdicts.count("-")))
        emit("")

    if args.json:
        payload = {}
        for key, value in it.items():
            payload[key] = {}
            for name, entry in value.items():
                if isinstance(entry, Stats):
                    payload[key][name] = {"min": entry.min, "max": entry.max,
                                          "distinct": entry.distinct}
                elif isinstance(entry, dict) and entry and isinstance(
                        next(iter(entry.values())), Stats):
                    payload[key][name] = {
                        sub: {"min": s.min, "max": s.max, "distinct": s.distinct}
                        for sub, s in entry.items()}
                else:
                    payload[key][name] = entry
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, default=str)

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
        print("markdown written to %s" % args.out)

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
