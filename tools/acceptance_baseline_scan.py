#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Scan the acceptance logs of one simulation round and print its baseline table.

Motivation: every phase of docs/review/next_step_plan_2026-10-19.md has to prove
that exactly one variable changed. That requires the same handful of numbers
(noise components, SNR, thresholds, hit rates, NIS quantiles, out-of-envelope
tracks) to be re-read from the six logs after each round. The logs are hundreds
of MB, so they are streamed line by line and never loaded whole.

Usage:
    python tools/acceptance_baseline_scan.py
    python tools/acceptance_baseline_scan.py --log-dir <dir> --json <out.json>

Exit code is 0 whenever the scan completes; an empty metric is reported as "-"
rather than being treated as an error, because a missing log is itself evidence.
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


def num(pattern, line, cast=float):
    m = pattern.search(line)
    if m is None:
        return None
    try:
        return cast(m.group(1))
    except (TypeError, ValueError):
        return None


def percentile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (pos - low)


class Stats(object):
    """min / max / distinct tracker, matching the audit's constant-value test."""

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

    def as_dict(self):
        return {
            "min": self.min,
            "max": self.max,
            "distinct": self.distinct,
            "constant": self.distinct == 1,
        }


# -- regexes over one log line ------------------------------------------------
RE_SIM_TIME = re.compile(r"仿真时间=([\d.]+)s")
RE_PHOTON = re.compile(r"光子噪声=([-\d.eE+]+)J")
RE_THERMAL = re.compile(r"热噪声=([-\d.eE+]+)J")
RE_READOUT = re.compile(r"读出噪声=([-\d.eE+]+)J")
RE_NOISE_TOTAL = re.compile(r"有效噪声能量=([-\d.eE+]+)J")
RE_SNR = re.compile(r"判决模块输入SNR=([-\d.eE+]+)")
RE_THRESHOLD = re.compile(r"门限=([-\d.eE+]+)")
RE_SNR_GATE = re.compile(r"SNR门结果=(\S+)")
RE_TRANSMITTANCE = re.compile(r"大气透过率=([-\d.eE+]+)")
RE_STAGE = re.compile(r"阶段=(\S+)")
RE_OMEGA_LOS = re.compile(r"视线角速率=([-\d.eE+]+)deg")
RE_ANG_RATE = re.compile(r"角速度=(\S+)")
RE_FRAMES = re.compile(r"帧数=(\d+)")
RE_TRACK_STATE = re.compile(r"跟踪状态=(\S+)")
RE_NIS = re.compile(r"NIS=([-\d.eE+]+)")
RE_NIS_GATE = re.compile(r"NIS门结果=(\S+)")
RE_RESIDUAL = re.compile(r"剩余角=([-\d.eE+]+)deg")
RE_POINTING = re.compile(r"指向状态=(\S+)")
RE_HITS = re.compile(r"连续命中=(\d+)")
RE_CAPTURE_FAIL = re.compile(r"捕获失败原因=(\S+)")
RE_DETECT_FLAG = re.compile(r"检测标志=(\S+)")
RE_IR_SNR = re.compile(r"IR_SNR=([-\d.eE+]+)")
RE_RADIANCE = re.compile(r"红外辐射强度=([-\d.eE+]+)W/sr")
RE_ENU = re.compile(r"目标状态ENU_位置=\(([-\d.]+),([-\d.]+),([-\d.]+)\)m")
RE_CLASS = re.compile(r"目标类别=(\S+)")
RE_CONFIDENCE = re.compile(r"置信度=([-\d.eE+]+)")
RE_MASK = re.compile(r"特征有效掩码=(\d+)")
RE_POL_ANGLE = re.compile(r"本征极化方向角=\(([-\d.]+),([-\d.]+)\)")
RE_REQUIRED_HITS = re.compile(r"要求连续命中=(\d+)")
RE_RCS = re.compile(r"RCS=([-\d.eE+]+)m\^2")
RE_SWERLING = re.compile(r"Swerling=(\d+)")


def iter_lines(path):
    with open(path, "rb") as handle:
        for raw in handle:
            try:
                yield raw.decode("utf-8")
            except UnicodeDecodeError:
                yield raw.decode("gbk", errors="replace")


def scan_ir_internal(path, acc):
    det = acc["detection"]           # item 75
    trk = acc["tracking"]            # item 77
    pnt = acc["pointing"]            # item 76
    for line in iter_lines(path):
        if "连续命中=" in line:
            pnt["hit_rows"] += 1
            hits = num(RE_HITS, line, int)
            if hits is not None:
                pnt["hits"][hits] = pnt["hits"].get(hits, 0) + 1
            required = num(RE_REQUIRED_HITS, line, int)
            if required is not None:
                pnt["required_hits"].add(required)
        if "有效噪声能量=" in line:
            det["rows"] += 1
            if num(RE_SNR_GATE, line, str) == "通过":
                det["snr_gate_pass"] += 1
            for key, pat in (
                ("photon", RE_PHOTON),
                ("thermal", RE_THERMAL),
                ("readout", RE_READOUT),
                ("total", RE_NOISE_TOTAL),
                ("snr", RE_SNR),
                ("threshold", RE_THRESHOLD),
                ("transmittance", RE_TRANSMITTANCE),
            ):
                det[key].add(num(pat, line))
            stage = num(RE_STAGE, line, str)
            if stage:
                det["stages"][stage] = det["stages"].get(stage, 0) + 1
        elif "角速度=" in line and "NIS=" in line:
            trk["rows"] += 1
            trk["omega"].add(num(RE_OMEGA_LOS, line))
            rate = num(RE_ANG_RATE, line, str) or ""
            if rate.startswith("不适用"):
                trk["not_applicable"] += 1
                trk["reasons"][rate] = trk["reasons"].get(rate, 0) + 1
            elif rate == "(0.000000,0.000000)":
                trk["zero_rate"] += 1
            else:
                trk["numeric_rate"] += 1
            frames = num(RE_FRAMES, line, int)
            if frames is not None:
                if frames == 0:
                    trk["zero_frame_rows"] += 1
                    if num(RE_TRACK_STATE, line, str) == "跟踪":
                        trk["zero_frame_tracking_rows"] += 1
            nis = num(RE_NIS, line)
            if nis is not None:
                trk["nis_values"].append(nis)
            gate = num(RE_NIS_GATE, line, str)
            if gate:
                trk["nis_gate"][gate] = trk["nis_gate"].get(gate, 0) + 1
                if gate == "超门" and nis is not None:
                    trk["nis_over_gate_no_exceed"] += 1 if nis <= 5.991 else 0
                    trk["nis_over_gate_rows"] += 1
        elif "剩余角=" in line:
            pnt["rows"] += 1
            residual = num(RE_RESIDUAL, line)
            if residual is not None:
                if residual == 0.0:
                    pnt["zero_residual"] += 1
                pnt["residual"].add(residual)
            status = num(RE_POINTING, line, str)
            if status:
                pnt["status"][status] = pnt["status"].get(status, 0) + 1
            hits = num(RE_HITS, line, int)
            if hits is not None:
                pnt["hits"][hits] = pnt["hits"].get(hits, 0) + 1
            reason = num(RE_CAPTURE_FAIL, line, str)
            if reason:
                pnt["failure_reasons"][reason] = pnt["failure_reasons"].get(reason, 0) + 1


def scan_ir_external(path, acc):
    obs = acc["observation"]         # item 67
    scene = acc["scene"]             # item 58
    for line in iter_lines(path):
        if "检测标志=" in line:
            obs["rows"] += 1
            flag = num(RE_DETECT_FLAG, line, str)
            if flag:
                obs["flags"][flag] = obs["flags"].get(flag, 0) + 1
            obs["ir_snr"].add(num(RE_IR_SNR, line))
            stage = num(RE_STAGE, line, str)
            if stage:
                obs["stages"][stage] = obs["stages"].get(stage, 0) + 1
        elif "红外辐射强度=" in line:
            scene["rows"] += 1
            scene["radiance"].add(num(RE_RADIANCE, line))


def scan_radar_external(path, acc):
    trk = acc["track_state"]         # item 65
    scn = acc["scene_radar"]         # item 62
    pol = acc["polarization"]        # item 46
    for line in iter_lines(path):
        if "RCS=" in line and "Swerling=" in line:
            scn["rows"] += 1
            rcs = num(RE_RCS, line)
            if rcs is not None:
                scn["rcs"].add(rcs)
            swerling = num(RE_SWERLING, line, int)
            if swerling is not None:
                scn["swerling"][swerling] = scn["swerling"].get(swerling, 0) + 1
        if "本征极化方向角=" in line:
            pol["rows"] += 1
            m = RE_POL_ANGLE.search(line)
            if m and float(m.group(1)) == 0.0 and float(m.group(2)) == 0.0:
                pol["zero_angle"] += 1
        if "目标状态ENU_位置=" not in line:
            continue
        m = RE_ENU.search(line)
        if m is None:
            continue
        trk["rows"] += 1
        up = float(m.group(3))
        norm = math.sqrt(float(m.group(1)) ** 2 + float(m.group(2)) ** 2 + up ** 2)
        trk["up_max"] = up if trk["up_max"] is None else max(trk["up_max"], up)
        trk["up_min"] = up if trk["up_min"] is None else min(trk["up_min"], up)
        trk["norm_max"] = norm if trk["norm_max"] is None else max(trk["norm_max"], norm)
        if up > 2293000.0:
            trk["up_over_limit"] += 1


def scan_radar_output(path, acc):
    rec = acc["recognition"]         # item 47A
    pol = acc["polarization"]        # item 46
    for line in iter_lines(path):
        if "综合特征识别功能测试" in line:
            rec["rows"] += 1
            cls = num(RE_CLASS, line, str)
            if cls:
                rec["classes"][cls] = rec["classes"].get(cls, 0) + 1
            conf = num(RE_CONFIDENCE, line)
            if conf is not None:
                if conf == 0.0:
                    rec["zero_confidence"] += 1
                rec["confidence"].add(conf)
            mask = num(RE_MASK, line, int)
            if mask is not None:
                rec["masks"][mask] = rec["masks"].get(mask, 0) + 1
        elif "本征极化方向角=" in line:
            pol["rows"] += 1
            m = RE_POL_ANGLE.search(line)
            if m and float(m.group(1)) == 0.0 and float(m.group(2)) == 0.0:
                pol["zero_angle"] += 1


def fmt(value, digits=3):
    if value is None:
        return "-"
    if isinstance(value, float):
        if value == 0.0:
            return "0"
        if abs(value) >= 1.0e5 or abs(value) < 1.0e-3:
            return "%.3e" % value
        return ("%." + str(digits) + "f") % value
    return str(value)


def percent(part, whole):
    if not whole:
        return "-"
    return "%.2f%%" % (100.0 * part / whole)


def build_stats():
    return {
        "detection": {
            "rows": 0, "snr_gate_pass": 0, "stages": {},
            "photon": Stats(), "thermal": Stats(), "readout": Stats(), "total": Stats(),
            "snr": Stats(), "threshold": Stats(), "transmittance": Stats(),
        },
        "tracking": {
            "rows": 0, "not_applicable": 0, "zero_rate": 0, "numeric_rate": 0,
            "reasons": {}, "omega": Stats(), "zero_frame_rows": 0,
            "zero_frame_tracking_rows": 0, "nis_values": [], "nis_gate": {},
            "nis_over_gate_rows": 0, "nis_over_gate_no_exceed": 0,
        },
        "pointing": {
            "rows": 0, "zero_residual": 0, "residual": Stats(),
            "status": {}, "hit_rows": 0, "hits": {}, "required_hits": Stats(),
            "failure_reasons": {},
        },
        "observation": {"rows": 0, "flags": {}, "ir_snr": Stats(), "stages": {}},
        "scene": {"rows": 0, "radiance": Stats()},
        "scene_radar": {"rows": 0, "rcs": Stats(), "swerling": {}},
        "track_state": {
            "rows": 0, "up_min": None, "up_max": None, "norm_max": None,
            "up_over_limit": 0,
        },
        "recognition": {
            "rows": 0, "classes": {}, "zero_confidence": 0,
            "confidence": Stats(), "masks": {},
        },
        "polarization": {"rows": 0, "zero_angle": 0},
    }


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR)
    parser.add_argument("--json", default=None, help="write the raw metrics as JSON")
    parser.add_argument("--out", default=None, help="write the markdown table to this file (UTF-8)")
    args = parser.parse_args(argv)

    present = {}
    for key, name in LOGS.items():
        path = os.path.join(args.log_dir, name)
        if os.path.exists(path):
            present[key] = path
        else:
            sys.stderr.write("missing log: %s\n" % path)

    acc = build_stats()
    for key, path in present.items():
        if key == "ir_internal":
            scan_ir_internal(path, acc)
        elif key == "ir_external":
            scan_ir_external(path, acc)
        elif key == "radar_external":
            scan_radar_external(path, acc)
        elif key == "radar_output":
            scan_radar_output(path, acc)

    det, trk, pnt = acc["detection"], acc["tracking"], acc["pointing"]
    obs, scene, trkst = acc["observation"], acc["scene"], acc["track_state"]
    scn, rec, pol = acc["scene_radar"], acc["recognition"], acc["polarization"]

    nis = trk["nis_values"]
    nis_rows = len(nis)
    trk["nis"] = {
        "rows": nis_rows, "min": min(nis) if nis else None,
        "max": max(nis) if nis else None,
        "p50": percentile(nis, 0.50), "p90": percentile(nis, 0.90),
        "p99": percentile(nis, 0.99), "p999": percentile(nis, 0.999),
    }

    lines = []

    def emit(text):
        print(text)
        lines.append(text)

    emit("## Round baseline (%s)" % args.log_dir)
    emit("")
    emit("| # | metric | value |")
    emit("|---|---|---|")
    rows = [
        ("75", "检测判决行数", det["rows"]),
        ("75", "光子噪声 min / max / distinct",
         "%s / %s / %d" % (fmt(det["photon"].min), fmt(det["photon"].max), det["photon"].distinct)),
        ("75", "热噪声 min / max / distinct",
         "%s / %s / %d" % (fmt(det["thermal"].min), fmt(det["thermal"].max), det["thermal"].distinct)),
        ("75", "读出噪声 min / max", "%s / %s" % (fmt(det["readout"].min), fmt(det["readout"].max))),
        ("75", "有效噪声能量 min / max", "%s / %s" % (fmt(det["total"].min), fmt(det["total"].max))),
        ("75", "判决模块输入SNR min / max", "%s / %s" % (fmt(det["snr"].min), fmt(det["snr"].max))),
        ("75", "门限 distinct / 值", "%d / %s" % (det["threshold"].distinct, fmt(det["threshold"].max))),
        ("75", "SNR 门通过率", percent(det["snr_gate_pass"], det["rows"])),
        ("75", "大气透过率 min / max",
         "%s / %s" % (fmt(det["transmittance"].min), fmt(det["transmittance"].max))),
        ("75", "阶段分布", json.dumps(det["stages"], ensure_ascii=False)),
        ("58", "红外辐射强度 distinct / 值",
         "%d / %s" % (scene["radiance"].distinct, fmt(scene["radiance"].max))),
        ("67", "观测行数 / 检测标志分布",
         "%d / %s" % (obs["rows"], json.dumps(obs["flags"], ensure_ascii=False))),
        ("67", "IR_SNR min / max", "%s / %s" % (fmt(obs["ir_snr"].min), fmt(obs["ir_snr"].max))),
        ("77", "跟踪行数 / 角速度分类",
         "%d / 不适用 %d · 零 %d · 有效 %d" % (
             trk["rows"], trk["not_applicable"], trk["zero_rate"], trk["numeric_rate"])),
        ("77", "角速度不适用理由分布", json.dumps(trk["reasons"], ensure_ascii=False)),
        ("77", "视线角速率 min / max（有效行）",
         "%s / %s" % (fmt(trk["omega"].min), fmt(trk["omega"].max))),
        ("77", "NIS p50 / p90 / p99 / max",
         "%s / %s / %s / %s" % (fmt(trk["nis"]["p50"]), fmt(trk["nis"]["p90"]),
                                fmt(trk["nis"]["p99"]), fmt(trk["nis"]["max"]))),
        ("77", "NIS 门分布", json.dumps(trk["nis_gate"], ensure_ascii=False)),
        ("77", "超门行中 NIS≤5.991 占比",
         percent(trk["nis_over_gate_no_exceed"], trk["nis_over_gate_rows"])),
        ("77", "帧数=0 行数 / 其中仍判跟踪",
         "%d / %d" % (trk["zero_frame_rows"], trk["zero_frame_tracking_rows"])),
        ("62", "场景目标行数 / RCS distinct / 值区间",
         "%d / %d / %s~%s" % (scn["rows"], scn["rcs"].distinct,
                              fmt(scn["rcs"].min), fmt(scn["rcs"].max))),
        ("62", "Swerling 分布", json.dumps({str(k): v for k, v in sorted(scn["swerling"].items())})),
        ("76", "连续命中打印行数 / 分布",
         "%d / %s" % (pnt["hit_rows"], json.dumps({str(k): v for k, v in sorted(pnt["hits"].items())}))),
        ("76", "要求连续命中", fmt(pnt["required_hits"].max)),
        ("76", "指向行数 / 剩余角为 0 占比",
         "%d / %s" % (pnt["rows"], percent(pnt["zero_residual"], pnt["rows"]))),
        ("76", "剩余角 min / max", "%s / %s" % (fmt(pnt["residual"].min), fmt(pnt["residual"].max))),
        ("76", "指向状态分布", json.dumps(pnt["status"], ensure_ascii=False)),
        ("76", "捕获失败原因分布", json.dumps(pnt["failure_reasons"], ensure_ascii=False)),
        ("65", "航迹行数 / Up>2293km 占比",
         "%d / %s" % (trkst["rows"], percent(trkst["up_over_limit"], trkst["rows"]))),
        ("65", "Up min / max", "%s / %s" % (fmt(trkst["up_min"]), fmt(trkst["up_max"]))),
        ("65", "|ENU| max", fmt(trkst["norm_max"])),
        ("47A", "识别行数 / 类别分布",
         "%d / %s" % (rec["rows"], json.dumps(rec["classes"], ensure_ascii=False))),
        ("47A", "置信度为 0 占比", percent(rec["zero_confidence"], rec["rows"])),
        ("47A", "置信度 max", fmt(rec["confidence"].max)),
        ("47A", "特征掩码分布", json.dumps({str(k): v for k, v in sorted(rec["masks"].items())})),
        ("46", "极化行数 / 方向角恒 0 占比",
         "%d / %s" % (pol["rows"], percent(pol["zero_angle"], pol["rows"]))),
    ]
    for item, metric, value in rows:
        emit("| %s | %s | %s |" % (item, metric, value))

    if args.json:
        payload = {
            "log_dir": args.log_dir,
            "detection": dict(det, photon=det["photon"].as_dict(), thermal=det["thermal"].as_dict(),
                              readout=det["readout"].as_dict(), total=det["total"].as_dict(),
                              snr=det["snr"].as_dict(), threshold=det["threshold"].as_dict(),
                              transmittance=det["transmittance"].as_dict()),
            "tracking": dict(trk, omega=trk["omega"].as_dict(), nis_values=None),
            "pointing": dict(pnt, residual=pnt["residual"].as_dict(),
                             required_hits=pnt["required_hits"].as_dict()),
            "observation": dict(obs, ir_snr=obs["ir_snr"].as_dict()),
            "scene": dict(scene, radiance=scene["radiance"].as_dict()),
            "scene_radar": dict(scn, rcs=scn["rcs"].as_dict()),
            "track_state": trkst,
            "recognition": dict(rec, confidence=rec["confidence"].as_dict()),
            "polarization": pol,
        }
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        emit("")
        emit("raw metrics written to %s" % args.json)

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
        print("markdown table written to %s" % args.out)

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
