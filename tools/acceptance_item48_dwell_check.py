#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cross-check the item-48 dwell points against the item-37 scan schedule.

Item 48 (`[波束扫描功能测试]`) writes the beam positions the radar actually
dwelled on this cycle; item 37 (`[空域自主搜索功能测试]`) writes how many
search dwells the scheduler handed out. They are two projections of the same
dwell plan, so the per-cycle counts must agree -- if they do not, the
acceptance logs describe two different scans.

What is checked, per cycle:

* search dwells: 48's `波位序号=` lines (legacy name `本周期序号=`) vs 37's
  `扫描调度信息=搜索=N`;
* the declared count line `本周期搜索驻留数=N` (2026-10-21 build onwards),
  which must equal both of the above;
* non-search dwells: 48's `驻留种类=` lines (指定驻留 / 外部跟踪驻留 /
  跟踪驻留) vs 37's 指定 / 跟踪 counters;
* the fixed slot budget: 搜索 + 指定 + 跟踪 must be the same on every cycle.

The beam-position table itself lives in `Rir_scan_pattern.csv`; when it is
readable the tool also reports its size, the implied frames per round and the
single-frame scan time, so the "dwell point count" is never guessed.

Usage:
    python tools/acceptance_item48_dwell_check.py
    python tools/acceptance_item48_dwell_check.py --log <RadarOutput.log>
    python tools/acceptance_item48_dwell_check.py --json <out.json>

Exit code is 0 only when every check passes.
"""

import argparse
import csv
import json
import os
import re
import sys

DEFAULT_LOG_DIR = r"C:\software\workspace\1qfosim\FOSimStudio\log"
ITEM48 = "[波束扫描功能测试]"
ITEM37 = "[空域自主搜索功能测试]"

RE_CYCLE = re.compile(r"仿真周期=(\d+)")
RE_SEARCH_INDEX = re.compile(r"波位序号=(\d+)")
RE_LEGACY_INDEX = re.compile(r"本周期序号=(\d+)")
RE_DECLARED_COUNT = re.compile(r"本周期搜索驻留数=(\d+)")
RE_DWELL_KIND = re.compile(r"驻留种类=(\S+)")
RE_SCHEDULE = re.compile(r"扫描调度信息=搜索=(\d+) 指定=(\d+) 跟踪=(\d+)")

KIND_DESIGNATE = "指定驻留"
KIND_EXTERNAL = "外部跟踪驻留"
KIND_INTERNAL = "跟踪驻留"


def detect_encoding(log_path, probe_bytes=65536):
  """Pick the log encoding: UTF-8 when it decodes cleanly, else GBK (host builds)."""
  with open(log_path, "rb") as stream:
    probe = stream.read(probe_bytes)
  # 探针可能正好切在汉字中间：去掉末尾几字节再试一次，避免把 UTF-8 误判成 GBK。
  for tail in (0, 4):
    chunk = probe[:len(probe) - tail] if tail else probe
    try:
      text = chunk.decode("utf-8")
    except UnicodeDecodeError:
      continue
    if "仿真周期" in text:
      return "utf-8"
  return "gbk"


def parse_round(log_path):
  """Stream one RadarOutput.log and collect per-cycle dwell counts."""
  searches = {}
  declared = {}
  kinds = {}
  schedule = {}
  legacy_index = False
  declared_seen = False
  total_lines = 0
  encoding = detect_encoding(log_path)

  with open(log_path, "r", encoding=encoding, errors="replace", newline="") as stream:
    for line in stream:
      total_lines += 1
      if ITEM48 in line:
        match = RE_CYCLE.search(line)
        if not match:
          continue
        cycle = int(match.group(1))
        kind = RE_DWELL_KIND.search(line)
        if kind:
          bucket = kinds.setdefault(cycle, {})
          bucket[kind.group(1)] = bucket.get(kind.group(1), 0) + 1
          continue
        declared_count = RE_DECLARED_COUNT.search(line)
        if declared_count:
          declared_seen = True
          declared[cycle] = int(declared_count.group(1))
          continue
        if RE_SEARCH_INDEX.search(line):
          searches[cycle] = searches.get(cycle, 0) + 1
        elif RE_LEGACY_INDEX.search(line):
          legacy_index = True
          searches[cycle] = searches.get(cycle, 0) + 1
      elif ITEM37 in line:
        match = RE_CYCLE.search(line)
        if not match:
          continue
        cycle = int(match.group(1))
        schedule_match = RE_SCHEDULE.search(line)
        if schedule_match:
          schedule[cycle] = (
              int(schedule_match.group(1)),
              int(schedule_match.group(2)),
              int(schedule_match.group(3)),
          )
  return {
      "lines": total_lines,
      "encoding": encoding,
      "searches": searches,
      "declared": declared,
      "kinds": kinds,
      "schedule": schedule,
      "legacy_index": legacy_index,
      "declared_seen": declared_seen,
  }


def read_pattern_size(csv_path):
  """Return (rows, distinct_az, distinct_el) or None when unreadable."""
  if not csv_path or not os.path.isfile(csv_path):
    return None
  rows = 0
  azimuths = set()
  elevations = set()
  with open(csv_path, "r", encoding="utf-8", errors="replace", newline="") as stream:
    reader = csv.DictReader(stream)
    if not reader.fieldnames:
      return None
    for record in reader:
      rows += 1
      azimuths.add(record.get("az_deg", ""))
      elevations.add(record.get("el_deg", ""))
  return rows, len(azimuths), len(elevations)


def verdict(ok):
  return "OK" if ok else "MISMATCH"


def main(argv):
  parser = argparse.ArgumentParser(
      description="Check item-48 dwell points against the item-37 scan schedule.")
  # 终端若是 GBK 代码页，个别符号无法编码；降级替换而不是整轮核对崩掉。
  if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")
  parser.add_argument("--log", default=os.path.join(DEFAULT_LOG_DIR, "RadarOutput.log"),
                      help="path to RadarOutput.log")
  parser.add_argument("--csv", default=os.path.join(DEFAULT_LOG_DIR, "Rir_scan_pattern.csv"),
                      help="path to Rir_scan_pattern.csv (beam-position table)")
  parser.add_argument("--dwell-sec", type=float, default=0.05,
                      help="per-dwell time in seconds (default 0.05 = budget / 20 slots)")
  parser.add_argument("--slots", type=int, default=20,
                      help="slot budget per cycle (default 20)")
  parser.add_argument("--show", type=int, default=10,
                      help="how many mismatching cycles to print")
  parser.add_argument("--json", dest="json_out", default=None,
                      help="also write the result as JSON")
  args = parser.parse_args(argv)

  if not os.path.isfile(args.log):
    print("missing log: %s" % args.log, file=sys.stderr)
    return 2

  data = parse_round(args.log)
  schedule = data["schedule"]
  searches = data["searches"]
  declared = data["declared"]
  kinds = data["kinds"]

  cycles = sorted(schedule)
  if not cycles:
    print("no item-37 schedule lines in %s" % args.log, file=sys.stderr)
    return 2

  # --- per-cycle search dwell counts -------------------------------------
  search_mismatch = []
  for cycle in cycles:
    have = searches.get(cycle, 0)
    want = schedule[cycle][0]
    if have != want:
      search_mismatch.append((cycle, have, want))

  # --- declared count line (new build) -----------------------------------
  declared_mismatch = []
  if data["declared_seen"]:
    for cycle in cycles:
      want = schedule[cycle][0]
      have = declared.get(cycle, -1)
      if have != want:
        declared_mismatch.append((cycle, have, want))

  # --- non-search dwell kinds -------------------------------------------
  designate_mismatch = []
  track_mismatch = []
  for cycle in cycles:
    bucket = kinds.get(cycle, {})
    designate = bucket.get(KIND_DESIGNATE, 0)
    track = bucket.get(KIND_EXTERNAL, 0) + bucket.get(KIND_INTERNAL, 0)
    if designate != schedule[cycle][1]:
      designate_mismatch.append((cycle, designate, schedule[cycle][1]))
    if track != schedule[cycle][2]:
      track_mismatch.append((cycle, track, schedule[cycle][2]))

  # --- fixed slot budget -------------------------------------------------
  totals = sorted({sum(value) for value in schedule.values()})

  total_search_37 = sum(value[0] for value in schedule.values())
  total_search_48 = sum(searches.get(cycle, 0) for cycle in cycles)
  total_designate_48 = sum(bucket.get(KIND_DESIGNATE, 0) for bucket in kinds.values())
  total_designate_37 = sum(value[1] for value in schedule.values())
  total_external = sum(bucket.get(KIND_EXTERNAL, 0) for bucket in kinds.values())
  total_internal = sum(bucket.get(KIND_INTERNAL, 0) for bucket in kinds.values())
  total_track_37 = sum(value[2] for value in schedule.values())

  pattern = read_pattern_size(args.csv)
  frames = None
  frame_seconds = None
  if pattern and total_search_37:
    frames = total_search_37 / float(pattern[0])
    frame_seconds = pattern[0] * args.dwell_sec

  ok = (not search_mismatch and not declared_mismatch and not designate_mismatch
        and not track_mismatch and len(totals) == 1)

  print("== 第 48 项（波束扫描）与第 37 项（空域自主搜索）驻留点数量核对 ==")
  print("日志      : %s（编码 %s）" % (args.log, data["encoding"]))
  print("周期数    : %d（第 37 项调度行 %d，第 48 项搜索驻留覆盖 %d）"
        % (len(cycles), len(schedule), len(searches)))
  if pattern:
    print("波位表    : %s  %d 个波位（方位 %d × 俯仰 %d）"
          % (os.path.basename(args.csv), pattern[0], pattern[1], pattern[2]))
  else:
    print("波位表    : 未读取（%s 不存在或为空）" % args.csv)
  if data["legacy_index"]:
    print("字段口径  : 旧构建（仍写 `本周期序号`；新构建写 `波位序号` + `本周期搜索驻留数`）")
  elif not data["declared_seen"]:
    print("字段口径  : 逐驻留行已用 `波位序号`，但缺少 `本周期搜索驻留数` 行")
  else:
    print("字段口径  : 新构建（`波位序号` + `本周期搜索驻留数`）")
  print()

  rows = [
      ("搜索驻留", "%d" % total_search_48, "%d" % total_search_37,
       verdict(not search_mismatch), "%d 处周期不一致" % len(search_mismatch)),
      ("指定驻留", "%d" % total_designate_48, "%d" % total_designate_37,
       verdict(not designate_mismatch), "%d 处周期不一致" % len(designate_mismatch)),
      ("跟踪驻留", "%d（外部 %d / 库内 %d）" % (total_external + total_internal,
                                              total_external, total_internal),
       "%d" % total_track_37, verdict(not track_mismatch),
       "%d 处周期不一致" % len(track_mismatch)),
  ]
  if data["declared_seen"]:
    rows.append(("本周期搜索驻留数", "%d 个周期有该行" % len(declared),
                 "%d" % total_search_37, verdict(not declared_mismatch),
                 "%d 处周期不一致" % len(declared_mismatch)))

  print("| 项目 | 第 48 项 | 第 37 项 | 判定 | 备注 |")
  print("|---|---|---|---|---|")
  for name, left, right, state, note in rows:
    print("| %s | %s | %s | %s | %s |" % (name, left, right, state, note))

  print()
  print("每周期驻留总数（搜索+指定+跟踪）: %s  %s"
        % ("/".join(str(value) for value in totals),
           "OK（与预算 %d 槽一致）" % args.slots if totals == [args.slots]
           else "MISMATCH（预期恒为 %d）" % args.slots))
  if frames is not None:
    search_per_cycle = total_search_37 / float(len(cycles))
    print("平均搜索驻留/周期: %.2f（%d 搜索驻留 ÷ %d 周期）"
          % (search_per_cycle, total_search_37, len(cycles)))
    print("扫描一帧所需周期数: %.1f（%d 波位 ÷ %.2f）"
          % (pattern[0] / search_per_cycle, pattern[0], search_per_cycle))
    print("扫描一帧所需时间  : %.1f s（%d 波位 × %.3f s）"
          % (frame_seconds, pattern[0], args.dwell_sec))
    print("本轮覆盖帧数      : %.2f 帧（%d 搜索驻留 ÷ %d 波位）"
          % (frames, total_search_37, pattern[0]))

  for label, mismatch in (("搜索驻留", search_mismatch),
                          ("本周期搜索驻留数", declared_mismatch),
                          ("指定驻留", designate_mismatch),
                          ("跟踪驻留", track_mismatch)):
    if not mismatch:
      continue
    print()
    print("前 %d 处%s不一致（周期: 第48项 / 第37项）:" % (args.show, label))
    for cycle, have, want in mismatch[:args.show]:
      print("  周期 %d: %d / %d" % (cycle, have, want))

  print()
  print("结论: %s" % ("一致" if ok else "存在不一致，见上表"))

  if args.json_out:
    with open(args.json_out, "w", encoding="utf-8") as out:
      json.dump({
          "log": args.log,
          "cycles": len(cycles),
          "search_total_48": total_search_48,
          "search_total_37": total_search_37,
          "designate_total_48": total_designate_48,
          "designate_total_37": total_designate_37,
          "track_total_48": total_external + total_internal,
          "track_total_37": total_track_37,
          "search_mismatch_cycles": [{"cycle": c, "item48": a, "item37": b}
                                     for c, a, b in search_mismatch],
          "declared_count_seen": data["declared_seen"],
          "declared_mismatch_cycles": [{"cycle": c, "item48": a, "item37": b}
                                       for c, a, b in declared_mismatch],
          "pattern_size": pattern[0] if pattern else None,
          "frame_seconds": frame_seconds,
          "ok": ok,
      }, out, ensure_ascii=False, indent=2)

  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
