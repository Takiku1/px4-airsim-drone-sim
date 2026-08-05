#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
baseline_check.py - PX4 飞行日志基线回归检查 (Step 2)

离线对比新飞行的轨迹指标与基线(JSON)，按容忍度规则输出 PASS/WARN/FAIL。
兼容两种 CSV 格式:
  - 格式 A (MAVSDK 遥测, check_square.py): t, phase, north_m, east_m, down_m, vn, ve, vd
  - 格式 B (ulg 真值,   analyze_ulg.py):  x, y, z, timestamp, vx, vy

指标算法与 check_square.py / analyze_ulg.py 完全一致，因此两种方式对同一飞行
得到的方形度口径相同(~95.3)，避免"双格式口径不一致"导致的误判。

用法:
  python3 baseline_check.py --flight trajectory_xxx.csv [--baseline path/to/trajectory_baseline.json]

退出码:
  0 = 全部 PASS/WARN (无 FAIL)
  1 = 存在任意 FAIL (飞行质量退化, 应阻止合入)
  2 = 输入/数据异常
"""
import argparse
import csv
import json
import math
import os
import sys

# 让脚本可 import check_square / analyze_ulg (仓库根) 及同目录模块 (scripts/)
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                      # scripts/  (fault_asserts 等)
sys.path.insert(0, os.path.dirname(_HERE))    # 仓库根     (check_square.py / analyze_ulg.py)

from check_square import evaluate, LEG_PHASES  # noqa: E402
from analyze_ulg import detect_legs  # noqa: E402

DEFAULT_BASELINE = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "tests", "fixtures", "trajectory_baseline.json",
    )
)


def load_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def detect_format(rows):
    cols = set(rows[0].keys()) if rows else set()
    if "north_m" in cols and "phase" in cols:
        return "mavsdk"
    if {"x", "y", "z", "timestamp", "vx", "vy"} <= cols:
        return "ulg"
    return None


def compute_metrics_mavsdk(rows):
    """格式 A: 复用 check_square.evaluate() 的方形度/边长/闭合/高度std。"""
    r = evaluate(rows)
    # 巡航高度(取反 down_m) 均值
    heights = [-float(x["down_m"]) for x in rows if x["phase"] in LEG_PHASES]
    alt_mean = sum(heights) / len(heights) if heights else 0.0
    # 最大合速度
    spd = [math.hypot(float(x["vn"]), float(x["ve"]), float(x["vd"])) for x in rows]
    max_speed = max(spd) if spd else 0.0
    t0 = float(rows[0]["t"])
    t1 = float(rows[-1]["t"])
    duration = t1 - t0
    return {
        "squareness": r["score"],
        "mean_side": r["mean_side"],
        "side_dev": r["side_dev"],
        "closure": r["closure"],
        "alt_mean": alt_mean,
        "alt_std": r["hstd"],
        "max_speed": max_speed,
        "duration": duration,
    }


def compute_metrics_ulg(rows):
    """格式 B: 复用 analyze_ulg.detect_legs() + 同款评分公式。"""
    xs, ys, zs, ts, vxs, vys = [], [], [], [], [], []
    for row in rows:
        xs.append(float(row["x"]))
        ys.append(float(row["y"]))
        zs.append(float(row["z"]))
        ts.append(int(row["timestamp"]))
        vxs.append(float(row["vx"]))
        vys.append(float(row["vy"]))
    n = len(xs)
    hs = [-z for z in zs]
    air = [i for i in range(n) if hs[i] > 1.0]
    xmin = min(xs[i] for i in air)
    xmax = max(xs[i] for i in air)
    ymin = min(ys[i] for i in air)
    ymax = max(ys[i] for i in air)
    north_span = xmax - xmin
    east_span = ymax - ymin
    la = air[-1]
    closure = math.hypot(xs[la], ys[la])

    legs = detect_legs(xs, ys, hs, ts)
    leg_lens = [math.hypot(xs[e] - xs[s], ys[e] - ys[s]) for (s, e, _h) in legs]
    if len(leg_lens) >= 4:
        mean_side = sum(leg_lens) / 4.0
        leg_err = sum(abs(L - 10.0) for L in leg_lens) / len(leg_lens)
    else:
        mean_side = (north_span + east_span) / 2.0
        leg_err = (abs(north_span - 10.0) + abs(east_span - 10.0)) / 2.0
    side_dev = max(abs(L - mean_side) for L in leg_lens) if leg_lens else 0.0

    # 巡航高度(四腿覆盖区间)
    cruise = set()
    for (s, e, _h) in legs:
        cruise.update(range(s, e + 1))
    cruise = sorted(cruise)
    if not cruise:
        cruise = [i for i in range(n) if 4.0 <= hs[i] <= 6.0]
    am = sum(hs[i] for i in cruise) / len(cruise)
    astd = math.sqrt(sum((hs[i] - am) ** 2 for i in cruise) / len(cruise))

    spd = [math.hypot(vxs[i], vys[i]) for i in range(n)]
    max_speed = max(spd)
    duration = (ts[-1] - ts[0]) / 1e6

    # 方形度评分(与 analyze_ulg.run_analysis 同款)
    pen_side = leg_err / 10.0
    pen_clos = closure / 10.0
    pen_aspect = abs(north_span - east_span) / 10.0
    pen_alt = astd / 1.0
    score = 100 * (1 - 0.40 * pen_side - 0.30 * pen_clos
                   - 0.15 * pen_aspect - 0.15 * pen_alt)
    score = max(0.0, min(100.0, score))

    return {
        "squareness": score,
        "mean_side": mean_side,
        "side_dev": side_dev,
        "closure": closure,
        "alt_mean": am,
        "alt_std": astd,
        "max_speed": max_speed,
        "duration": duration,
    }


def evaluate_rules(metrics, baseline, tol):
    """返回 (rules_dict, drop, cinc, vinc)。rules: key->(status, detail)。"""
    rules = {}
    # 方形度: baseline - flight = drop (正=退化)
    drop = baseline["squareness"] - metrics["squareness"]
    if drop >= tol["squareness_drop_fail"]:
        rules["squareness"] = ("FAIL", "方形度下降 %.2f ≥ %.1f" % (drop, tol["squareness_drop_fail"]))
    elif drop >= tol["squareness_drop_warn"]:
        rules["squareness"] = ("WARN", "方形度下降 %.2f (%.1f~%.1f 区间)" % (drop, tol["squareness_drop_warn"], tol["squareness_drop_fail"]))
    else:
        rules["squareness"] = ("PASS", "方形度波动 %.2f < %.1f" % (drop, tol["squareness_drop_warn"]))

    # 闭合误差: flight - baseline = increase (正=退化)
    cinc = metrics["closure"] - baseline["closure"]
    if cinc >= tol["closure_increase_warn"]:
        rules["closure"] = ("WARN", "闭合误差增加 %.3f m ≥ %.2f m" % (cinc, tol["closure_increase_warn"]))
    else:
        rules["closure"] = ("PASS", "闭合误差变化 %+.3f m" % cinc)

    # 最大速度: flight - baseline = increase (正=退化)
    vinc = metrics["max_speed"] - baseline["max_speed"]
    if vinc >= tol["max_speed_increase_fail"]:
        rules["max_speed"] = ("FAIL", "最大速度增加 %.2f m/s ≥ %.1f m/s" % (vinc, tol["max_speed_increase_fail"]))
    else:
        rules["max_speed"] = ("PASS", "最大速度变化 %+.2f m/s" % vinc)

    return rules, drop, cinc, vinc


ICON = {"PASS": "✅", "WARN": "⚠️", "FAIL": "❌", "INFO": "ℹ️"}

LABELS = [
    ("squareness", "方形度(0-100)", "%.2f", "%.2f"),
    ("mean_side", "四边边长均值(m)", "%.3f", "%.3f"),
    ("side_dev", "边长偏差(m)", "%.3f", "%.3f"),
    ("closure", "闭合误差(m)", "%.3f", "%.3f"),
    ("alt_mean", "巡航高度均值(m)", "%.3f", "%.3f"),
    ("alt_std", "高度std(m)", "%.3f", "%.3f"),
    ("max_speed", "最大速度(m/s)", "%.3f", "%.3f"),
    ("duration", "飞行时长(s)", "%.2f", "%.2f"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flight", "--input", dest="flight", required=True,
                    help="新飞行轨迹 CSV (--flight / --input 均可)")
    ap.add_argument("--baseline", default=DEFAULT_BASELINE, help="基线 JSON")
    args = ap.parse_args()

    if not os.path.isfile(args.flight):
        print("[err] 飞行文件不存在: %s" % args.flight)
        sys.exit(2)
    if not os.path.isfile(args.baseline):
        print("[err] 基线文件不存在: %s" % args.baseline)
        sys.exit(2)

    with open(args.baseline, encoding="utf-8") as f:
        base = json.load(f)
    b = base["baseline"]
    tol_raw = base["tolerances"]
    tol = {
        "squareness_drop_warn": float(tol_raw["squareness_drop_warn"]),
        "squareness_drop_fail": float(tol_raw["squareness_drop_fail"]),
        "closure_increase_warn": float(tol_raw["closure_increase_warn"]),
        "max_speed_increase_fail": float(tol_raw["max_speed_increase_fail"]),
    }

    rows = load_csv(args.flight)
    fmt = detect_format(rows)
    if fmt is None:
        print("[err] 无法识别 CSV 格式(需含 north_m/phase 或 x/y/z/timestamp/vx/vy)")
        sys.exit(2)
    metrics = compute_metrics_mavsdk(rows) if fmt == "mavsdk" else compute_metrics_ulg(rows)

    rules, _drop, _cinc, _vinc = evaluate_rules(metrics, b, tol)

    # ---- 输出 ----
    print("=== PX4 飞行基线回归检查 ===")
    print("飞行文件: %s (格式: %s)" % (args.flight, fmt))
    print("基线文件: %s" % args.baseline)
    print()
    print("%-16s %10s %10s %11s  %s" % ("指标", "基线", "本次", "偏差", "判定"))
    print("-" * 70)
    fail_count = 0
    warn_count = 0
    for key, lab, bfmt, mfmt in LABELS:
        bval = b[key]
        mval = metrics[key]
        dev = mval - bval
        if key in rules:
            st, detail = rules[key]
            if st == "FAIL":
                fail_count += 1
            elif st == "WARN":
                warn_count += 1
            line = "%-16s %10s %10s %+11.3f  %s %s" % (
                lab, bfmt % bval, mfmt % mval, dev, ICON[st], detail)
        else:
            line = "%-16s %10s %10s %+11.3f  %s" % (
                lab, bfmt % bval, mfmt % mval, dev, ICON["INFO"])
        print(line)
    print()

    if fail_count > 0:
        print("### 结论: ❌ FAIL (%d 项失败, %d 项告警) — 飞行质量低于基线, 应阻止合入"
              % (fail_count, warn_count))
        sys.exit(1)
    elif warn_count > 0:
        print("### 结论: ⚠️ WARN (%d 项告警, 无失败) — 可合入但建议关注" % warn_count)
        sys.exit(0)
    else:
        print("### 结论: ✅ PASS — 飞行质量符合基线")
        sys.exit(0)


if __name__ == "__main__":
    main()
