#!/usr/bin/env python3
"""
check_square.py - 评估 square_mission.py 输出的 MAVSDK 轨迹 CSV 的方形度。

读取列: t, phase, north_m, east_m, down_m, vn, ve, vd
按 phase(north/east/south/west) 提取四个角点坐标，计算:
    - 四边边长及边长偏差
    - 闭合误差 (最后角点回到起点距离)
    - 四个内角偏差 (应为 90°)
    - 巡航段高度稳定性 (down_m 标准差)
输出 0~100 的方形度评分 (复用 analyze_ulg.py 的加权方法论)。

用法:
    python3 check_square.py --input trajectory_xxx.csv --threshold 80
    python3 check_square.py --threshold 80        # 缺省 glob trajectory_*.csv

退出码: 评分 > threshold -> 0 (通过)；否则 -> 1 (失败)；数据缺失/异常 -> 2。
"""
import argparse
import csv
import glob
import math
import os
import sys

LEG_PHASES = ["north", "east", "south", "west"]


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def corner(rows, phase):
    """取某 phase 最后一行作为该腿终点(角点)。返回 (north, east, down)。"""
    pts = [r for r in rows if r["phase"] == phase]
    if not pts:
        return None
    last = pts[-1]
    return (float(last["north_m"]), float(last["east_m"]), float(last["down_m"]))


def evaluate(rows):
    start = corner(rows, "takeoff") or (0.0, 0.0, 0.0)
    P = {}
    for ph in LEG_PHASES:
        c = corner(rows, ph)
        if c is None:
            raise ValueError("缺少相位 '%s' 的角点数据" % ph)
        P[ph] = c

    # 角点顺序: start -> north -> east -> south -> west(应回 start)
    pts = [start] + [P[k] for k in LEG_PHASES]  # pts[0..4], pts[4]=west 终点
    west_end = pts[4]

    # 四边形四角: start, north, east, south (west 终点应≈start, 仅用于闭合)
    quad = [start, P["north"], P["east"], P["south"]]

    # 四边 + 闭合 (闭环边 = south -> start, 即 west 腿回原点)
    sidelens = []
    for i in range(4):
        a, b = quad[i], quad[(i + 1) % 4]
        sidelens.append(math.hypot(b[0] - a[0], b[1] - a[1]))
    mean_side = sum(sidelens) / 4.0
    side_dev = max(abs(s - mean_side) for s in sidelens)
    min_s, max_s = min(sidelens), max(sidelens)
    aspect = max_s / min_s if min_s > 1e-6 else 99.0

    # 闭合误差: 最后角点(west) 距起点
    closure = math.hypot(west_end[0] - start[0], west_end[1] - start[1])

    # 四个内角偏差 (四边形四角, 应均为 90°)
    max_ang_dev = 0.0
    for i in range(4):
        v = quad[i]
        a = quad[(i - 1) % 4]  # 前一角
        b = quad[(i + 1) % 4]  # 后一角
        vin = (a[0] - v[0], a[1] - v[1])  # 入边反向
        vout = (b[0] - v[0], b[1] - v[1])  # 出边
        dot = vin[0] * vout[0] + vin[1] * vout[1]
        cross = vin[0] * vout[1] - vin[1] * vout[0]
        ang = math.degrees(math.atan2(abs(cross), dot))
        max_ang_dev = max(max_ang_dev, abs(ang - 90.0))

    # 高度稳定性: 四腿巡航段 down_m 的 std
    heights = [float(r["down_m"]) for r in rows if r["phase"] in LEG_PHASES]
    if heights:
        hm = sum(heights) / len(heights)
        hstd = math.sqrt(sum((h - hm) ** 2 for h in heights) / len(heights))
    else:
        hm, hstd = 0.0, 99.0

    # ---- 评分 (复用 analyze_ulg.py 方法论: 边/闭合/长宽/夹角/高度 加权) ----
    pen_side = min(1.0, side_dev / mean_side) if mean_side > 1e-6 else 1.0
    pen_clos = min(1.0, closure / mean_side) if mean_side > 1e-6 else 1.0
    pen_aspect = min(1.0, (max_s - min_s) / mean_side) if mean_side > 1e-6 else 1.0
    pen_angle = min(1.0, max_ang_dev / 90.0)
    pen_alt = min(1.0, hstd / 1.0)  # 高度 std > 1m 视为不稳
    score = 100 * (
        1
        - 0.35 * pen_side
        - 0.25 * pen_clos
        - 0.15 * pen_aspect
        - 0.15 * pen_angle
        - 0.10 * pen_alt
    )
    score = max(0.0, min(100.0, score))

    return {
        "score": score,
        "mean_side": mean_side,
        "side_dev": side_dev,
        "aspect": aspect,
        "closure": closure,
        "max_ang_dev": max_ang_dev,
        "hstd": hstd,
        "sidelens": sidelens,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--input",
        default=None,
        help="MAVSDK 轨迹 CSV；缺省则 glob 当前目录 trajectory_*.csv",
    )
    ap.add_argument("--threshold", type=float, default=80.0)
    args = ap.parse_args()

    if args.input:
        path = args.input
    else:
        matches = sorted(glob.glob("trajectory_*.csv"))
        if not matches:
            print("[err] 未找到 trajectory_*.csv")
            sys.exit(2)
        path = matches[-1]

    if not os.path.isfile(path):
        print("[err] 文件不存在: %s" % path)
        sys.exit(2)

    rows = load(path)
    try:
        r = evaluate(rows)
    except ValueError as e:
        print("[err] %s" % e)
        sys.exit(1)

    print("[评估] 文件: %s" % path)
    print(
        "[评估] 四边边长 = %s m, 均值 %.2f m"
        % ([round(s, 2) for s in r["sidelens"]], r["mean_side"])
    )
    print(
        "[评估] 边长偏差 = %.2f m, 长宽比 = %.3f"
        % (r["side_dev"], r["aspect"])
    )
    print(
        "[评估] 闭合误差 = %.2f m, 最大夹角偏差 = %.1f°"
        % (r["closure"], r["max_ang_dev"])
    )
    print("[评估] 高度 std = %.3f m" % r["hstd"])
    print(
        "[评估] 方形度评分 = %.1f / 100  (阈值 %.0f)"
        % (r["score"], args.threshold)
    )
    if r["score"] > args.threshold:
        print("[PASS] 方形度 %.1f > %.0f ✓" % (r["score"], args.threshold))
        sys.exit(0)
    else:
        print("[FAIL] 方形度 %.1f 未达 %.0f ✗" % (r["score"], args.threshold))
        sys.exit(1)


if __name__ == "__main__":
    main()
