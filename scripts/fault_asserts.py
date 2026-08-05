#!/usr/bin/env python3
"""PX4 故障注入断言 (纯函数, 无 mavsdk 依赖)。

从 fault_test.py 抽离, 目的: pytest 回归测试能在无 mavsdk 的环境
(如 GitHub ubuntu-latest) 直接 import 这些断言, 对"黄金"故障遥测 CSV
反复校验 PX4 failsafe 行为, 而无需拉起真实 PX4 SITL。

输入 rec.rows 格式: list[dict], 每 dict 含
  t, phase, mode, north_m, east_m, down_m, vn, ve, vd
(down_m 为负表示高于 home; 高度 = -down_m)
"""
import math


def find_injection_idx(rows, t_inj):
    # 返回第一个 t >= t_inj 的行(注入时刻), 不是最后一个
    for i, r in enumerate(rows):
        if r["t"] >= t_inj:
            return i
    return len(rows) - 1


def assert_gps(rows, inj_idx, alt):
    after = rows[inj_idx:]
    if not after:
        return False, "注入后无遥测"
    modes = {r["mode"] for r in after}
    left = any(m and m != "OFFBOARD" for m in modes)
    alt_inj = -rows[inj_idx]["down_m"]
    min_alt = min(-r["down_m"] for r in after)
    descended = (alt_inj - min_alt) > 1.0
    n0, e0 = rows[inj_idx]["north_m"], rows[inj_idx]["east_m"]
    max_horiz = max(math.hypot(r["north_m"] - n0, r["east_m"] - e0) for r in after)
    ok = left and descended
    return ok, (f"离开offboard={left} 注入高度={alt_inj:.2f}m 最低={min_alt:.2f}m "
                f"下降={(alt_inj-min_alt):.2f}m 最大水平位移={max_horiz:.2f}m")


def assert_keep_flying(rows, inj_idx, label):
    after = rows[inj_idx:]
    if not after:
        return False, "注入后无遥测"
    modes = [r["mode"] for r in after if r["mode"]]
    failsafe = any(m in ("LAND", "RETURN") for m in modes)
    n0, e0 = rows[inj_idx]["north_m"], rows[inj_idx]["east_m"]
    path = 0.0
    px, py = n0, e0
    for r in after:
        path += math.hypot(r["north_m"] - px, r["east_m"] - py)
        px, py = r["north_m"], r["east_m"]
    alts = [-r["down_m"] for r in after]
    alt_valid = all(0.0 < a < 20.0 for a in alts)
    ok = (not failsafe) and path > 3.0 and alt_valid
    return ok, (f"进入failsafe={failsafe} 继续飞行路径={path:.2f}m 高度有效={alt_valid} "
                f"(末模式={modes[-1] if modes else '-'})")


def assert_link_loss(rows, inj_idx, alt):
    after = rows[inj_idx:]
    if not after:
        return False, "注入后无遥测"
    modes = {r["mode"] for r in after}
    left = any(m and m != "OFFBOARD" for m in modes)
    n0, e0 = rows[inj_idx]["north_m"], rows[inj_idx]["east_m"]
    max_horiz = max(math.hypot(r["north_m"] - n0, r["east_m"] - e0) for r in after)
    alt_inj = -rows[inj_idx]["down_m"]
    max_alt = max(-r["down_m"] for r in after)
    min_alt = min(-r["down_m"] for r in after)
    not_climb = (max_alt - alt_inj) < 0.8
    descended = (alt_inj - min_alt) > 1.0
    ok = left and max_horiz < 2.5 and not_climb and descended
    return ok, (f"离开offboard={left} 最大水平位移={max_horiz:.2f}m "
                f"注入高度={alt_inj:.2f}m 最高={max_alt:.2f}m 最低={min_alt:.2f}m "
                f"下降={(alt_inj-min_alt):.2f}m 不爬升={not_climb}")


def assert_geofence(rows, inj_idx):
    after = rows[inj_idx:]
    if not after:
        return False, "越界后无遥测"
    modes = [r["mode"] for r in after if r["mode"]]
    entered_return = any(m == "RETURN" for m in modes) or any(m and m != "OFFBOARD" for m in modes)
    peak = max(after, key=lambda r: math.hypot(r["north_m"], r["east_m"]))
    pi = after.index(peak)
    tail = after[pi + 1:]
    homeward = False
    if tail:
        dist_end = math.hypot(tail[-1]["north_m"], tail[-1]["east_m"])
        dist_peak = math.hypot(peak["north_m"], peak["east_m"])
        homeward = dist_end < dist_peak
    ok = entered_return
    return ok, (f"进入RETURN/离开offboard={entered_return} 峰值距离={math.hypot(peak['north_m'],peak['east_m']):.1f}m "
                f"末距离HOME={math.hypot(tail[-1]['north_m'],tail[-1]['east_m']):.1f}m 朝家={homeward} "
                f"(末模式={modes[-1] if modes else '-'})")
