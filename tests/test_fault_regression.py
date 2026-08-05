"""PX4 故障注入遥测回归测试 (golden fixtures)。

每个故障用例在真实 PX4 SITL 上跑出的遥测 CSV 被固化为 golden fixture
(见 tests/fixtures/fault/fault_case{1..5}.csv)。本测试用 fault_asserts 中的纯函数
反复校验: 注入故障后 PX4 的 failsafe 行为是否符合预期。这样即使不拉起真实 SITL,
CI 也能守住"故障注入断言逻辑没被改坏"这一回归门槛。

触发: 与正常飞行门禁并列, 由 sim-flight.yml / fault-regression-selfhosted.yml 的
单元测试步骤执行 (python3 -m pytest -q tests/)。
"""
import csv
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# fault_asserts 在 scripts/, 无 mavsdk 依赖, 可在此直接 import
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import fault_asserts as fa  # noqa: E402

ALT = 5.0
FIX = os.path.join(ROOT, "tests", "fixtures", "fault")


def _load(case):
    path = os.path.join(FIX, f"fault_case{case}.csv")
    assert os.path.exists(path), f"缺少 golden fixture: {path}"
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "t": float(r["t"]),
                "phase": (r["phase"] or "").strip(),
                "mode": (r["mode"] or "").strip(),
                "north_m": float(r["north_m"]),
                "east_m": float(r["east_m"]),
                "down_m": float(r["down_m"]),
                "vn": float(r["vn"]),
                "ve": float(r["ve"]),
                "vd": float(r["vd"]),
            })
    return rows


def _find_inj_idx(rows, case):
    """定位注入时刻在遥测中的行号 (纯启发式, 与真实注入点对齐)。"""
    if case in (2, 3):
        # 注入发生在 edge1_north 之后、edge2_east 之前; 用 phase 边界定位
        # (这类用例注入不改变飞行模式, 模式全程保持 OFFBOARD)
        last_edge1 = -1
        for i, r in enumerate(rows):
            if r["phase"] == "edge1_north":
                last_edge1 = i
        return last_edge1 + 1 if last_edge1 >= 0 else 0
    # case 1/4/5: 注入后模式离开 OFFBOARD (进入 LAND/RETURN)。
    # 跳过起飞前的初始 HOLD/空模式: 先定位首个 OFFBOARD, 再找其后首个非 OFFBOARD。
    first_offboard = None
    for i, r in enumerate(rows):
        if r["mode"] == "OFFBOARD":
            first_offboard = i
            break
    if first_offboard is None:
        return 0
    for i in range(first_offboard, len(rows)):
        if rows[i]["mode"] != "OFFBOARD":
            return i
    return len(rows) - 1


def test_case1_gps_loss_blind_land():
    rows = _load(1)
    idx = _find_inj_idx(rows, 1)
    passed, detail = fa.assert_gps(rows, idx, ALT)
    assert passed, f"case1 GPS 丢失应触发 blind land: {detail}"


def test_case2_baro_failure_keeps_flying():
    rows = _load(2)
    idx = _find_inj_idx(rows, 2)
    passed, detail = fa.assert_keep_flying(rows, idx, "baro")
    assert passed, f"case2 气压计故障应保持飞行且高度有效: {detail}"


def test_case3_mag_failure_keeps_flying():
    rows = _load(3)
    idx = _find_inj_idx(rows, 3)
    passed, detail = fa.assert_keep_flying(rows, idx, "mag")
    assert passed, f"case3 磁力计故障应保持飞行且高度有效: {detail}"


def test_case4_link_loss_blind_land_in_place():
    rows = _load(4)
    idx = _find_inj_idx(rows, 4)
    passed, detail = fa.assert_link_loss(rows, idx, ALT)
    assert passed, f"case4 链路丢失应原地 blind land: {detail}"


def test_case5_geofence_return():
    rows = _load(5)
    idx = _find_inj_idx(rows, 5)
    passed, detail = fa.assert_geofence(rows, idx)
    assert passed, f"case5 地理围栏越界应触发 RETURN: {detail}"


def test_all_five_golden_fixtures_present():
    for c in (1, 2, 3, 4, 5):
        assert os.path.exists(os.path.join(FIX, f"fault_case{c}.csv")), \
            f"缺少 fault_case{c}.csv golden fixture"
