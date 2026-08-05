#!/usr/bin/env python3
"""
PX4 故障注入与容错测试套件 (B 项目主脚本)

在 SIH (sihsim_quadx) + MAVSDK offboard 下注入 5 类故障，断言 PX4 failsafe 行为：
  1 GPS 丢失     -> blind land        (高度下降 + 水平基本不动)
  2 气压计故障   -> EKF 回落 GPS 高度  (继续飞 + 高度仍有效)
  3 磁力计故障   -> EKF 降级           (继续飞 + 位置稳)
  4 链路丢失     -> 数据链路失效保护    (离开 offboard + 不水平移动 + 不爬升 + 高度下降)
                   实现: 控制链路跑独立子进程, 主进程 SIGKILL 它 = 真实断链
  5 地理围栏越界 -> RETURN             (模式切 RETURN + 朝 HOME 返航)

前置: PX4 SIH 已在运行并在 14540 发 MAVLink; SYS_FAILURE_EN=1 (本脚本自动设置)。
断言失败 exit 1, 全部通过 exit 0。每个用例的遥测写独立 CSV 供 pytest 回归读取。

用法:
    python3 fault_test.py --case 1 [--addr udp://:14540] [--out-dir ...]
"""
import argparse
import asyncio
import csv
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.failure import FailureUnit, FailureType

# 断言逻辑抽离到 fault_asserts (纯函数, 无 mavsdk 依赖), 方便 pytest 在无 mavsdk
# 环境(如 GitHub ubuntu-latest)直接 import 做故障遥测回归。脚本目录已自动在 sys.path,
# 但显式插入一次, 避免被其它路径顺序干扰。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fault_asserts import (assert_gps, assert_keep_flying, assert_link_loss,
                           assert_geofence, find_injection_idx)

DEFAULT_ADDR = os.environ.get("MAVSDK_ADDR", "udp://:14540")
# case 4 用 GCS 遥测链路单独记录(与控制链路 14540 分离, 才能真实模拟断链)
# PX4 GCS 实例: local 18570 / remote 14550, 遥测发往 14550 -> 客户端监听 14550 收遥测
TELEM_ADDR = os.environ.get("MAVSDK_TELEM_ADDR", "udp://:14550")
if os.path.isdir("/mnt/d/AirSim/mission"):
    LOG_DIR = "/mnt/d/AirSim/mission/fault_cases"
else:
    LOG_DIR = os.path.expanduser("~/airsim_fault_cases")


class Recorder:
    """后台订阅位置 + 飞行模式遥测, 写入 CSV; 对外暴露最新值用于判定。"""

    def __init__(self, drone, csv_path):
        self.drone = drone
        self.csv_path = csv_path
        self.rows = []
        self._tasks = []
        self._stop = False
        self.phase = "init"
        self.latest = None          # (north, east, down)
        self.latest_mode = None     # str

    async def _pos(self):
        t0 = time.time()
        async for pv in self.drone.telemetry.position_velocity_ned():
            if self._stop:
                break
            p, v = pv.position, pv.velocity
            self.latest = (p.north_m, p.east_m, p.down_m)
            self.rows.append({
                "t": round(time.time() - t0, 3),
                "phase": self.phase,
                "mode": self.latest_mode or "",
                "north_m": round(p.north_m, 3),
                "east_m": round(p.east_m, 3),
                "down_m": round(p.down_m, 3),
                "vn": round(v.north_m_s, 3),
                "ve": round(v.east_m_s, 3),
                "vd": round(v.down_m_s, 3),
            })

    async def _mode(self):
        async for m in self.drone.telemetry.flight_mode():
            if self._stop:
                break
            self.latest_mode = str(m)

    def start(self):
        self._tasks = [asyncio.ensure_future(self._pos()),
                       asyncio.ensure_future(self._mode())]

    async def wait_first_fix(self, timeout=15):
        deadline = time.time() + timeout
        while self.latest is None and time.time() < deadline:
            await asyncio.sleep(0.2)
        return self.latest is not None

    async def stop(self):
        self._stop = True
        for t in self._tasks:
            t.cancel()
            try:
                await t
            except BaseException:
                pass
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, "w", newline="") as f:
            w = csv.DictWriter(
                f, fieldnames=["t", "phase", "mode", "north_m", "east_m", "down_m", "vn", "ve", "vd"])
            w.writeheader()
            w.writerows(self.rows)
        print(f"[rec ] {len(self.rows)} 条遥测 -> {self.csv_path}")


async def wait_connected(drone, addr, timeout=60):
    print(f"[conn] 连接 {addr} (超时 {timeout}s) ...")
    await drone.connect(system_address=addr)
    deadline = time.time() + timeout
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[conn] 已连接到飞控")
            return
        if time.time() > deadline:
            raise RuntimeError(f"连接超时 {timeout}s: 未收到 is_connected 心跳")
    raise RuntimeError("连接失败")


async def set_param(drone, name, value, as_int=False):
    """设置参数, 按候选方法名探测 (不同 mavsdk 版本 API 名不一)。"""
    if as_int:
        cands = ["set_param_int", "set_param_int32", "set_int_param"]
    else:
        cands = ["set_param_float", "set_float_param"]
    last = None
    for c in cands:
        fn = getattr(drone.param, c, None)
        if fn is None:
            continue
        try:
            if as_int:
                await fn(name, int(value))
            else:
                await fn(name, float(value))
            print(f"[cfg ] {name} = {value}")
            return
        except Exception as e:
            last = e
    print(f"[warn] {name} 设置失败(可忽略): {last}")


async def set_velocity_limits(drone, xy_max=2.0, xy_cruise=1.5, z_max=1.5):
    for name, val in {"MPC_XY_VEL_MAX": xy_max, "MPC_XY_CRUISE": xy_cruise,
                      "MPC_Z_VEL_MAX_DN": z_max, "MPC_Z_VEL_MAX_UP": z_max}.items():
        try:
            await set_param(drone, name, val, as_int=False)
        except Exception as e:
            print(f"[warn] {name} 设置失败(可忽略): {e}")


async def goto(drone, rec, north, east, alt, label, tol, timeout, hold):
    rec.phase = label
    target_down = -alt
    print(f"[fly ] {label:6s} -> N={north:6.1f} E={east:6.1f} alt={alt:.1f}", end="", flush=True)
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, target_down, 0.0))
    t0 = time.time()
    reached = None
    while True:
        await asyncio.sleep(0.2)
        el = time.time() - t0
        if rec.latest is not None:
            n, e, d = rec.latest
            if math.hypot(n - north, e - east) < tol and abs(d - target_down) < tol * 2:
                if reached is None:
                    reached = time.time()
                if time.time() - reached >= hold:
                    print(f"   到位 {el:5.1f}s")
                    return True
            else:
                reached = None
        if el > timeout:
            print(f"   !! 超时 {timeout:.0f}s (继续)")
            return False


async def safe(coro, label, t=12):
    """带超时的动作包装, 避免已降落/异常态下 MAVSDK 调用无限阻塞。"""
    try:
        return await asyncio.wait_for(coro, timeout=t)
    except Exception as e:
        print(f"[warn] {label} 超时/异常(忽略): {e}")
        return None


# ---------------------------------------------------------------------------
# case 4 专用: 控制链路跑在独立子进程, 主进程用 GCS 链路(18570)只记录遥测。
# 注入 = SIGKILL 控制子进程 -> PX4 收不到 setpoint -> offboard 超时 ->
# COM_OBL_RC_ACT(=4 Land) 失效保护 -> 高度下降(blind land)。
# ---------------------------------------------------------------------------
def _kill_pub(pub):
    """强杀控制子进程及其整个进程树(含 mavsdk_server 子进程)。

    mavsdk_server 由 System() 以 subprocess 方式拉起, 持有真正的 MAVLink UDP 套接字
    与 offboard setpoint 循环。仅 SIGKILL python 主进程会把它孤儿化, 它继续发 setpoint
    -> PX4 永远收得到"新鲜" setpoint -> offboard 超时失效保护不触发。因此必须连它的
    子进程(mavsdk_server)一起杀。"""
    pid = pub.pid
    # 1) 杀直接子进程(mavsdk_server 默认是 publisher 的直属子进程)
    try:
        out = subprocess.check_output(["pgrep", "-P", str(pid)]).decode().split()
        for c in out:
            c = c.strip()
            if c.isdigit():
                try:
                    os.kill(int(c), signal.SIGKILL)
                except Exception:
                    pass
    except Exception:
        pass
    # 2) 杀整个进程组(publisher 以 start_new_session 成为会话首进程, pgid==pid)
    try:
        pgid = os.getpgid(pid)
        os.killpg(pgid, signal.SIGKILL)
    except Exception:
        try:
            pub.kill()
        except Exception:
            pass


async def run_publisher(addr, out_dir, alt, side, tol, seg_timeout, hold):
    """case 4 的 offboard 控制子进程: 解锁 -> offboard -> 起飞 -> 飞到 edge1 ->
    然后空转(MAVSDK 自动重发最后一条 setpoint 维持 OFFBOARD), 直到被父进程 SIGKILL。
    被杀 = 控制链路真实丢失。"""
    # 独立 mavsdk_server 端口(50052): 让 mavsdk_server 成为本进程直属子进程,
    # 父进程 SIGKILL 本进程时可一并杀掉它(否则被孤儿化的 mavsdk_server 会继续往
    # PX4 发 setpoint, 使 offboard 超时失效保护永不触发)。
    drone = System(port=50052)
    await wait_connected(drone, addr)
    await set_velocity_limits(drone)
    rec_pub = Recorder(drone, os.path.join(out_dir, "_publisher_tmp.csv"))
    rec_pub.start()
    if not await rec_pub.wait_first_fix():
        print("[warn] publisher: 15s 内未收到位置遥测")
    if not await arm_with_retry(drone, rec_pub):
        await rec_pub.stop()
        return
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -alt, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[err ] publisher offboard 启动失败: {e}")
        await drone.action.disarm()
        await rec_pub.stop()
        return
    await goto(drone, rec_pub, 0.0, 0.0, alt, "takeoff", tol, seg_timeout, hold)
    await goto(drone, rec_pub, side, 0.0, alt, "edge1", tol, seg_timeout, hold)
    print("[pub ] 已到达 edge1, MAVSDK 持续重发 setpoint 维持 OFFBOARD, 等待被 kill ...")
    while True:
        await asyncio.sleep(5)


async def run_case4(telem, case, alt, side, tol, seg_timeout, hold, out_dir):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"fault_case{case}_{stamp}.csv")
    await wait_connected(telem, TELEM_ADDR)
    rec = Recorder(telem, csv_path)
    rec.start()
    if not await rec.wait_first_fix():
        print("[warn] 15s 内未在 GCS 链路收到位置遥测(检查 18570 端口)")
    # 启动控制子进程(连 14540)
    print("[pub ] 启动 offboard 控制子进程 (连 14540) ...")
    pub = await asyncio.create_subprocess_exec(
        sys.executable, os.path.abspath(__file__),
        "--publisher", "--case", "4",
        "--addr", "udp://:14540", "--out-dir", out_dir,
        "--alt", str(alt), "--side", str(side),
        start_new_session=True)
    # 等待进入 OFFBOARD 且达标高度
    ready = False
    deadline = time.time() + 55
    while time.time() < deadline:
        await asyncio.sleep(0.5)
        if rec.latest_mode == "OFFBOARD" and rec.latest and (-rec.latest[2]) > 4.0:
            ready = True
            break
    if not ready:
        print(f"[err ] 控制子进程未在超时内使无人机进入 OFFBOARD (mode={rec.latest_mode})")
        _kill_pub(pub)
        await safe(rec.stop(), "rec.stop")
        return 1, "case4: 控制子进程未进入 OFFBOARD", csv_path
    # 设失效保护: offboard 丢失 -> Land
    await set_param(telem, "COM_OBL_RC_ACT", 4, as_int=True)
    # 注意 COM_OF_LOSS_T 是 float 参数! 用 as_int=True 会 TYPE_MISMATCH 导致设置失败,
    # 停在默认 0 = 禁用 offboard 丢失检测 -> PX4 永不判链路丢失 -> 一直停在 OFFBOARD 不下降。
    await set_param(telem, "COM_OF_LOSS_T", 2.0, as_int=False)
    inj_idx_holder = {"t": rec.rows[-1]["t"] if rec.rows else 0.0}
    print("[inj ] SIGKILL 控制子进程(含 mavsdk_server) -> 真实链路丢失")
    _kill_pub(pub)
    # 确认控制链路真的断了: publisher 与其 mavsdk_server 都应消失
    await asyncio.sleep(1)
    alive_server = ""
    try:
        alive_server = subprocess.check_output(
            ["pgrep", "-af", "mavsdk_server.*50052"]).decode().strip()
    except Exception:
        alive_server = ""
    print(f"[dbg ] pub.returncode={pub.returncode} 残留mavsdk_server(50052)={alive_server or '无'}")
    await asyncio.sleep(14)  # offboard 超时(2s) + blind land 下降
    inj_idx = find_injection_idx(rec.rows, inj_idx_holder["t"])
    passed, detail = assert_link_loss(rec.rows, inj_idx, alt)
    print("[rec ] 停止记录 ...")
    await safe(rec.stop(), "rec.stop")
    try:
        await safe(telem.offboard.stop(), "offboard.stop")
        if await safe(_in_air(telem), "_in_air", 8):
            await safe(telem.action.land(), "land")
        await safe(telem.action.disarm(), "disarm")
    except Exception as e:
        print(f"[warn] 收尾异常(忽略): {e}")
    verdict = "PASS" if passed else "FAIL"
    print(f"[{verdict}] case 4: {detail}")
    return (0 if passed else 1), detail, csv_path


# ---------------------------------------------------------------------------
async def fly_and_inject(drone, rec, case, addr, alt, side, tol, seg_timeout, hold, out_dir):
    if case == 4:
        # case 4 用独立控制子进程 + GCS 遥测链路, 不走下面的统一 arm/offboard 流程
        telem = System()
        return await run_case4(telem, case, alt, side, tol, seg_timeout, hold, out_dir)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"fault_case{case}_{stamp}.csv")
    rec.csv_path = csv_path

    await wait_connected(drone, addr)
    await set_velocity_limits(drone)

    rec.start()
    if not await rec.wait_first_fix():
        print("[warn] 15s 内未收到位置遥测")

    # 启用故障注入 (cases 1-3 需要; case 4 用 offboard 断开模拟链路丢失; case 5 用 GF 参数)
    if case in (1, 2, 3):
        await set_param(drone, "SYS_FAILURE_EN", 1, as_int=True)

    print("[arm ] 解锁 ...")
    if not await arm_with_retry(drone, rec):
        await rec.stop()
        return 1, "解锁失败", csv_path
    rec.phase = "takeoff"
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -alt, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[err ] offboard 启动失败: {e}")
        await drone.action.disarm()
        await rec.stop()
        return 1, "offboard 启动失败", csv_path

    print(f"[fly ] 起飞到 {alt} m ...")
    await goto(drone, rec, 0.0, 0.0, alt, "takeoff", tol, seg_timeout, hold)

    # 地理围栏用例: 起飞后立刻设小半径围栏 (home 已在 arm 后建立)
    if case == 5:
        await set_param(drone, "GF_MAX_HOR_DIST", 12.0, as_int=False)
        await set_param(drone, "GF_MAX_VER_DIST", 10.0, as_int=False)
        await set_param(drone, "GF_ACTION", 3, as_int=True)  # 3 = Return

    # 飞第 1 条边 (北)
    await goto(drone, rec, side, 0.0, alt, "edge1_north", tol, seg_timeout, hold)

    inj_idx_holder = {}

    def mark_injection():
        inj_idx_holder["t"] = rec.rows[-1]["t"] if rec.rows else 0.0

    passed = False
    detail = ""
    try:
        if case == 1:  # GPS 丢失
            mark_injection()
            print("[inj ] SENSOR_GPS OFF")
            await drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OFF, 0)
            await goto(drone, rec, side, side, alt, "edge2_east", tol, seg_timeout, hold)
            await asyncio.sleep(8)
            inj_idx = find_injection_idx(rec.rows, inj_idx_holder.get("t", 0))
            passed, detail = assert_gps(rec.rows, inj_idx, alt)

        elif case == 2:  # 气压计故障
            mark_injection()
            print("[inj ] SENSOR_BARO OFF")
            await drone.failure.inject(FailureUnit.SENSOR_BARO, FailureType.OFF, 0)
            for (n, e, lb) in [(side, side, "edge2_east"), (0, side, "edge3_south"),
                               (0, 0, "edge4_west")]:
                await goto(drone, rec, n, e, alt, lb, tol, seg_timeout, hold)
            await asyncio.sleep(3)
            inj_idx = find_injection_idx(rec.rows, inj_idx_holder.get("t", 0))
            passed, detail = assert_keep_flying(rec.rows, inj_idx, "baro")

        elif case == 3:  # 磁力计故障
            mark_injection()
            print("[inj ] SENSOR_MAG OFF")
            await drone.failure.inject(FailureUnit.SENSOR_MAG, FailureType.OFF, 0)
            for (n, e, lb) in [(side, side, "edge2_east"), (0, side, "edge3_south"),
                               (0, 0, "edge4_west")]:
                await goto(drone, rec, n, e, alt, lb, tol, seg_timeout, hold)
            await asyncio.sleep(3)
            inj_idx = find_injection_idx(rec.rows, inj_idx_holder.get("t", 0))
            passed, detail = assert_keep_flying(rec.rows, inj_idx, "mag")

        elif case == 5:  # 地理围栏越界
            mark_injection()
            print("[inj ] 飞出围栏 (N=25) -> 期望 RETURN")
            await goto(drone, rec, 25.0, 0.0, alt, "breach_out", tol, seg_timeout, hold)
            await asyncio.sleep(10)
            inj_idx = find_injection_idx(rec.rows, inj_idx_holder.get("t", 0))
            passed, detail = assert_geofence(rec.rows, inj_idx)

    except Exception as e:
        detail = f"飞行/断言异常: {e}"
        passed = False

    # ---- 恢复 + 收尾 (全部加超时, 避免已降落态下 MAVSDK 调用阻塞) ----
    print("[rec ] 停止记录 ...")
    await safe(rec.stop(), "rec.stop")
    try:
        if case in (1, 2, 3):
            print("[rec ] 恢复故障 OK")
            unit = (FailureUnit.SENSOR_GPS if case == 1 else
                    FailureUnit.SENSOR_BARO if case == 2 else
                    FailureUnit.SENSOR_MAG)
            await safe(drone.failure.inject(unit, FailureType.OK, 0), "recover")
        if case == 5:
            await safe(set_param(drone, "GF_MAX_HOR_DIST", 0.0, as_int=False), "gf_reset")
        await safe(drone.offboard.stop(), "offboard.stop")
        if await safe(_in_air(drone), "_in_air", 8):
            await safe(drone.action.land(), "land")
            try:
                async for ia in drone.telemetry.in_air():
                    if not ia:
                        break
            except Exception:
                pass
        await safe(drone.action.disarm(), "disarm")
    except Exception as e:
        print(f"[warn] 收尾异常(忽略): {e}")

    verdict = "PASS" if passed else "FAIL"
    print(f"[{verdict}] case {case}: {detail}")
    return (0 if passed else 1), detail, csv_path


async def _in_air(drone):
    async for ia in drone.telemetry.in_air():
        return ia
    return False


async def arm_with_retry(drone, rec, attempts=5):
    """等健康 + 重试解锁。SIH 偶发 arm 被拒(COMMAND_DENIED), 多为上帧状态残留,
    延时重试通常可解决; 同时等待 health_all_ok(若支持)确保可解锁。"""
    # 尝试等待 EKF/位置健康 (API 不同版本名不一, 探测式)
    for _ in range(40):
        ok = False
        for fnname in ("health_all_ok", "is_armable"):
            fn = getattr(drone.telemetry, fnname, None)
            if fn is None:
                continue
            try:
                ok = await asyncio.wait_for(fn(), timeout=3)
                break
            except Exception:
                ok = False
        if ok:
            break
        await asyncio.sleep(0.5)
    last = None
    for i in range(attempts):
        try:
            await drone.action.arm()
            print("[arm ] 解锁成功")
            return True
        except Exception as e:
            last = e
            print(f"[arm ] 解锁被拒(重试 {i+1}/{attempts}): {e}")
            await asyncio.sleep(2)
    print(f"[err ] 解锁最终失败: {last}")
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", type=int, required=False, choices=[1, 2, 3, 4, 5])
    ap.add_argument("--addr", default=DEFAULT_ADDR)
    ap.add_argument("--out-dir", default=LOG_DIR)
    ap.add_argument("--alt", type=float, default=5.0)
    ap.add_argument("--side", type=float, default=10.0)
    ap.add_argument("--tol", type=float, default=0.5)
    ap.add_argument("--seg-timeout", type=float, default=25.0)
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--publisher", action="store_true",
                    help="case 4 控制子进程模式: 仅做 offboard 飞行, 被杀即链路丢失")
    args = ap.parse_args()

    if args.publisher:
        try:
            asyncio.run(asyncio.wait_for(
                run_publisher(args.addr, args.out_dir, args.alt, args.side,
                              args.tol, args.seg_timeout, args.hold),
                timeout=600))
        except Exception as e:
            print(f"[pub ] 异常退出: {e}")
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)

    if args.case is None:
        print("--case 必填 (或加 --publisher 进入控制子进程模式)")
        sys.stdout.flush()
        os._exit(2)

    drone = System()
    rec = Recorder(drone, "")
    rc, detail, csv_path = 0, "", ""
    try:
        rc, detail, csv_path = asyncio.run(
            asyncio.wait_for(
                fly_and_inject(drone, rec, args.case, args.addr, args.alt, args.side,
                               args.tol, args.seg_timeout, args.hold, args.out_dir),
                timeout=280))
    except asyncio.TimeoutError:
        print("[fatal] 整体超时(280s), 强制退出")
        rc = 1
    except Exception as e:
        print(f"[fatal] {e}")
        rc = 1
    print(f"轨迹CSV: {csv_path}")
    # MAVSDK 的 System() 会启动非守护后台线程(UDP reader), 即使 asyncio.run
    # 返回、sys.exit 抛出异常, 解释器也要等该线程结束才退出 -> 进程挂死被
    # timeout 杀掉(rc=124)。这里直接 os._exit 强杀进程, 让启动脚本拿到正确退出码。
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)


if __name__ == "__main__":
    main()
