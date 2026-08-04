#!/usr/bin/env python3
"""
AirSim + PX4 SITL 方形航线任务 (MAVSDK offboard)

航线: 起飞 alt -> 北 side -> 东 side -> 南 side -> 西 side(回原点) -> 降落

与早期版本的区别:
  * 每条边采用「到点判定」而非固定 sleep，避免未到位就转弯导致轨迹被削成圆角/菱形
  * 到达后额外 hold 一小段时间，让四个角点在轨迹上清晰可辨
  * 全程记录 NED 位置/速度到 CSV，供 analyze_trajectory.py 使用

用法:
    python3 square_mission.py [--alt 5] [--side 10] [--tol 0.5] [--seg-timeout 25]
"""

import argparse
import asyncio
import csv
import math
import os
import sys
import time
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# offboard 连接地址: 优先用环境变量(由 run_flight_mission.sh 实测 PX4 端口后注入), 默认 14540
DEFAULT_ADDR = os.environ.get("MAVSDK_ADDR", "udp://:14540")

# 轨迹 CSV 落点: 优先写到 Windows 侧 D:\AirSim\mission\ (WSL 里是 /mnt/d/...),
# 这样 Windows 侧的助手能直接读到并分析; 若该挂载不可用则退回 WSL 家目录。
if os.path.isdir("/mnt/d/AirSim/mission"):
    LOG_DIR = "/mnt/d/AirSim/mission"
else:
    LOG_DIR = os.path.expanduser("~/airsim_missions")


class TelemetryRecorder:
    """后台订阅位置遥测, 写入 CSV, 并对外提供最新位置用于到点判定。"""

    def __init__(self, drone, csv_path):
        self.drone = drone
        self.csv_path = csv_path
        self.rows = []
        self._task = None
        self._stop = False
        self.phase = "init"
        self.latest = None  # (north, east, down)

    async def _run(self):
        t0 = time.time()
        async for pv in self.drone.telemetry.position_velocity_ned():
            if self._stop:
                break
            p, v = pv.position, pv.velocity
            self.latest = (p.north_m, p.east_m, p.down_m)
            self.rows.append(
                {
                    "t": round(time.time() - t0, 3),
                    "phase": self.phase,
                    "north_m": round(p.north_m, 3),
                    "east_m": round(p.east_m, 3),
                    "down_m": round(p.down_m, 3),
                    "vn": round(v.north_m_s, 3),
                    "ve": round(v.east_m_s, 3),
                    "vd": round(v.down_m_s, 3),
                }
            )

    def start(self):
        self._task = asyncio.ensure_future(self._run())

    async def wait_first_fix(self, timeout=15):
        deadline = time.time() + timeout
        while self.latest is None and time.time() < deadline:
            await asyncio.sleep(0.2)
        return self.latest is not None

    async def stop(self):
        self._stop = True
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except BaseException:
                pass
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, "w", newline="") as f:
            w = csv.DictWriter(
                f,
                fieldnames=["t", "phase", "north_m", "east_m", "down_m", "vn", "ve", "vd"],
            )
            w.writeheader()
            w.writerows(self.rows)
        print(f"[rec ] {len(self.rows)} 条遥测已写入 {self.csv_path}")


async def wait_connected(drone, addr, timeout=60):
    print(f"[conn] 连接 {addr} (超时 {timeout}s) ...")
    await drone.connect(system_address=addr)
    deadline = time.time() + timeout
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[conn] 已连接到飞控")
            return
        if time.time() > deadline:
            raise RuntimeError(
                f"连接 {addr} 超时 {timeout}s: 未收到 is_connected 心跳 "
                f"(请确认 PX4 SIH 已启动且在 14540 发 MAVLink)"
            )
    raise RuntimeError("连接失败")


async def wait_ready(drone, timeout=90):
    print("[chk ] 等待位置估计收敛 (软检查, 超时仅警告) ...")
    deadline = time.time() + timeout
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[chk ] 位置估计 OK")
            return
        if time.time() > deadline:
            # offboard 位置控制基于本地 NED, 不严格依赖全局定位, 故仅警告
            print("[warn] 位置估计未在 %ds 内完全就绪, 仍尝试继续 (gyro=%s accel=%s mag=%s)"
                  % (timeout, health.is_gyrometer_calibration_ok,
                     health.is_accelerometer_calibration_ok, health.is_magnetometer_calibration_ok))
            return


async def set_velocity_limits(drone, xy_max=2.0, xy_cruise=1.5, z_max=1.5):
    """限制 PX4 水平/垂直速度, 让方形航线棱角分明、不过冲。

    之前未限速时 offboard 速度冲到 ~7 m/s, 拐角处惯性过冲 ~3.5m,
    飞成 ~13m 边而非 10m。限速到 2 m/s 后轨迹为干净 10x10 方框。
    """
    params = {
        "MPC_XY_VEL_MAX": float(xy_max),
        "MPC_XY_CRUISE": float(xy_cruise),
        "MPC_Z_VEL_MAX_DN": float(z_max),
        "MPC_Z_VEL_MAX_UP": float(z_max),
    }
    print(f"[cfg ] 设置速度限制 (xy_max={xy_max} cruise={xy_cruise} z={z_max} m/s) ...")
    ok = 0
    for name, val in params.items():
        try:
            # MAVSDK 各版本 param API 名称不一: 新版本 set_param_float, 旧版本 set_float_param
            try:
                await drone.param.set_param_float(name, val)
            except AttributeError:
                await drone.param.set_float_param(name, val)
            ok += 1
        except Exception as e:
            print(f"[warn] 参数 {name} 设置失败 (可忽略): {e}")
    print(f"[cfg ] 已应用 {ok}/{len(params)} 项速度限制")


async def goto(drone, rec, north, east, alt, label, tol, timeout, hold):
    """飞往目标点并等待真正到达 (水平误差 < tol 且高度误差 < tol*2)。"""
    rec.phase = label
    target_down = -alt
    print(f"[fly ] {label:5s} -> N={north:6.1f}  E={east:6.1f}  alt={alt:.1f}", end="", flush=True)

    await drone.offboard.set_position_ned(PositionNedYaw(north, east, target_down, 0.0))

    t_start = time.time()
    reached_at = None
    while True:
        await asyncio.sleep(0.2)
        elapsed = time.time() - t_start

        if rec.latest is not None:
            n, e, d = rec.latest
            herr = math.hypot(n - north, e - east)
            verr = abs(d - target_down)
            if herr < tol and verr < tol * 2:
                if reached_at is None:
                    reached_at = time.time()
                # 稳定 hold 秒后认为真正到位
                if time.time() - reached_at >= hold:
                    print(f"   到位 用时 {elapsed:5.1f}s  水平误差 {herr:.2f} m")
                    return True
            else:
                reached_at = None

        if elapsed > timeout:
            if rec.latest is not None:
                n, e, d = rec.latest
                herr = math.hypot(n - north, e - east)
                print(f"   !! 超时 {timeout:.0f}s  残余误差 {herr:.2f} m (继续下一段)")
            else:
                print(f"   !! 超时 {timeout:.0f}s  无位置数据")
            return False


async def run(addr, alt, side, tol, seg_timeout, hold, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"trajectory_{stamp}.csv")

    drone = System()
    await wait_connected(drone, addr)
    await wait_ready(drone)

    # 限速: 避免 offboard 速度冲太高导致拐角 overshoot (之前飞成 ~13m 边)
    await set_velocity_limits(drone)

    rec = TelemetryRecorder(drone, csv_path)
    rec.start()
    if not await rec.wait_first_fix():
        print("[warn] 15s 内未收到位置遥测, 到点判定将退化为超时控制")

    print("[arm ] 解锁 ...")
    await drone.action.arm()

    # offboard 启动前必须先推送 setpoint
    rec.phase = "takeoff"
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -alt, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[err ] offboard 启动失败: {e._result.result}")
        await drone.action.disarm()
        await rec.stop()
        return

    print(f"[fly ] 起飞到 {alt} m ...")
    await goto(drone, rec, 0.0, 0.0, alt, "takeoff", tol, seg_timeout, hold)

    corners = [
        (side, 0.0, "north"),
        (side, side, "east"),
        (0.0, side, "south"),
        (0.0, 0.0, "west"),
    ]
    ok = 0
    for n, e, label in corners:
        if await goto(drone, rec, n, e, alt, label, tol, seg_timeout, hold):
            ok += 1

    print(f"[fly ] 四边完成, 到位 {ok}/4")

    print("[land] 停止 offboard 并降落 ...")
    rec.phase = "land"
    try:
        await drone.offboard.stop()
    except OffboardError as e:
        print(f"[warn] offboard 停止异常: {e._result.result}")

    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    print("[land] 已落地")
    await asyncio.sleep(2)

    await rec.stop()
    print(f"[done] 任务完成, 轨迹: {csv_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=DEFAULT_ADDR)
    ap.add_argument("--out-dir", default=LOG_DIR, help="轨迹 CSV 输出目录")
    ap.add_argument("--alt", type=float, default=5.0, help="飞行高度 m")
    ap.add_argument("--side", type=float, default=10.0, help="方形边长 m")
    ap.add_argument("--tol", type=float, default=0.5, help="到点容差 m")
    ap.add_argument("--seg-timeout", type=float, default=25.0, help="单边最长等待 s")
    ap.add_argument("--hold", type=float, default=1.0, help="到点后稳定保持 s")
    args = ap.parse_args()
    asyncio.run(run(args.addr, args.alt, args.side, args.tol, args.seg_timeout, args.hold, args.out_dir))


if __name__ == "__main__":
    main()
