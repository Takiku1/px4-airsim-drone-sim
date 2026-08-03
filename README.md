# PX4 + AirSim 无人机仿真链路（方形航线自动飞行 + 日志分析）

[![CI](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/ci.yml)

> 无需实机，即可在 PC 上搭建一条完整的无人机**仿真 / 系统测试**链路：
> **PX4 SITL（WSL2） ↔ AirSim（UE5.3 / Windows） ↔ MAVSDK-Python ↔ QGroundControl**，
> 自动执行方形航线飞行，并基于 PX4 机载 `ulg` 日志做轨迹评分。

这是本人求职 **无人机仿真 / 系统测试** 方向的实习项目主线，重点体现：
**在没有物理飞控与实机的情况下，独立打通"仿真—飞行—采集—分析—评分"闭环的能力**。

---

## 仿真链路架构

```
┌─────────────────────┐         TCP 4560          ┌──────────────────────────┐
│  AirSim (UE5.3)     │ <-----------------------> │  PX4 SITL v1.15.2        │
│  Windows 11 (RTX4060)│   传感器/姿态/位姿         │  WSL2 Ubuntu 22.04        │
└─────────────────────┘                           └────────────┬─────────────┘
                                                                │ UDP (MAVLink)
                                                                │ offboard
┌─────────────────────┐                           ┌────────────▼─────────────┐
│  QGroundControl     │ <--- UDP 14550 ----------> │  MAVSDK-Python 任务脚本   │
│  (地面站监控)        │   遥测/虚拟摇杆            │  square_mission.py        │
└─────────────────────┘                           └────────────┬─────────────┘
                                                                │ 写 CSV 轨迹
                                                                ▼
                                                analyze_ulg.py → 方形度评分 + 轨迹 SVG
```

- **AirSim**：Windows 侧 UE5.3 渲染无人机与场景，通过 TCP 4560 向 PX4 提供仿真状态。
- **PX4 SITL**：在 WSL2 中运行飞控固件（`none_iris` 机型），输出 MAVLink，并落盘 `ulg` 飞行日志。
- **MAVSDK-Python**：以 offboard 模式向 PX4 下发位置设定点，执行方形航线，实时记录 N-E-D 轨迹 CSV。
- **QGC**：UDP 14550 接入，用于监控与手动接管。

---

## 环境要求

| 组件 | 版本 / 说明 |
| --- | --- |
| Windows | 11，独立显卡（本项目 RTX 4060 Laptop 8GB） |
| WSL2 | Ubuntu 22.04，镜像网络模式（与 Windows 共享 localhost） |
| PX4-Autopilot | v1.15.2，`make px4_sitl none_iris` |
| AirSim | Colosseum-AI fork，UE5.3，`settings.json` 设 `TcpPort=4560` |
| MAVSDK-Python | `pip install mavsdk`（任务脚本依赖） |
| ulog2csv | PX4 自带，将 `ulg` 转为 CSV 供分析 |
| QGC | 任意版本，UDP 14550 连接 |

---

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `run_flight_mission.sh` | **一键飞行脚本**（在 WSL2 运行）：拉起 PX4 → 等 `Simulator connected` → 实测 offboard UDP 端口 → 飞方形 → 采集 `ulg` + CSV 落盘到 Windows 侧。 |
| `square_mission.py` | MAVSDK-Python offboard 方形航线任务。参数：`--addr`（MAVLink 地址）、`--out-dir`（CSV 输出目录）、`--alt` 高度（默认 5m）、`--side` 边长（默认 10m）、`--tol` 到点容差、`--seg-timeout` 单边超时、`--hold` 到点保持。 |
| `analyze_ulg.py` | 纯标准库轨迹分析：读 `ulog2csv` 产出的 `vehicle_local_position` CSV，计算**方形度评分（0–100）**、闭合误差、高度稳定性、最大速度，并输出俯视轨迹 SVG。 |
| `extract_ulg.sh` | 找最新 `ulg` → 拷到 Windows → `ulog2csv` → 列出 CSV，为分析做准备。 |
| `diag_ulg.sh` | PX4 日志快速诊断（端口、连接、关键参数核对）。 |
| `requirements.txt` | Python 依赖（`mavsdk`）。 |

> 注：脚本中 `D:\AirSim\mission`、`/mnt/d/AirSim/mission`、`$HOME/PX4-Autopilot` 为作者本机路径，
> 使用时请按自己的环境修改。

---

## 快速上手

### 1. 启动 AirSim（Windows 侧）
在 UE5.3 中以 `-game` 模式打开 Blocks 地图，确认 `settings.json` 中 `TcpPort=4560`，
AirSim 在 TCP 4560 监听。

### 2. 一键飞行（WSL2 侧）
```bash
bash /mnt/d/AirSim/mission/run_flight_mission.sh
```
脚本会自动完成：启动 PX4 SITL → 等待与 AirSim 连接 → 探测 offboard 端口 →
执行 10m 方形航线 → 停止 PX4 并拷贝 `ulg` + 轨迹 CSV 到 Windows 侧。

如需自定义：
```bash
MAVSDK_ADDR="udpout://127.0.0.1:18570" \
  python3 square_mission.py --addr "udpout://127.0.0.1:18570" \
  --out-dir /mnt/d/AirSim/mission --alt 5 --side 10
```

### 3. 分析日志（Windows 侧）
```bash
# 先由 extract_ulg.sh 把 ulg 转成 CSV
bash extract_ulg.sh

# 再分析（参数为 ulog2csv 产出的 vehicle_local_position CSV）
python analyze_ulg.py D:\AirSim\mission\ulg_csv\07_50_35_vehicle_local_position_0.csv
```
输出：方形度评分 + `trajectory_ulg_*.svg` 俯视轨迹图。

---

## 航线指标与评分

`analyze_ulg.py` 的方形度评分综合四项惩罚：
- **腿长误差**（四边是否等长，权重 0.40）
- **闭合误差**（回到起点偏差，权重 0.30）
- **长宽比**（是否接近正方形，权重 0.15）
- **高度稳定性**（巡航段高度波动，权重 0.15）

### 实测结果（限速参数调优后）

| 数据源 | 方形度 | 备注 |
| --- | --- | --- |
| MAVSDK 遥测 CSV | 88.8 / 100 | 四边 ≈10.4m，闭合误差 0.39m，巡航高度 4.98±0.05m，最大速度 2.37 m/s |
| PX4 机载 `ulg` 真值 | **95.3 / 100** | 机载日志交叉验证，轨迹更平滑 |

限速参数（`param set` 后 `param commit` 并重启 PX4）：
```
MPC_XY_VEL_MAX = 2.0
MPC_XY_CRUISE  = 1.5
```
无限速时方形度仅 33（overshoot 3.5m、速度 7.1 m/s），加限速后轨道质量显著改善——这体现了**仿真环境下的参数调优闭环**。

---

## 项目价值（面向系统测试 / 仿真岗）

- ✅ 从零打通跨平台（Windows + WSL2）仿真链路，理解 SITL / HITL 边界。
- ✅ 用 offboard 模式编程控制无人机，处理 MAVLink 端口协商、连接时序、日志落盘时机等真实坑点。
- ✅ 建立"飞行—日志—评分"自动化测试闭环，可复用于回归测试与参数对比。
- ✅ 所有步骤无需实机，适合作为可复现、可演示的投递作品。

---

## License

MIT © 余佳畅 (Takiku1)
