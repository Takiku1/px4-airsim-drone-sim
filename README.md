# PX4 + AirSim 无人机仿真链路（方形航线自动飞行 + 日志分析）

[![CI](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/ci.yml)
[![Phase 2](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/sim-flight.yml/badge.svg)](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/sim-flight.yml)
[![Fault Regression (self-hosted)](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/fault-regression-selfhosted.yml/badge.svg)](https://github.com/Takiku1/px4-airsim-drone-sim/actions/workflows/fault-regression-selfhosted.yml)

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
| `scripts/fault_test.py` | **故障注入与容错测试主脚本**：在 PX4 SIH + MAVSDK offboard 下注入 5 类故障（GPS 丢失 / 气压计 / 磁力计 / 链路丢失 / 地理围栏），断言 PX4 failsafe 行为并写出独立遥测 CSV。链路丢失用例通过独立子进程 + 强杀 `mavsdk_server` 子树真实模拟断链。 |
| `scripts/run_fault_test.sh` | 故障套件启动器：拉起 SIH → 逐用例重启保证隔离 → 各输出 PASS/FAIL，全绿才退出 0。 |
| `scripts/fault_asserts.py` | 纯函数断言（无 `mavsdk` 依赖），被 `fault_test.py` 与 pytest 回归共同复用。 |
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

## 自动化仿真测试 (CI)

本项目把"飞行—采集—评分"闭环接入 GitHub Actions，作为可复现的回归测试：

| 工作流 | 文件 | 内容 | 触发 |
| --- | --- | --- | --- |
| 代码质量 CI（Phase 1） | `.github/workflows/ci.yml` | `py_compile` + `bash -n` 语法检查 + `pytest` 跑 `analyze_ulg.py` 回归（真实样本 95.3 / 完美方形 ≥95 / 随机轨迹 <50） | push / PR |
| 轨迹回归门禁 CI（Phase 2） | `.github/workflows/sim-flight.yml` | 用本地已验证的真实飞行遥测 `tests/fixtures/trajectory_golden.csv`（方形度 95.3）作 golden fixture，在 CI 反复校验 `check_square.py` 方形度评分能识别干净方形（>80）；并跑 `pytest` 分析回归 | push / PR / 手动 |

> **Phase 2 为什么是"回归门禁"而不是"云端真飞"**：GitHub 托管 runner 上 `docker run` 拉起 PX4 SIH
> 会挂死（已三次验证，与网络模式无关），且 PX4 官方 `.deb` 未发布在 GitHub release —— 云端目前无法
> 可靠跑真实 SITL。因此把本地已验证的真实飞行遥测（方形度 95.3，MAVSDK 遥测 + PX4 机载 `ulg` 双源
> 交叉验证）固化进仓库作为 golden 样本，在 CI 中持续校验评分逻辑。真实 SITL 飞行本身是项目核心能力，
> 已在本地（WSL PX4 SIH + MAVSDK offboard）实测达成，可作演示 / 本地回归；未来若要在 CI 跑真实仿真，
> 可在本机部署 **self-hosted runner**（WSL 原生跑 PX4，规避 GitHub runner 的 docker 限制）。
> `check_square.py` 读取 `square_mission.py` 输出的 MAVSDK 轨迹 CSV（`north_m/east_m/down_m/phase`），
> 按相位提取四角点，计算边长 / 闭合 / 夹角 / 高度稳定性并评分，复用 `analyze_ulg.py` 的加权方法论。

---

## 故障注入与容错回归测试

在 SIH（headless，无需 AirSim / GPU）下注入 5 类典型故障，断言 PX4 的失效保护（failsafe）行为是否符合预期。每个用例独立重启 SIH，保证状态干净、CI 可复现。

| 用例 | 故障 | 期望 PX4 行为 | 关键断言 |
| --- | --- | --- | --- |
| 1 | GPS 丢失 | 盲降（blind land） | 离开 OFFBOARD + 高度下降 >1m |
| 2 | 气压计故障 | EKF 回落 GPS 高度，继续飞 | 保持 OFFBOARD + 继续飞行 + 高度有效 |
| 3 | 磁力计故障 | EKF 降级，继续飞 | 保持 OFFBOARD + 继续飞行 + 高度有效 |
| 4 | 链路丢失 | 数据链路失效保护 | 离开 OFFBOARD + 原地（水平 <2.5m）+ 不爬升 + 下降 >1m |
| 5 | 地理围栏越界 | RETURN | 进入 RETURN / 离开 OFFBOARD + 朝 HOME 返航 |

**链路丢失的真实模拟（关键坑点）**：PX4 的 offboard 丢失判定依据是 *setpoint 新鲜度*（`offboard_control_mode` 消息是否持续到达），而非链路整体心跳。MAVSDK 的 `System()` 会以子进程方式拉起独立的 `mavsdk_server` 并持有真正的 MAVLink 套接字与 setpoint 循环——仅杀掉 Python 控制进程会把它**孤儿化**，使其继续发 setpoint，PX4 永不判链路丢失。因此 case 4 让控制链路跑在独立子进程，注入 = `SIGKILL` 该子进程及其整个进程树（含 `mavsdk_server` 子进程），才能触发真正的 blind land。

本地运行（5 个用例全跑，约 6 分钟）：
```bash
bash scripts/run_fault_test.sh /mnt/d/AirSim/mission/px4-airsim-drone-sim
# 单跑某用例: 末尾加用例号, 如 ... 4
```

### pytest 回归（golden fixture）

真实 SITL 跑出的遥测 CSV 固化为 `tests/fixtures/fault/fault_case{1..5}.csv`，由 `tests/test_fault_regression.py` 用 `fault_asserts` 纯函数反复校验。CI 中**无需拉起真实 SITL** 即可守住"故障注入断言逻辑没被改坏"的回归门槛：
```bash
python3 -m pytest tests/test_fault_regression.py -v
```

### self-hosted CI

`.github/workflows/fault-regression-selfhosted.yml` 与正常飞行门禁 `sim-flight-selfhosted.yml` **并列**，均只手动触发（`workflow_dispatch`，避免公开仓库 fork PR 在本机执行代码）：在本人 self-hosted runner 上 `workflow_dispatch` → WSL2 PX4 SIH → 跑 5 故障用例 → 全绿才通过。遥测 CSV 作为 artifact 上传。

---

## 项目价值（面向系统测试 / 仿真岗）

- ✅ 从零打通跨平台（Windows + WSL2）仿真链路，理解 SITL / HITL 边界。
- ✅ 用 offboard 模式编程控制无人机，处理 MAVLink 端口协商、连接时序、日志落盘时机等真实坑点。
- ✅ 建立"飞行—日志—评分"自动化测试闭环，可复用于回归测试与参数对比。
- ✅ 所有步骤无需实机，适合作为可复现、可演示的投递作品。

---

## License

MIT © 余佳畅 (Takiku1)
