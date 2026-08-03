#!/bin/bash
# ===================================================================
# AirSim + PX4 SITL 方形航线联调一键脚本 (在 WSL2 Ubuntu 中运行)
#
# 前提: Windows 侧 UE5.3 已用 -game 模式打开 Blocks (AirSim 在 TCP 4560 监听)
#
# 用法(在 WSL 中一条命令即可):
#       bash /mnt/d/AirSim/mission/run_flight_mission.sh
#
# 本脚本负责: 拉起 PX4(none_iris) -> 验证 "Simulator connected" ->
#            实测 PX4 offboard UDP 端口 -> 飞方形 -> 采集 ulg + CSV 到
#            /mnt/d/AirSim/mission/ (即 Windows 的 D:\AirSim\mission\)。
#            分析由 Windows 侧助手完成, 本脚本只负责飞与采集。
# ===================================================================

set -u

# ---------- 解释器/工具探测 ----------
PYTHON=""
if command -v python3 >/dev/null 2>&1; then
    PYTHON=python3
elif command -v python >/dev/null 2>&1; then
    PYTHON=python
else
    echo "!!! 找不到 python3 或 python, 无法运行任务"
    exit 1
fi
command -v make >/dev/null 2>&1 || { echo "!!! 找不到 make (PX4 需要)"; exit 1; }

SRC_DIR="/mnt/d/AirSim/mission"
PX4_DIR="$HOME/PX4-Autopilot"
WIN_DIR="/mnt/d/AirSim/mission"          # 落盘到 Windows 侧, 助手可读
PX4_LOG="/tmp/px4_sitl.log"
STATUS="$WIN_DIR/mission_status.txt"

MISSION="$HOME/square_mission.py"

mkdir -p "$WIN_DIR"

echo "==================================================="
echo " AirSim + PX4 方形航线任务"
echo "==================================================="

# ---------- [0] 自同步脚本 (从 Windows 侧拷到 \$HOME 并修正行尾) ----------
echo "[0/6] 同步任务脚本 ..."
if [ -d "$SRC_DIR" ]; then
    for f in square_mission.py analyze_trajectory.py analyze_ulg.py; do
        if [ -f "$SRC_DIR/$f" ]; then
            cp "$SRC_DIR/$f" "$HOME/$f"
            sed -i 's/\r$//' "$HOME/$f"
            chmod +x "$HOME/$f"
            echo "      + $f"
        fi
    done
else
    echo "      (找不到 $SRC_DIR, 沿用 \$HOME 中已有脚本)"
fi

# ---------- 依赖自检 ----------
echo "      检查 Python 依赖 ..."
$PYTHON - <<'PYEOF'
import importlib, sys
missing = [m for m in ("mavsdk",) if importlib.util.find_spec(m) is None]
if missing:
    print("      !!! 缺少必需模块: " + ", ".join(missing))
    print("      请执行: pip3 install " + " ".join(missing))
    sys.exit(1)
print("      依赖 OK (mavsdk)")
PYEOF
if [ $? -ne 0 ]; then exit 1; fi

# ---------- 检查 AirSim 是否在监听 4560 ----------
echo "      检查 AirSim TCP 4560 ..."
SEEN_4560=0
if command -v ss >/dev/null 2>&1; then
    ss -tln 2>/dev/null | grep -q ":4560" && SEEN_4560=1
elif command -v netstat >/dev/null 2>&1; then
    netstat -tln 2>/dev/null | grep -q ":4560" && SEEN_4560=1
fi
if [ "$SEEN_4560" -eq 1 ]; then
    echo "      4560 已监听 (AirSim 运行中)"
else
    echo "      !!! 未检测到 4560 监听。请确认 Windows 侧 AirSim (PID 31764) 仍在运行。"
    echo "          (若 WSL 非镜像网络且 4560 不可见, 可忽略此警告, 但链接大概率会失败)"
fi

# ---------- [1] 启动 PX4 (none_iris, 常驻) ----------
pkill -f "px4_sitl" 2>/dev/null
pkill -f "bin/px4" 2>/dev/null
sleep 1

echo "[1/6] 启动 PX4 SITL (none_iris) ..."
cd "$PX4_DIR" || { echo "找不到 $PX4_DIR"; exit 1; }
PX4_SIM_HOST_ADDR=127.0.0.1 nohup make px4_sitl none_iris > "$PX4_LOG" 2>&1 &
PX4_PID=$!
echo "      PID=$PX4_PID   日志: $PX4_LOG"

# ---------- [2] 等待连接 ----------
echo "[2/6] 等待与 AirSim 建立连接 (最多 150s) ..."
CONNECTED=0
for i in $(seq 1 150); do
    if grep -qi "Simulator connected" "$PX4_LOG" 2>/dev/null; then
        CONNECTED=1
        echo "      >>> Simulator connected  (用时 ${i}s)"
        break
    fi
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo "      !!! PX4 进程已退出, 日志尾部:"
        tail -30 "$PX4_LOG"
        exit 1
    fi
    sleep 1
done

if [ "$CONNECTED" -eq 0 ]; then
    echo "      !!! 150s 内未收到 'Simulator connected'"
    echo "      排查: AirSim 是否在运行? settings.json TcpPort=4560? WSL 是否镜像网络?"
    tail -30 "$PX4_LOG"
    exit 1
fi

echo "      等待 EKF 收敛 (15s) ..."
sleep 15

# ---------- [3] 实测 PX4 offboard UDP 端口 ----------
echo "[3/6] 检测 PX4 MAVLink offboard 端口 ..."
MAV_PORT=""
PORTS=""
if [ -n "${MAVSDK_PORT:-}" ]; then
    MAV_PORT="$MAVSDK_PORT"
    echo "      使用指定的 MAVSDK_PORT=$MAV_PORT"
fi
if [ -z "$MAV_PORT" ]; then
    for i in $(seq 1 20); do
        if command -v ss >/dev/null 2>&1; then
            PORTS=$(ss -ulnp 2>/dev/null | grep -i px4 | grep -oP ':\K[0-9]+' | tr -d '\r' | sort -n | uniq)
        elif command -v netstat >/dev/null 2>&1; then
            PORTS=$(netstat -ulnp 2>/dev/null | grep -i px4 | grep -oP ':\K[0-9]+' | tr -d '\r' | sort -n | uniq)
        else
            PORTS=""
        fi
        # 候选优先级 (依据 PX4 实际监听端口): 18570(外部API/offboard) > 14580(仿真器MAVLink) > 14280 > 13030
        for cand in 18570 14580 14280 13030; do
            if echo "$PORTS" | grep -qx "$cand"; then MAV_PORT=$cand; break 2; fi
        done
        [ -n "$PORTS" ] && MAV_PORT=$(echo "$PORTS" | head -1) && break
        sleep 1
    done
fi
[ -z "$MAV_PORT" ] && MAV_PORT=18570
echo "      PX4 UDP 监听端口: $(echo $PORTS | tr '\n' ' ')"
# PX4 在这些端口上监听(server); MAVSDK 应作为 client 连出, 用 udpout:// 避免 bind 冲突
MISSION_ADDR="udpout://127.0.0.1:${MAV_PORT}"
echo "      >>> 使用 offboard 地址: $MISSION_ADDR"

# ---------- [4] 执行任务 ----------
echo "[4/6] 执行方形航线任务 (addr=$MISSION_ADDR) ..."
MAVSDK_ADDR="$MISSION_ADDR" $PYTHON "$MISSION" --addr "$MISSION_ADDR" --out-dir "$WIN_DIR"
MISSION_RC=$?
echo "      任务退出码: $MISSION_RC"
CSV_FILE=$(ls -t "$WIN_DIR"/trajectory_*.csv 2>/dev/null | head -1)

# ---------- [5] 收集 ulg ----------
echo "[5/6] 收集 PX4 ulg 日志 ..."
# SITL 的 ulg 在 PX4 进程退出时才 finalize 落盘, 所以必须先杀 PX4, 等文件出现, 再拷贝
pkill -f "px4_sitl" 2>/dev/null
pkill -f "bin/px4" 2>/dev/null
echo "      已停止 PX4, 等待 ulg 落盘 (最多 15s) ..."
ULG=""
for i in $(seq 1 15); do
    ULG=$(ls -t \
            "$PX4_DIR"/build/px4_sitl_default/log/*/*.ulg \
            "$PX4_DIR"/build/px4_sitl_default/rootfs/log/*/*.ulg \
            2>/dev/null | head -1)
    [ -n "$ULG" ] && break
    sleep 1
done
ULG_WIN=""
if [ -n "$ULG" ]; then
    sleep 1   # 再等 1s 确保写完整
    cp "$ULG" "$WIN_DIR/"
    ULG_WIN="$WIN_DIR/$(basename "$ULG")"
    SZ=$(stat -c%s "$ULG_WIN" 2>/dev/null || echo "?")
    echo "      已复制: $ULG_WIN ($SZ bytes)"
else
    echo "      未找到 ulg 日志。"
    echo "      可能原因: SDLOG_MODE=0 或 PX4 在 [4] 阶段已异常退出。"
    echo "      修复: 在 PX4 控制台执行 'param set SDLOG_MODE 1' 后 'param commit',"
    echo "            重启 PX4 再飞一次即可生成 ulg。"
fi

# ---------- [6] 写状态文件 (供 Windows 侧助手读取) ----------
echo "[6/6] 写状态文件 ..."
{
    echo "MISSION_RC=$MISSION_RC"
    echo "MAV_PORT=$MAV_PORT"
    echo "CSV=$CSV_FILE"
    echo "ULG=$ULG_WIN"
    echo "DONE=$(date '+%Y-%m-%d %H:%M:%S')"
} > "$STATUS"
echo "      状态: $STATUS"

echo
echo "==================================================="
echo " 飞行+采集完成。产物在 Windows 侧 D:\\AirSim\\mission\\ :"
echo "   CSV : $CSV_FILE"
echo "   ULG : $ULG_WIN"
echo " 把以上路径告诉 Windows 侧助手即可开始分析。"
echo "==================================================="
