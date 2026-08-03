#!/bin/bash
# ===================================================================
# diag_ulg.sh — 排查 PX4 ulg 日志为何没生成 (在 WSL2 Ubuntu 中运行)
#
# 用法:
#       bash /mnt/d/AirSim/mission/diag_ulg.sh
#
# 本脚本自动完成你列的检查项:
#   [1] 列出 SITL 日志目录 (log/ 与 rootfs/log/ 两种布局都查)
#   [2] 找最新 .ulg 并给出大小/时间
#   [3] 若 PX4 正在运行, 通过 MAVSDK 读取 SDLOG_MODE / SDLOG_PROFILE 等参数
#   [4] 给出结论与一次性修复命令
# ===================================================================

set -u

PX4_DIR="${HOME}/PX4-Autopilot"
PYTHON=""
if command -v python3 >/dev/null 2>&1; then PYTHON=python3
elif command -v python  >/dev/null 2>&1; then PYTHON=python
else PYTHON=""; fi

echo "==================================================="
echo " PX4 ulg 日志排查"
echo " PX4_DIR = $PX4_DIR"
echo "==================================================="

# ---------- [1] 日志目录 ----------
echo "[1] SITL 日志目录与内容 (按时间):"
for d in "$PX4_DIR/build/px4_sitl_default/log" "$PX4_DIR/build/px4_sitl_default/rootfs/log"; do
    if [ -d "$d" ]; then
        echo "     目录: $d"
        ls -lt "$d" 2>/dev/null | head -6
    else
        echo "     目录不存在: $d"
    fi
done

# ---------- [2] 最新 ulg ----------
echo
echo "[2] 最新 .ulg 文件 (全局按时间):"
ULG=$(ls -t "$PX4_DIR"/build/px4_sitl_default/log/*/*.ulg \
           "$PX4_DIR"/build/px4_sitl_default/rootfs/log/*/*.ulg 2>/dev/null | head -1)
if [ -n "$ULG" ]; then
    SZ=$(stat -c%s "$ULG" 2>/dev/null || echo "?")
    echo "     >>> $ULG"
    echo "         大小: $SZ bytes   修改时间: $(date -r "$ULG" 2>/dev/null)"
else
    echo "     >>> 未找到任何 .ulg (日志从未生成, 高度怀疑 SDLOG 未启用)"
fi

# ---------- [3] 通过 MAVSDK 读 SDLOG 参数 ----------
echo
echo "[3] 若 PX4 正在运行, 读取 SDLOG 相关参数:"
PORTS=""
if command -v ss >/dev/null 2>&1; then
    PORTS=$(ss -ulnp 2>/dev/null | grep -i px4 | grep -oP ':\K[0-9]+' | tr -d '\r' | sort -n | uniq)
elif command -v netstat >/dev/null 2>&1; then
    PORTS=$(netstat -ulnp 2>/dev/null | grep -i px4 | grep -oP ':\K[0-9]+' | tr -d '\r' | sort -n | uniq)
fi
ADDR=""
for cand in 18570 14580 14280 13030; do
    echo "$PORTS" | grep -qx "$cand" && ADDR="udpout://127.0.0.1:$cand" && break
done
[ -z "$ADDR" ] && ADDR="udpout://127.0.0.1:18570"

if [ -n "$PYTHON" ]; then
$PYTHON - "$ADDR" <<'PYEOF'
import sys, asyncio
from mavsdk import System
async def main():
    addr = sys.argv[1]
    d = System()
    try:
        await d.connect(system_address=addr)
        async for st in d.core.connection_state():
            if st.is_connected: break
    except Exception as e:
        print("     PX4 未连接 (%s): %s" % (addr, e))
        print("     (若 PX4 没在跑, 跳过本步; 仍可用 [1][2] 的文件判断)")
        return
    for p in ("SDLOG_MODE","SDLOG_PROFILE","SDLOG_DIR","SDLOG_UUID"):
        try:
            res = await d.param.get_param_int(p)
            val = getattr(res, "value", res)
            print("     %s = %s" % (p, val))
        except Exception as e:
            print("     %s = (读取失败: %s)" % (p, e))
asyncio.run(main())
PYEOF
else
    echo "     (未找到 python, 跳过; 可在 PX4 控制台手动执行: param show SDLOG_MODE)"
fi

# ---------- [4] 结论与修复 ----------
echo
echo "[4] 结论与修复建议:"
if [ -z "$ULG" ]; then
    echo "     >>> 没有 ulg。最常见原因: SDLOG_MODE=0 (PX4 默认不记录飞行日志)。"
    echo
    echo "     一次性修复 (在 PX4 控制台 pxh> 执行, 或 QGC 参数页设置):"
    echo "         param set SDLOG_MODE 1"
    echo "         param commit"
    echo "     然后重启 PX4 (重新 make px4_sitl none_iris), 再飞一次即生成 ulg。"
    echo "     注: SITL 的参数会持久化到构建目录, 设一次即可, 后续飞行都会记录。"
else
    echo "     >>> 已找到 ulg。若 Windows 侧没拿到, 是 run_flight_mission.sh 旧路径问题,"
    echo "         现已修复 (同时搜 log/ 与 rootfs/log/), 重跑飞行即可自动拷到 D:\\AirSim\\mission\\。"
fi
echo "==================================================="
