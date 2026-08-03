#!/bin/bash
# extract_ulg.sh — 从 WSL 拷贝最新 ulg 到 Windows 并转 CSV
# 用法（PowerShell）: wsl.exe -d Ubuntu-22.04 -- bash /mnt/d/AirSim/mission/extract_ulg.sh
set -euo pipefail

PX4_DIR="$HOME/PX4-Autopilot"
WIN_DIR="/mnt/d/AirSim/mission"
CSV_DIR="$WIN_DIR/ulg_csv"

echo "=================================================== "
echo " ULG 提取 + CSV 转换"
echo " PX4_DIR = $PX4_DIR"
echo "=================================================== "

# [1] 找最新 ulg
ULG=$(ls -t "$PX4_DIR"/build/px4_sitl_default/rootfs/log/*/*.ulg \
           "$PX4_DIR"/build/px4_sitl_default/log/*/*.ulg \
           2>/dev/null | head -1)

if [ -z "$ULG" ]; then
    echo "ERROR: 未找到任何 .ulg 文件。请先飞一次任务生成日志。"
    exit 1
fi

ULG_SIZE=$(stat -c%s "$ULG" 2>/dev/null || echo "?")
echo "[1] 最新 ulg: $ULG"
echo "    大小: $ULG_SIZE bytes"

# [2] 拷贝到 Windows
cp "$ULG" "$WIN_DIR/"
ULG_BASE="$(basename "$ULG")"
echo "[2] 已拷贝到 Windows: D:\\AirSim\\mission\\$ULG_BASE"

# [3] 装 pyulog + 转 CSV
export PATH="$HOME/.local/bin:$PATH"
if ! command -v ulog2csv >/dev/null 2>&1; then
    echo "[3] 安装 pyulog ..."
    python3 -m pip install --user -q pyulog
fi

mkdir -p "$CSV_DIR"
echo "[3] 正在转换 ulg -> CSV ..."
( cd "$CSV_DIR" && ulog2csv "$ULG" )

# [4] 列出关键 CSV
echo ""
echo "---- 生成的 CSV 文件 ----"
ls "$CSV_DIR" | grep -iE "vehicle_local_position|trajectory|actuator|attitude|vehicle_gps|sensor_combined" || true
echo ""
echo "✅ 完成！关键文件:"
echo "   ulg : D:\\AirSim\\mission\\$ULG_BASE"
echo "   CSV : D:\\AirSim\\mission\\ulg_csv\\"
