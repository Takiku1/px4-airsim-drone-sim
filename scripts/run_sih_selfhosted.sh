#!/usr/bin/env bash
# 在 WSL 内拉起 PX4 SIH -> 飞 10x10 方框 -> check_square 断言 >80
# 由 sim-flight-selfhosted.yml 的 self-hosted runner 调用。
# 参数 $1 = 仓库在 WSL 中的路径 (Windows 侧 D:\AirSim\mission\px4-airsim-drone-sim 经 wslpath -u 转换后传入)
set -e
# 非交互 wsl shell 的 PATH 可能不全, 显式补上标准路径
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
REPO="$1"
RC=0

# 守卫: 路径必须有效且含 square_mission.py, 否则提前报错(避免下面 bash 找不到文件 -> 127)
if [ -z "$REPO" ] || [ ! -f "$REPO/square_mission.py" ]; then
  echo "ERROR: REPO 路径无效或脚本缺失: REPO='$REPO'"
  echo "  期望传入含 square_mission.py 的仓库根目录 (如 /mnt/d/AirSim/mission/px4-airsim-drone-sim)"
  exit 1
fi

echo "== start PX4 SIH (WSL, headless, 内置物理引擎, 无需 AirSim/Gazebo) =="
cd ~/PX4-Autopilot
# 后台拉起 PX4；SIH 监听 udp 14580、把 mavlink 发往 14540(MAVSDK 客户端侧)
nohup make px4_sitl sihsim_quadx > /tmp/px4_sih.log 2>&1 &
PX4PID=$!
echo "px4 pid=$PX4PID"

echo "== wait for PX4 SIH UDP endpoint :14580 (端口十六进制 38F4) =="
READY=0
for i in $(seq 1 90); do
  # 优先 ss; 缺失则回退到 /proc/net/udp 解析(14580=0x38F4, 14540=0x390C)
  if { command -v ss >/dev/null 2>&1 && ss -lunp 2>/dev/null | grep -qE ':14580|:14540'; } \
     || grep -qiE ':38F4|:390C' /proc/net/udp 2>/dev/null; then
    echo "PX4 SIH ready after ~$((i*2))s"; READY=1; break
  fi
  sleep 2
done
if [ "$READY" -ne 1 ]; then
  echo "PX4 failed to start. tail of /tmp/px4_sih.log:"; tail -40 /tmp/px4_sih.log
  kill $PX4PID 2>/dev/null; exit 1
fi

echo "== fly square + score =="
# square_mission.py 默认把轨迹写到 Windows 侧 D:\AirSim\mission (WSL:/mnt/d/AirSim/mission),
# 而非仓库目录, 因此从这里抓轨迹, 并用脚本末尾打印的路径精确锁定最新文件
MISSION="/mnt/d/AirSim/mission"
mkdir -p "$MISSION"
# 注意: grep 找不到时返回非 0, 用 || true 防止 set -e 提前退出
OUT=$(python3 "$REPO/square_mission.py" --out-dir "$MISSION" 2>&1 | tee /dev/stderr | grep -oP '轨迹: \K\S+' || true)
latest="$OUT"
if [ -z "$latest" ] || [ ! -f "$latest" ]; then
  echo "ERROR: square_mission.py 未产生轨迹 CSV (捕获到: '$latest')"
  kill $PX4PID 2>/dev/null; exit 1
fi
echo "scoring $latest"
python3 "$REPO/check_square.py" --input "$latest" --threshold 80 || RC=$?

kill $PX4PID 2>/dev/null
echo "exit code = $RC"
exit $RC
