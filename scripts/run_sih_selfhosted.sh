#!/usr/bin/env bash
# 在 WSL 内拉起 PX4 SIH -> 飞 10x10 方框 -> check_square 断言 >80
# 由 sim-flight-selfhosted.yml 的 self-hosted runner 调用。
# 参数 $1 = 仓库在 WSL 中的路径 (runner checkout 经 wslpath -a 转换后传入)
set -e
REPO="$1"
RC=0

echo "== start PX4 SIH (WSL, headless, 内置物理引擎, 无需 AirSim/Gazebo) =="
cd ~/PX4-Autopilot
# 后台拉起 PX4；SIH 监听 udp 14580、把 mavlink 发往 14540(MAVSDK 客户端侧)
nohup make px4_sitl sihsim_quadx > /tmp/px4_sih.log 2>&1 &
PX4PID=$!
echo "px4 pid=$PX4PID"

echo "== wait for PX4 SIH mavlink endpoint (listens udp :14580) =="
READY=0
for i in $(seq 1 60); do
  if ss -lunp | grep -q :14580; then echo "PX4 SIH ready after ~$((i*2))s"; READY=1; break; fi
  sleep 2
done
if [ "$READY" -ne 1 ]; then
  echo "PX4 failed to start. tail of /tmp/px4_sih.log:"; tail -40 /tmp/px4_sih.log
  kill $PX4PID 2>/dev/null; exit 1
fi

echo "== fly square + score =="
# square_mission.py 默认把轨迹写到 Windows 侧 D:\AirSim\mission (WSL:/mnt/d/AirSim/mission),
# 而非 checkout 目录, 因此从这里抓轨迹, 并用脚本末尾打印的路径精确锁定最新文件
MISSION="/mnt/d/AirSim/mission"
mkdir -p "$MISSION"
OUT=$(python3 "$REPO/square_mission.py" --out-dir "$MISSION" 2>&1 | tee /dev/stderr | grep -oP '轨迹: \K\S+')
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
