#!/usr/bin/env bash
# 故障注入回归套件启动器 (B 项目 step 2)
# 起 SIH -> 等 Ready -> 依次跑 5 个故障用例 -> 各输出 PASS/FAIL
# 每个用例重启 SIH 保证状态干净 (CI 确定性)。
# 用法:
#   run_fault_test.sh <REPO> [单用例号 1-5, 省略则全跑]
set -e
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH

arg="${1:-/mnt/d/AirSim/mission/px4-airsim-drone-sim}"
case "$arg" in
  /mnt/*)       REPO="$arg" ;;
  [A-Za-z]:*)   d="${arg:0:1}"; d="${d,,}"; rest="${arg:3}"; rest="${rest//\\//}"; REPO="/mnt/${d}/${rest}" ;;
  *)            REPO="/mnt/d/AirSim/mission/px4-airsim-drone-sim" ;;
esac
SINGLE="${2:-}"

if [ ! -f "$REPO/scripts/fault_test.py" ]; then
  echo "ERROR: 找不到 fault_test.py (REPO=$REPO)"
  exit 1
fi

OUTDIR=/mnt/d/AirSim/mission/fault_cases
mkdir -p "$OUTDIR"

start_sih() {
  cd ~/PX4-Autopilot
  # 先清掉上一次可能残留的 SIH/PX4 进程, 避免旧实例仍占用 14540/14580 端口
  # (旧实例若已 failsafe/已解锁, 新 MAVSDK 连上它会被 COMMAND_DENIED)。
  pkill -9 -f px4_sitl 2>/dev/null; pkill -9 -f sihsim_quadx 2>/dev/null
  pkill -9 -f "make px4_sitl" 2>/dev/null
  pkill -9 -f "fault_test.py --publisher" 2>/dev/null || true
  # 清掉上一轮残留的 mavsdk_server(默认端口 50051 会被新进程复用, 导致 publisher
  # 不拉起自己的 server, 进而 SIGKILL publisher 时 setpoint 仍在发 -> offboard 失效保护不触发)
  pkill -9 -f mavsdk_server 2>/dev/null || true
  sleep 2
  PX4BIN=build/px4_sitl_default/bin/px4
  if [ -x "$PX4BIN" ]; then
    nohup "$PX4BIN" < /dev/null > /tmp/px4_sih.log 2>&1 &
  else
    nohup make px4_sitl_default < /dev/null > /tmp/px4_sih.log 2>&1 &
  fi
  PX4PID=$!
  READY=0
  for i in $(seq 1 240); do
    if grep -qiE ':38F4|:390C' /proc/net/udp 2>/dev/null; then
      echo "PX4 SIH ready after ~$((i*2))s"; READY=1; break
    fi
    if ! kill -0 $PX4PID 2>/dev/null; then
      echo "PX4 进程退出(编译/启动失败). 日志尾部:"; tail -40 /tmp/px4_sih.log
      exit 1
    fi
    sleep 2
  done
  if [ "$READY" -ne 1 ]; then
    echo "PX4 超时未就绪"; tail -40 /tmp/px4_sih.log
    kill $PX4PID 2>/dev/null; exit 1
  fi
  echo "$PX4PID" > /tmp/sih_pid
}

run_case() {
  local c="$1"
  echo "########## CASE $c ##########"
  start_sih
  PX4PID=$(cat /tmp/sih_pid)
  set +e
  timeout 240 python3 "$REPO/scripts/fault_test.py" --case "$c" --out-dir "$OUTDIR" > /tmp/fault_case${c}.log 2>&1
  RC=$?
  set -e
  cat /tmp/fault_case${c}.log
  kill $PX4PID 2>/dev/null
  pkill -9 -f "fault_test.py --publisher" 2>/dev/null || true
  pkill -9 -f mavsdk_server 2>/dev/null || true
  if [ "$RC" -eq 0 ]; then
    echo "[RESULT] case $c = PASS"
  else
    echo "[RESULT] case $c = FAIL (rc=$RC)"
    echo "== PX4 日志尾部 =="; tail -30 /tmp/px4_sih.log || true
  fi
  return $RC
}

OVERALL=0
if [ -n "$SINGLE" ]; then
  run_case "$SINGLE" || OVERALL=1
else
  for c in 1 2 3 4 5; do
    run_case "$c" || OVERALL=1
  done
fi

echo "########## OVERALL = $([ $OVERALL -eq 0 ] && echo ALL_PASS || echo SOME_FAIL) ##########"
exit $OVERALL
