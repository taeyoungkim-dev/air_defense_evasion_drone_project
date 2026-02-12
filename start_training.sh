#!/bin/bash
# ============================================
# AA-Evasion Drone Training Launcher
# ============================================
# 
# 이 스크립트는 학습에 필요한 모든 정보를 출력합니다.
# 각 터미널을 수동으로 열어서 명령어를 실행하세요.
#

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  AA-Evasion Drone Training Guide${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 프로젝트 경로 설정
PROJECT_DIR="$HOME/workspace/air_defense_evasion_drone_project"
ROS2_WS="$PROJECT_DIR/ros2_ws"
PX4_DIR="$PROJECT_DIR/PX4-Autopilot"
FLIGHT_CONTROL="$ROS2_WS/src/flight_control/flight_control"

echo -e "${BLUE}📋 5개의 터미널을 열고 다음 명령어를 순서대로 실행하세요:${NC}"
echo ""

# Terminal 1
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}터미널 1️⃣: Micro-XRCE-DDS-Agent${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "MicroXRCEAgent udp4 -p 8888"
echo ""
echo -e "${BLUE}✓ [CREATE Client] 로그가 나오면 정상${NC}"
echo ""

# Terminal 2
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}터미널 2️⃣: PX4 SITL + Gazebo${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "cd $PX4_DIR"
echo "HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris"
echo ""
echo -e "${BLUE}✓ 'pxh>' 프롬프트와 'uxrce_dds_client synchronized' 로그 확인${NC}"
echo ""

# Terminal 3
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}터미널 3️⃣: Turret Simulator${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "cd $ROS2_WS"
echo "source /opt/ros/humble/setup.bash  # 또는 setup.zsh"
echo "source install/setup.bash"
echo "ros2 run flight_control turret_sim"
echo ""
echo -e "${BLUE}✓ 'Fire! Active Bullets: X' 로그가 나오면 정상${NC}"
echo ""

# Terminal 4
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}터미널 4️⃣: Training (학습 시작)${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "cd $FLIGHT_CONTROL"
echo "source /opt/ros/humble/setup.bash"
echo "source $ROS2_WS/install/setup.bash"
echo "python3 train.py"
echo ""
echo -e "${BLUE}✓ 'Initial State - Nav: X, Arm: Y' (X,Y ≠ 0) 확인${NC}"
echo ""

# Terminal 5
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}터미널 5️⃣: TensorBoard (선택)${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "cd $FLIGHT_CONTROL"
echo "tensorboard --logdir=./logs"
echo ""
echo -e "${BLUE}✓ 브라우저에서 http://localhost:6006 접속${NC}"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  추가 정보${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}📚 상세 가이드: $PROJECT_DIR/RUN.md${NC}"
echo -e "${BLUE}📝 TODO 리스트: $PROJECT_DIR/TODO_PROJECTILE_SYSTEM.md${NC}"
echo ""
echo -e "${RED}⚠️  문제 발생 시:${NC}"
echo "   killall -9 px4 gzserver gzclient MicroXRCEAgent python3"
echo "   그 후 터미널 1부터 다시 시작"
echo ""
echo -e "${GREEN}Good luck! 🚀${NC}"
