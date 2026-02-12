# 🔧 편리한 Alias 설정

다음 내용을 `~/.bashrc` 또는 `~/.zshrc`에 추가하면 명령어를 간단하게 실행할 수 있습니다.

---

## 📝 **설정 방법**

### **1. 파일 열기:**
```bash
# Bash 사용자
nano ~/.bashrc

# Zsh 사용자
nano ~/.zshrc
```

### **2. 아래 내용 복사해서 붙여넣기:**

```bash
# ============================================
# AA-Evasion Drone Project Aliases
# ============================================

# 프로젝트 경로
export DRONE_PROJECT="$HOME/workspace/air_defense_evasion_drone_project"
export DRONE_ROS2="$DRONE_PROJECT/ros2_ws"
export DRONE_FLIGHT="$DRONE_ROS2/src/flight_control/flight_control"

# 빠른 이동
alias cdrone="cd $DRONE_PROJECT"
alias cdrone-ros="cd $DRONE_ROS2"
alias cdrone-px4="cd $DRONE_PROJECT/PX4-Autopilot"
alias cdrone-train="cd $DRONE_FLIGHT"

# ROS2 환경 설정
alias drone-source="source /opt/ros/humble/setup.bash && source $DRONE_ROS2/install/setup.bash"

# 학습 관련
alias drone-train="cdrone-train && drone-source && python3 train.py"
alias drone-test="cdrone-train && drone-source && python3 test_env.py"
alias drone-tb="cdrone-train && tensorboard --logdir=./logs"

# 빌드
alias drone-build="cd $DRONE_ROS2 && colcon build --symlink-install && source install/setup.bash"
alias drone-clean="cd $DRONE_ROS2 && rm -rf build install log"

# 프로세스 관리
alias drone-kill="killall -9 px4 gzserver gzclient MicroXRCEAgent python3 2>/dev/null; echo 'All processes killed'"
alias drone-check="ps aux | grep -E '(px4|gazebo|MicroXRCE|python3.*train)' | grep -v grep"

# 로그 확인
alias drone-logs="ls -lht $DRONE_FLIGHT/logs/ | head -10"
alias drone-models="ls -lht $DRONE_FLIGHT/models/ | head -10"

# 가이드 보기
alias drone-help="cat $DRONE_PROJECT/RUN.md | less"
alias drone-start="$DRONE_PROJECT/start_training.sh"

# 토픽 모니터링
alias drone-topics="ros2 topic list | grep -E '(fmu|turret)'"
alias drone-bullets="ros2 topic echo /turret/bullets --once"
alias drone-status="ros2 topic echo /fmu/out/vehicle_status_v1 --once --qos-reliability best_effort"
```

### **3. 저장 및 적용:**
```bash
# Ctrl+O (저장), Ctrl+X (종료)

# 적용
source ~/.bashrc  # 또는 source ~/.zshrc
```

---

## 🚀 **사용 예시**

### **빠른 이동:**
```bash
cdrone          # 프로젝트 루트로
cdrone-train    # 학습 코드 위치로
cdrone-px4      # PX4 디렉토리로
```

### **학습 시작 (터미널 4용):**
```bash
drone-train     # 자동으로 경로 이동 + 환경 설정 + train.py 실행
```

### **TensorBoard (터미널 5용):**
```bash
drone-tb        # TensorBoard 실행
```

### **빌드 & 클린:**
```bash
drone-build     # ROS2 워크스페이스 빌드
drone-clean     # 빌드 파일 삭제
```

### **프로세스 관리:**
```bash
drone-check     # 실행 중인 프로세스 확인
drone-kill      # 모든 관련 프로세스 강제 종료
```

### **토픽 모니터링:**
```bash
drone-topics    # 토픽 리스트 확인
drone-bullets   # 총알 데이터 확인
drone-status    # 드론 상태 확인
```

### **로그 확인:**
```bash
drone-logs      # 최근 로그 파일 10개
drone-models    # 최근 모델 파일 10개
```

### **도움말:**
```bash
drone-help      # RUN.md 보기
drone-start     # 학습 시작 가이드 보기
```

---

## 🎯 **추천 워크플로우**

### **학습 시작:**
```bash
# 터미널 1
MicroXRCEAgent udp4 -p 8888

# 터미널 2
cdrone-px4
HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris

# 터미널 3
drone-source
ros2 run flight_control turret_sim

# 터미널 4
drone-train

# 터미널 5
drone-tb
```

### **문제 발생 시:**
```bash
drone-kill      # 모든 프로세스 종료
drone-check     # 프로세스 확인 (비어있어야 함)
# 터미널 1부터 다시 시작
```

### **빌드 후 학습:**
```bash
drone-build     # 빌드
drone-train     # 학습 시작
```

---

## 💡 **팁**

### **1. 터미널 자동 이름 설정 (tmux 사용자):**
```bash
# ~/.tmux.conf에 추가
bind-key D split-window -h \; \
  send-keys "MicroXRCEAgent udp4 -p 8888" C-m \; \
  split-window -v \; \
  send-keys "cdrone-px4 && HEADLESS=1 make px4_sitl gazebo-classic_iris" C-m
```

### **2. 학습 상태 실시간 모니터링:**
```bash
watch -n 1 drone-check
```

### **3. 자동 환경 설정 (zsh 전용):**
```bash
# ~/.zshrc에 추가
if [[ $PWD == $DRONE_PROJECT* ]]; then
    drone-source 2>/dev/null
fi
```

---

**이제 명령어가 훨씬 간단해졌습니다! 🎉**
