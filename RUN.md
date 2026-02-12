# 🚀 실행 가이드 (Run Guide)

이 문서는 드론 회피 AI 학습을 실행하는 **완전한 가이드**입니다.

---

## 📋 **사전 체크리스트**

실행 전 다음을 확인하세요:

```bash
# ✅ ROS2 Humble 설치 확인
ros2 --version
# 출력: ros2 doctor version: humble

# ✅ Python 패키지 설치 확인
python3 -c "import torch, gymnasium, stable_baselines3; print('OK')"

# ✅ PX4-Autopilot 존재 확인
ls ~/workspace/air_defense_evasion_drone_project/PX4-Autopilot

# ✅ ROS2 워크스페이스 빌드 확인
source ~/workspace/air_defense_evasion_drone_project/ros2_ws/install/setup.bash
ros2 pkg list | grep flight_control
```

모두 정상이면 다음 단계로 진행하세요.

---

## 🎯 **실행 순서 (5개 터미널)**

### **터미널 1️⃣: Micro-XRCE-DDS-Agent**

```bash
# 실행
MicroXRCEAgent udp4 -p 8888
```

**예상 출력:**
```
[1708001234.567890] info     | UDPv4AgentLinux.cpp | init | running... | port: 8888
[1708001235.123456] info     | Root.cpp           | create_client | session: 0x...
```

**역할:** PX4 SITL ↔ ROS2 메시지 브릿지

**문제 해결:**
```bash
# "command not found" 에러 시
which MicroXRCEAgent
# 없으면 INSTALL.md의 4.2절 참고하여 설치
```

---

### **터미널 2️⃣: PX4 SITL + Gazebo**

```bash
cd ~/workspace/air_defense_evasion_drone_project/PX4-Autopilot

# 빠른 학습 (권장) - GUI 없음, 3배속
HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris

# 또는 시각화 보며 학습 (느림)
make px4_sitl gazebo-classic_iris
```

**예상 출력:**
```
INFO  [init] Mixer: etc/mixers/quad_w.main.mix on /dev/pwm_output0
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s
INFO  [mavlink] MAVLink only on localhost
INFO  [uxrce_dds_client] synchronized with time offset ...ns  ← 중요!
pxh>  ← 이 프롬프트가 나오면 정상
```

**환경 변수 설명:**
- `HEADLESS=1`: Gazebo 창 숨김 (GPU/CPU 절약)
- `PX4_SIM_SPEED_FACTOR=3`: 시뮬레이션 3배 가속
  - 1 = 실시간
  - 3 = 3배속 (권장, 학습 3배 빠름)
  - 10 = 10배속 (불안정할 수 있음)

**문제 해결:**
```bash
# "uxrce_dds_client synchronized" 로그가 안 보이면
# → 터미널 1의 Agent가 실행 중인지 확인

# Gazebo가 안 열리면
killall -9 gzserver gzclient
make px4_sitl gazebo-classic_iris
```

---

### **터미널 3️⃣: Turret Simulator**

```bash
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws

# ROS2 환경 설정
source /opt/ros/humble/setup.bash  # zsh 사용자는 setup.zsh
source install/setup.bash

# 터렛 시뮬레이터 실행
ros2 run flight_control turret_sim
```

**예상 출력:**
```
[INFO] [turret_sim_node]: Fire! Active Bullets: 1
[INFO] [turret_sim_node]: Fire! Active Bullets: 2
[INFO] [turret_sim_node]: Fire! Active Bullets: 3
...
```

**토픽 확인:**
```bash
# 새 터미널에서
ros2 topic list | grep turret
# 출력: /turret/bullets, /turret/visual

# 총알 데이터 확인
ros2 topic echo /turret/bullets --once
# 출력: data: [20.0, 5.0, 0.0, 15.2, -3.1, -2.5, ...]
```

**문제 해결:**
```bash
# "ros2 run" 에러 시
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

### **터미널 4️⃣: 학습 시작 (Training)**

```bash
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/workspace/air_defense_evasion_drone_project/ros2_ws/install/setup.bash

# 학습 시작
python3 train.py
```

**예상 출력 (정상):**
```
Training Start! Logs: ./logs/20260213-123456, Models: ./models/20260213-123456

>>> Resetting Environment...
>>> Waiting for initial state...
>>> Initial State - Nav: 0, Arm: 1  ← Nav, Arm이 0이 아니어야 함!
>>> Phase 1: Sending Offboard heartbeat for 2 seconds...
>>> Phase 2: Switching to Offboard mode...
>>> Offboard mode activated! (Attempt: 3)
>>> Phase 3: Arming...
>>> Armed successfully! (Attempt: 2)
>>> Auto-Takeoff to 2.0m...
>>> Ready to Fly! Handing over control to AI.

-----------------------------
| time/              |      |
|    fps             | 10   |
|    iterations      | 1    |
| rollout/           |      |
|    ep_len_mean     | 45   |
|    ep_rew_mean     | -23  |
-----------------------------
```

**학습 지표 설명:**
- `ep_len_mean`: 평균 에피소드 길이 (생존 시간) → **증가해야 함**
- `ep_rew_mean`: 평균 보상 → **증가해야 함**
- `fps`: 학습 속도 (높을수록 빠름)

**문제 해결:**
```bash
# "Nav: 0, Arm: 0"에서 멈추면
# → 터미널 1, 2가 제대로 실행 중인지 확인
# → PX4에서 "uxrce_dds_client synchronized" 로그 확인

# "Arming Failed" 반복되면
# → 모든 터미널 종료 (Ctrl+C)
# → 터미널 1부터 다시 순서대로 시작

# "ModuleNotFoundError: No module named 'flight_control'" 에러
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

### **터미널 5️⃣ (선택): TensorBoard**

```bash
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control

# TensorBoard 실행
tensorboard --logdir=./logs
```

**접속:** 브라우저에서 `http://localhost:6006`

**확인할 그래프:**
- **rollout/ep_rew_mean**: 보상 추이 (↗ 상승하면 학습 중)
- **rollout/ep_len_mean**: 생존 시간 (↗ 상승하면 잘 버팀)
- **train/policy_loss**: 정책 손실 (안정화되어야 함)

---

## 🛑 **학습 중지**

### **안전한 종료:**
1. 터미널 4 (train.py)에서 `Ctrl+C`
2. 잠시 대기 (모델 저장)
3. 터미널 3 (turret_sim)에서 `Ctrl+C`
4. 터미널 2 (PX4)에서 `Ctrl+C`
5. 터미널 1 (Agent)에서 `Ctrl+C`

### **강제 종료 (문제 발생 시):**
```bash
killall -9 python3 px4 gzserver gzclient MicroXRCEAgent
```

---

## 📊 **학습 결과 확인**

### **저장된 파일 위치:**
```
./logs/20260213-123456/        # TensorBoard 로그
./models/20260213-123456/      # 학습된 모델
├── drone_ppo_100000_steps.zip  # 10만 스텝 체크포인트
├── drone_ppo_200000_steps.zip  # 20만 스텝 체크포인트
└── drone_ppo_final.zip         # 최종 모델 (100만 스텝)
```

### **모델 테스트:**
```python
# test_model.py (새로 생성)
from stable_baselines3 import PPO
from flight_control.drone_env import DroneEnv

# 학습된 모델 로드
model = PPO.load("./models/20260213-123456/drone_ppo_final")

# 환경 생성
env = DroneEnv()
obs, _ = env.reset()

# 테스트 실행
for i in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, _, _ = env.step(action)
    print(f"Step {i}: Reward={reward:.2f}")
    
    if done:
        print("Episode finished!")
        obs, _ = env.reset()
```

---

## 🔧 **파라미터 조정**

### **학습 속도 높이기 (train.py):**
```python
# Line 29
learning_rate=0.001,  # 0.0003 → 0.001 (3배 빠름, 불안정할 수 있음)
```

### **학습 시간 줄이기 (train.py):**
```python
# Line 45
model.learn(total_timesteps=100_000)  # 1,000,000 → 100,000 (테스트용)
```

### **터렛 난이도 조정 (turret_sim_new.py):**
```python
# Line 19-20
self.bullet_speed = 30.0   # 50 → 30 (쉬움)
self.fire_rate = 1.0       # 0.5 → 1.0 (느림)
```

---

## 🎯 **학습 성공 기준**

### **1단계 (초기 - 1~10만 스텝):**
- ✅ 드론이 추락하지 않고 공중에 떠있음
- ✅ 이륙 성공률 > 90%
- ✅ `ep_len_mean` > 50

### **2단계 (중기 - 10~50만 스텝):**
- ✅ 총알을 일부 회피함
- ✅ 목표 방향으로 이동 시작
- ✅ `ep_rew_mean` > 0

### **3단계 (후기 - 50~100만 스텝):**
- ✅ 총알을 효과적으로 회피
- ✅ 목표 도달 성공 (일부 에피소드)
- ✅ `ep_rew_mean` > 50

---

## 📝 **체크리스트 (실행 전)**

```
□ 터미널 1: MicroXRCEAgent 실행 중
□ 터미널 2: PX4 SITL "pxh>" 프롬프트 확인
□ 터미널 2: "uxrce_dds_client synchronized" 로그 확인
□ 터미널 3: turret_sim "Fire!" 로그 확인
□ 터미널 4: train.py 실행, "Nav: X, Arm: Y" (X,Y ≠ 0)
□ 터미널 5: TensorBoard 그래프 확인 (선택)
```

---

## ⚡ **빠른 참조 (Quick Reference)**

### **전체 명령어 (복사용):**
```bash
# Terminal 1
MicroXRCEAgent udp4 -p 8888

# Terminal 2
cd ~/workspace/air_defense_evasion_drone_project/PX4-Autopilot && HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris

# Terminal 3
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run flight_control turret_sim

# Terminal 4
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control && source /opt/ros/humble/setup.bash && source ~/workspace/air_defense_evasion_drone_project/ros2_ws/install/setup.bash && python3 train.py

# Terminal 5 (Optional)
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control && tensorboard --logdir=./logs
```

---

**학습 시작하세요! 🚀**
