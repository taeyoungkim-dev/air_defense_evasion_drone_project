# 🚁 Project: AA-Evasion (Anti-Air Evasion Drone)

> **Deep Reinforcement Learning based UAV Navigation in Hostile Environments**

## 📋 목차 (Table of Contents)
- [빠른 실행](#-빠른-실행-quick-start)
- [개요](#1-개요-overview)
- [설치 가이드](#설치-installation)
- [프로젝트 구조](#프로젝트-구조-project-structure)
- [개발 환경](#2-개발-환경-development-environment)
- [시나리오 및 위협 정의](#3-시나리오-및-위협-정의-scenario--threat)
- [강화학습 설계](#4-강화학습-설계-rl-design)
- [개발 로드맵](#6-개발-로드맵-roadmap)

---

## 🚀 빠른 실행 (Quick Start)

학습을 시작하려면 **5개의 터미널**을 순서대로 실행하세요.

### **터미널 1: Micro-XRCE-DDS-Agent (PX4-ROS2 브릿지)**
```bash
MicroXRCEAgent udp4 -p 8888
```
**역할:** PX4와 ROS2 간 메시지 변환

---

### **터미널 2: PX4 SITL + Gazebo (드론 시뮬레이터)**
```bash
cd ~/workspace/air_defense_evasion_drone_project/PX4-Autopilot
HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris
```
**역할:** 드론 물리 시뮬레이션 및 비행 컨트롤러  
**옵션:**
- `HEADLESS=1`: Gazebo GUI 숨김 (빠른 학습)
- `PX4_SIM_SPEED_FACTOR=3`: 시뮬레이션 속도 3배 (학습 가속)

**GUI 보려면:**
```bash
make px4_sitl gazebo-classic_iris  # HEADLESS 제거
```

---

### **터미널 3: Turret Simulator (대공포)**
```bash
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws
source /opt/ros/humble/setup.bash  # 또는 setup.zsh
source install/setup.bash
ros2 run flight_control turret_sim_new
```
**역할:** 투사체 발사 및 위협 시뮬레이션

---

### **터미널 4: 학습 시작 (Training)**
```bash
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control
source /opt/ros/humble/setup.bash
source ~/workspace/air_defense_evasion_drone_project/ros2_ws/install/setup.bash
python3 train.py
```
**역할:** PPO 알고리즘으로 AI 학습

**예상 출력:**
```
Training Start! Logs: ./logs/20260213-..., Models: ./models/20260213-...
>>> Resetting Environment...
>>> Initial State - Nav: 0, Arm: 1
>>> Phase 1: Sending Offboard heartbeat for 2 seconds...
>>> Phase 2: Switching to Offboard mode...
>>> Offboard mode activated!
>>> Phase 3: Arming...
>>> Armed successfully!
>>> Auto-Takeoff to 2.0m...
>>> Ready to Fly! Handing over control to AI.
```

---

### **터미널 5 (선택): TensorBoard (학습 모니터링)**
```bash
cd ~/workspace/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control
tensorboard --logdir=./logs
```
브라우저에서 `http://localhost:6006` 접속

**확인 사항:**
- `ep_rew_mean`: 평균 보상 (증가해야 함)
- `ep_len_mean`: 에피소드 길이 (생존 시간)

---

### **🛑 학습 중지**

각 터미널에서 `Ctrl+C` 입력

**학습된 모델 위치:**
```
./models/YYYYMMDD-HHMMSS/
├── drone_ppo_100000_steps.zip
├── drone_ppo_200000_steps.zip
└── drone_ppo_final.zip
```

---

### **⚠️ 문제 해결**

#### **문제: "Nav: 0, Arm: 0"에서 멈춤**
```bash
# 모든 프로세스 종료
killall -9 px4 gzserver gzclient MicroXRCEAgent

# 순서대로 재시작 (터미널 1 → 2 → 3 → 4)
```

#### **문제: "vehicle_status 토픽 없음"**
- PX4 최신 버전은 `/fmu/out/vehicle_status_v1` 사용
- `drone_env.py` Line 37에서 확인

#### **문제: "총알 데이터 없음"**
```bash
# 총알 토픽 확인
ros2 topic echo /turret/bullets --once
```

---

## 📦 설치 (Installation)

자세한 설치 가이드는 [INSTALL.md](INSTALL.md)를 참고하세요.

**빠른 시작:**
```bash
git clone https://github.com/YOUR_USERNAME/air_defense_evasion_drone_project.git
cd air_defense_evasion_drone_project
./setup_environment.sh
```

---

## 📂 프로젝트 구조 (Project Structure)

```
air_defense_evasion_drone_project/
├── README.md                      # 프로젝트 개요
├── INSTALL.md                     # 설치 가이드
├── .gitignore                     # Git 제외 파일 목록
├── requirements.txt               # Python 의존성
├── dependencies.repos             # ROS2 외부 패키지
├── setup_environment.sh           # 자동 환경 설정 스크립트
│
├── ros2_ws/                       # ROS2 워크스페이스
│   └── src/
│       └── flight_control/        # 비행 제어 패키지
│           └── offboard_test.py   # Offboard 제어 테스트
│
├── models/                        # Gazebo 모델 (향후)
├── gazebo_worlds/                 # 시뮬레이션 월드 (향후)
└── rl_training/                   # 강화학습 코드 (향후)
```

**제외된 대형 의존성 (로컬 전용):**
- `Micro-XRCE-DDS-Agent/` - PX4-ROS2 브릿지
- `PX4-Autopilot/` - PX4 SITL
- `ros2_ws/src/px4_msgs/` - dependencies.repos로 관리

---

## 1. 개요 (Overview)
본 프로젝트는 **연발 방공망(Anti-Aircraft Artillery)의 위협 하에서 생존하며 목표를 타격하는 자율 비행 드론**을 개발하는 것을 목표로 합니다.

기존의 단순 장애물 회피와 달리, 적의 **예측 사격(Lead Shot)**을 역이용하는 고기동 회피 알고리즘을 **강화학습(PPO)**을 통해 학습시킵니다. 드론은 개별 투사체를 추적하는 비현실적인 가정 대신, **적의 총구 방향(Aim Vector)**을 인식하여 회피 기동을 수행합니다.

### 🎯 핵심 목표
* **생존(Survival):** 고속으로 연사되는 대공포 화망 회피.
* **임무(Mission):** 최단 시간 내 목표 지점(Target) 도달.
* **현실성(Realism):** 온보드 비전 센서로 획득 가능한 정보(적의 조준 방향)만을 관측값으로 사용.

---

## 2. 개발 환경 (Development Environment)
안정성과 레퍼런스 확보를 위해 표준화된 ROS2-PX4 스택을 사용합니다.

| Component | Version / Detail |
| :--- | :--- |
| **OS** | Ubuntu 22.04 LTS (WSL2 supported) |
| **Middleware** | ROS2 Humble Hawksbill |
| **Simulator** | Gazebo Classic 11 |
| **Flight Controller** | PX4-Autopilot (SITL) |
| **Drone Model** | 3DR Iris Quadcopter |
| **RL Framework** | Gymnasium (OpenAI Gym), Stable-baselines3 (PPO) |
| **Language** | Python 3.10, C++ |

---

## 3. 시나리오 및 위협 정의 (Scenario & Threat)

### 3.1. 환경 (World)
* **맵:** `Empty World` (장애물이 없는 평지).
* **배치:**
    * **Drone:** $(0, 0, 0)$ 이륙.
    * **Turret:** $(d, 0, 0)$ 전방 배치 (거리 가변).
    * **Target:** 터렛 후방 혹은 인근의 특정 좌표.

### 3.2. 적 위협: 자동 추적 대공포 (Auto-Tracking Turret)
드론을 격추하기 위해 설계된 가상의 대공 기관포입니다.

#### **투사체 물리 시스템 (Projectile Physics)**
기존의 히트스캔(즉시 명중) 방식 대신, **실제 총알이 물리적으로 날아가는 시스템**을 구현합니다.

* **조준 로직 (Perfect Lead Shot):**
    * 드론의 현재 위치와 속도를 기반으로 미래 위치를 완벽하게 예측
    * 터렛의 회전 속도는 무한대로 가정 (즉시 조준 가능)
    * 하지만 총알이 날아가는 동안 드론이 기동하면 빗나감

* **총알 물리:**
    * 각 총알은 3D 공간에서 독립적으로 비행
    * 위치 업데이트: `P_new = P_old + V * dt`
    * 중력 무시 (단순화)
    * 지면 충돌 시 소멸 (`z >= 0` in NED)

* **무기 제원:**
    * **발사 속도:** 120 RPM (0.5초당 1발) → 600 RPM (최종 목표)
    * **탄속:** 초기 학습 시 $50m/s$ → 중간 $200m/s$ → 최종 $850m/s$ (Curriculum Learning)
    * **사거리:** 100m
    * **피격 판정:** 총알과 드론 간 거리 < 0.5m

#### **회피 전략 (Evasion Tactics)**
드론은 다음과 같은 전략을 학습해야 합니다:
1. **예측 불가능한 움직임:** 일정 속도 직선 비행 시 100% 명중 → 불규칙한 가감속과 방향 전환 필요
2. **탄도 인식:** 날아오는 총알의 위치와 속도를 관측하여 위험도 판단
3. **타이밍 기동:** 총알이 가까워지는 순간 급기동(Jinking)으로 회피
4. **화망 돌파:** 여러 총알이 동시에 날아올 때 틈새를 찾아 통과

---

## 4. 강화학습 설계 (RL Design)

### 4.1. 관측 공간 (Observation Space)
드론은 센서를 통해 **날아오는 총알의 실시간 위치와 속도**를 인식합니다.

1.  **Target Info (목표 정보):**
    * 목표 지점까지의 상대 위치 벡터 $(x, y, z)$ - 3D

2.  **Self State (자신 정보):**
    * 현재 선속도(Linear Velocity) 벡터 $(v_x, v_y, v_z)$ - 3D
    * (쿼드콥터의 물리적 한계 내에 있는 값)

3.  **Bullet Info (총알 정보) - 가변 길이:**
    * 각 총알마다 6개 데이터: `[pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]`
    * 최대 N개의 총알 추적 (N=10~20)
    * 총알이 없으면 0으로 패딩
    
    > **💡 전략 포인트:** 
    > - 총알의 **현재 위치**와 **속도 벡터**를 이용해 미래 궤적 예측
    > - 드론과의 **최근접 거리(CPA)** 계산
    > - 위험한 총알(가까운 것)과 안전한 총알(먼 것) 구분
    > - 선분 교차 판정으로 충돌 시간 예측

**관측 공간 차원:** `3 + 3 + (N * 6)` where N = 최대 총알 개수

### 4.2. 행동 공간 (Action Space)
PPO 알고리즘 적용을 위해 **연속적인(Continuous) 값**을 사용합니다.
* **Type:** `Box(-1.0, 1.0, shape=(3,))`
* **Actions:**
    1.  `Target Velocity X` (전후)
    2.  `Target Velocity Y` (좌우)
    3.  `Target Velocity Z` (상하)
* **Control Mode:** PX4 Offboard Velocity Control 모드 사용. (자세 제어는 필요 시 추후 도입)

### 4.3. 보상 함수 (Reward Function)
목표 달성 여부와 효율성을 기반으로 보상을 부여합니다.

$$R_{total} = R_{terminal} + R_{efficiency} + R_{safety}$$

1.  **Terminal Reward (종료 보상):**
    * **성공(Success):** 목표 반경 $2m$ 이내 도달 시 `+100`
    * **실패(Fail):**
        * **투사체 피격:** 총알과의 거리 < 0.5m → `-100` (즉시 종료)
        * **지면 충돌:** z ≥ 0 (NED) → `-100` (즉시 종료)
        * **구역 이탈:** 맵 경계 벗어남 → `-100` (즉시 종료)

2.  **Efficiency Reward (효율성 보상):**
    * **시간 페널티:** 매 스텝마다 `-0.1` (최단 시간 유도)
    * **접근 보상:** 목표와의 거리가 줄어들면 `+거리_감소 * 0.5`

3.  **Safety Reward (안전 보상) - 새로 추가:**
    * **근접 회피:** 총알이 2m 이내로 근접했다가 멀어지면 `+5` (위험 회피 성공)
    * **무모한 기동 페널티:** 불필요하게 빠른 속도 변화 시 `-에너지_소모`
    * **생존 보너스:** 매 스텝 생존 시 `+0.1` (장기 생존 장려)

#### **피격 판정 로직 (Hit Detection)**
```
for each bullet:
    distance = ||bullet_pos - drone_pos||
    if distance < 0.5m:
        Hit! → Episode End, Reward = -100
    elif distance < 2.0m:
        Near Miss → Reward = +5 (once per bullet)
```

---

## 5. 드론 물리 모델 (Drone Dynamics)
* **모델:** PX4 Standard `iris`.
* **제약 사항:** 인위적인 제한(배터리, 속도 리미트)은 두지 않으며, 시뮬레이터 상의 `iris` 기체가 낼 수 있는 물리적 최대 속도와 가속도를 100% 활용함.

---

## 6. 개발 로드맵 (Roadmap)
1.  **Phase 1: Environment Setup** ✅ 완료
    * ROS2-Gazebo-PX4 연동 환경 구축
    * Offboard 모드로 드론 속도 제어 API 테스트
    * vehicle_status_v1 토픽 연동

2.  **Phase 2: Threat Implementation** ✅ 완료
    * ROS2 노드로 투사체 시뮬레이터(`turret_sim_new.py`) 구현
    * Lead Shot 알고리즘으로 완벽한 예측 사격
    * 총알 물리 시뮬레이션 (위치 업데이트, 지면 충돌 판정)
    * 총알 데이터 ROS2 토픽 발행 (`/turret/bullets`)

3.  **Phase 3: RL Integration** 🔄 진행 중
    * Gymnasium Custom Env 작성 (`reset()`, `step()` 구현)
    * 총알 데이터 구독 및 관측 공간 통합
    * 피격 판정 로직 추가
    * PPO 알고리즘 연결 (Stable-baselines3)

4.  **Phase 4: Training & Validation** ⏳ 대기
    * **Curriculum Learning:**
        * Level 1: 탄속 50m/s, 발사 속도 120 RPM
        * Level 2: 탄속 200m/s, 발사 속도 300 RPM
        * Level 3: 탄속 850m/s, 발사 속도 600 RPM
    * 랜덤 터렛 위치, 학습된 모델 평가
    * 화망 돌파율, 생존 시간, 목표 도달률 분석