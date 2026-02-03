# 🚁 Project: AA-Evasion (Anti-Air Evasion Drone)

> **Deep Reinforcement Learning based UAV Navigation in Hostile Environments**

## 📋 목차 (Table of Contents)
- [개요](#1-개요-overview)
- [설치 가이드](#설치-installation)
- [프로젝트 구조](#프로젝트-구조-project-structure)
- [개발 환경](#2-개발-환경-development-environment)
- [시나리오 및 위협 정의](#3-시나리오-및-위협-정의-scenario--threat)
- [강화학습 설계](#4-강화학습-설계-rl-design)
- [개발 로드맵](#6-개발-로드맵-roadmap)

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
* **사격 로직 (Lead Shot):** 드론의 현재 위치와 속도를 계산하여, 탄착 시간에 드론이 있을 것으로 **예상되는 지점**을 향해 사격.
* **무기 제원:**
    * **발사 속도:** 600 RPM (0.1초당 1발).
    * **탄속:** 초기 학습 시 $300m/s$ $\rightarrow$ 최종 $850m/s$ (Curriculum Learning).
    * **탄퍼짐:** 완벽한 명중률 대신 약간의 분산(Dispersion)을 적용하여 화망 형성.

---

## 4. 강화학습 설계 (RL Design)

### 4.1. 관측 공간 (Observation Space)
드론은 비전 센서를 통해 **적의 형상(총구 방향)**을 인식한다고 가정합니다. 날아오는 총알(투사체)의 개별 위치는 알 수 없습니다.

1.  **Target Info (목표 정보):**
    * 목표 지점까지의 상대 위치 벡터 $(x, y, z)$.
2.  **Self State (자신 정보):**
    * 현재 선속도(Linear Velocity) 벡터 $(v_x, v_y, v_z)$.
    * (쿼드콥터의 물리적 한계 내에 있는 값).
3.  **Threat Info (위협 정보):**
    * **Turret Relative Position:** 적 포탑의 상대 위치.
    * **Turret Aim Vector:** **현재 적의 총구가 향하고 있는 방향 단위 벡터.**
    
    > **💡 전략 포인트:** 에이전트는 `Aim Vector`가 자신의 미래 경로와 일치할 때(Lock-on), 속도나 방향을 급격히 바꾸는 기동을 학습해야 함.

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

$$R_{total} = R_{terminal} + R_{efficiency}$$

1.  **Terminal Reward (종료 보상):**
    * **성공(Success):** 목표 반경 $1m$ 이내 도달 시 `+100`.
    * **실패(Fail):**
        * 투사체 피격 시 `-100`.
        * 지면 충돌(Crash) 시 `-100`.
        * 구역 이탈(Out of bound) 시 `-100`.
2.  **Efficiency Reward (효율성 보상):**
    * **시간 페널티:** 매 스텝마다 미세한 마이너스 보상 (최단 시간 유도).
    * **접근 보상:** 목표물과의 거리가 줄어들면 비례하여 보상 지급 (Sparse reward 문제 해결).

---

## 5. 드론 물리 모델 (Drone Dynamics)
* **모델:** PX4 Standard `iris`.
* **제약 사항:** 인위적인 제한(배터리, 속도 리미트)은 두지 않으며, 시뮬레이터 상의 `iris` 기체가 낼 수 있는 물리적 최대 속도와 가속도를 100% 활용함.

---

## 6. 개발 로드맵 (Roadmap)
1.  **Phase 1: Environment Setup**
    * ROS2-Gazebo-PX4 연동 환경 구축.
    * Offboard 모드로 드론 속도 제어 API 테스트.
2.  **Phase 2: Threat Implementation**
    * Gazebo 플러그인 또는 ROS2 노드로 'Lead Shot Turret' 구현.
    * 드론 위치 토픽(`odom`)을 구독하여 조준하고 총알을 생성하는 로직 개발.
3.  **Phase 3: RL Integration**
    * Gymnasium Custom Env 작성 (`reset()`, `step()` 구현).
    * PPO 알고리즘 연결 (Stable-baselines3).
4.  **Phase 4: Training & Validation**
    * 초기: 느린 탄속, 고정된 터렛 위치에서 학습.
    * 심화: 빠른 탄속, 랜덤 터렛 위치, 학습된 모델 평가.