# 🎯 TODO: Projectile System 통합

`turret_sim_new.py`에 맞춰 학습 시스템을 업데이트해야 합니다.

---

## 📋 **필수 작업 (Critical)**

### ✅ **1. turret_sim_new.py 활성화**
- [ ] `turret_sim.py` → `turret_sim_new.py`로 교체
- [ ] `setup.py` 수정:
  ```python
  entry_points={
      'console_scripts': [
          'offboard_test = flight_control.offboard_test:main',
          'turret_sim = flight_control.turret_sim_new:main'  # ← 변경
      ],
  }
  ```
- [ ] 빌드: `cd ros2_ws && colcon build --symlink-install`

---

### 🔴 **2. drone_env.py - 총알 데이터 구독**

#### **2.1. 총알 데이터 구독 추가**
```python
# Line ~36 (Pub/Sub 설정 부분에 추가)
self.sub_bullets = self.node.create_subscription(
    Float32MultiArray, 
    '/turret/bullets',  # turret_sim_new.py가 발행하는 토픽
    self.bullets_cb, 
    10  # QoS
)
```

#### **2.2. 총알 콜백 함수 추가**
```python
# Line ~63 (Callbacks 섹션에 추가)
def bullets_cb(self, msg):
    """
    총알 데이터 형식: [x,y,z, vx,vy,vz, x2,y2,z2, vx2,vy2,vz2, ...]
    6개씩 묶어서 파싱
    """
    self.bullets_data = []
    data = msg.data
    
    # 6개씩 묶어서 총알 정보 추출
    for i in range(0, len(data), 6):
        if i + 5 < len(data):
            bullet = {
                'pos': np.array([data[i], data[i+1], data[i+2]]),
                'vel': np.array([data[i+3], data[i+4], data[i+5]])
            }
            self.bullets_data.append(bullet)
```

#### **2.3. 변수 초기화**
```python
# Line ~44 (변수 초기화 부분)
self.bullets_data = []  # 총알 리스트
self.max_bullets = 20   # 관측 공간에 포함할 최대 총알 개수
```

---

### 🟡 **3. drone_env.py - 관측 공간 변경**

#### **3.1. observation_space 재정의**
```python
# Line ~52 (Gym Space 정의 부분)
# 기존
# self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)

# 변경
# Observation: [Target_Rel(3), Self_Vel(3), Bullets(max_bullets * 6)]
obs_dim = 3 + 3 + (self.max_bullets * 6)  # 3 + 3 + 120 = 126
self.observation_space = spaces.Box(
    low=-np.inf, 
    high=np.inf, 
    shape=(obs_dim,), 
    dtype=np.float32
)
```

#### **3.2. _get_obs() 함수 수정**
```python
# Line ~173 (기존 _get_obs 교체)
def _get_obs(self):
    # 1. Target Relative Position (3)
    target_rel = self.target_pos - self.current_pos
    
    # 2. Self Velocity (3)
    self_vel = self.current_vel
    
    # 3. Bullets Data (max_bullets * 6)
    bullets_flat = []
    for i in range(self.max_bullets):
        if i < len(self.bullets_data):
            b = self.bullets_data[i]
            bullets_flat.extend([
                float(b['pos'][0]), float(b['pos'][1]), float(b['pos'][2]),
                float(b['vel'][0]), float(b['vel'][1]), float(b['vel'][2])
            ])
        else:
            # 총알이 없으면 0으로 패딩
            bullets_flat.extend([0.0] * 6)
    
    # 전체 관측 벡터 조합
    obs = np.concatenate([
        target_rel,           # 3
        self_vel,             # 3
        np.array(bullets_flat)  # max_bullets * 6
    ])
    
    return obs.astype(np.float32)
```

---

### 🔴 **4. drone_env.py - 피격 판정 로직 추가**

#### **4.1. step() 함수에 피격 판정 추가**
```python
# Line ~139 (step 함수의 보상 계산 부분)
def step(self, action):
    # ... 기존 코드 ...
    
    # 4. 보상 계산
    reward = 0.0
    done = False
    truncated = False
    
    # -------------------------------------------------
    # [새로 추가] 총알 피격 판정
    # -------------------------------------------------
    hit = False
    near_miss_count = 0
    
    for bullet in self.bullets_data:
        dist = np.linalg.norm(bullet['pos'] - self.current_pos)
        
        # 피격 (0.5m 이내)
        if dist < 0.5:
            hit = True
            reward -= 100.0
            done = True
            print("💥 Hit by bullet! Episode terminated.")
            break
        
        # 근접 회피 (2.0m 이내)
        elif dist < 2.0:
            near_miss_count += 1
    
    # 근접 회피 보상
    if near_miss_count > 0 and not hit:
        reward += 5.0 * near_miss_count
    
    # 생존 보너스
    if not hit:
        reward += 0.1
    
    # -------------------------------------------------
    # [기존] 목표 도달 판정
    # -------------------------------------------------
    if not hit:  # 피격되지 않았을 때만 체크
        dist_to_target = np.linalg.norm(self.target_pos - self.current_pos)
        reward -= dist_to_target * 0.01  # 거리 페널티
        
        if dist_to_target < 2.0:  # 목표 도달
            reward += 100.0
            done = True
            print("🎯 Target Reached!")
        
        # 추락 판정
        if self.current_pos[2] > 0.5:
            reward -= 100.0
            done = True
            print("💥 Crashed!")
    
    return obs, reward, done, truncated, {}
```

---

### 🟢 **5. train.py - 하이퍼파라미터 조정 (선택)**

총알 데이터가 추가되어 관측 공간이 커졌으므로 네트워크 크기 조정 고려:

```python
# Line ~24
model = PPO(
    "MlpPolicy", 
    env, 
    verbose=1,
    tensorboard_log=log_dir,
    learning_rate=0.0003,
    n_steps=2048,
    batch_size=64,
    ent_coef=0.01,
    device="cpu",
    # 추가 (선택)
    policy_kwargs=dict(
        net_arch=[256, 256, 128]  # 네트워크 크기 증가
    )
)
```

---

## 🧪 **테스트 절차**

### **Step 1: turret_sim_new 단독 테스트**
```bash
# Terminal 1: PX4
make px4_sitl gazebo-classic_iris

# Terminal 2: DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Turret (수정 후)
ros2 run flight_control turret_sim

# Terminal 4: 총알 데이터 확인
ros2 topic echo /turret/bullets
```

**예상 출력:**
```
data: [20.0, 5.0, 0.0, 15.2, -3.1, -2.5, ...]
```

---

### **Step 2: drone_env 통합 테스트**
```bash
# test_env.py 실행
python3 test_env.py
```

**확인 사항:**
- ✅ 총알 데이터가 관측에 포함되는가?
- ✅ 피격 판정이 작동하는가?
- ✅ Episode가 정상 종료되는가?

---

### **Step 3: 학습 시작**
```bash
python3 train.py
```

**모니터링:**
- `ep_rew_mean`: 평균 보상 (증가해야 함)
- `ep_len_mean`: 에피소드 길이 (생존 시간)
- Hit rate: 피격당한 비율

---

## 📊 **성공 기준**

- [ ] 총알 데이터가 ROS2 토픽으로 정상 발행됨
- [ ] drone_env가 총알 데이터를 관측 공간에 포함
- [ ] 피격 판정이 정확하게 작동 (0.5m 이내)
- [ ] AI가 학습 시작 (ep_rew_mean 증가)
- [ ] 드론이 총알을 회피하는 행동 학습

---

## 🎯 **최종 목표**

**AI가 다음을 학습:**
1. 총알의 위치와 속도를 보고 위험도 판단
2. 위험한 총알이 가까워질 때 급기동(Jinking)
3. 불필요한 기동 최소화 (에너지 효율)
4. 화망 틈새를 찾아 목표 도달

---

## ⏰ **예상 작업 시간**

- Setup.py 수정 및 빌드: **5분**
- drone_env.py 수정: **30분**
- 테스트 및 디버깅: **20분**
- 학습 시작: **즉시**

**총 예상 시간: ~1시간**

---

## 📝 **참고 사항**

### **총알 개수 제한 (max_bullets)**
- 현재: 20개
- 너무 많으면 관측 공간이 너무 커짐
- 너무 적으면 모든 총알을 추적하지 못함
- 권장: 10~20개

### **피격 판정 거리**
- Hit: 0.5m (드론 반경 고려)
- Near Miss: 2.0m (위험 감지 범위)
- 필요시 조정 가능

### **좌표계 주의**
- PX4: NED (North-East-Down)
- z < 0: 공중
- z = 0: 지면
- z > 0: 지하

---

**모든 작업을 완료하면 투사체 회피 AI 학습 시작!** 🚀
