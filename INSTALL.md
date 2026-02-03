# 설치 가이드 (Installation Guide)

## 1. 시스템 요구사항 (System Requirements)

- **OS:** Ubuntu 22.04 LTS
- **ROS2:** Humble Hawksbill
- **Gazebo:** Classic 11
- **Python:** 3.10+
- **Disk Space:** ~10GB

## 2. 사전 준비 (Prerequisites)

### 2.1. ROS2 Humble 설치

```bash
# ROS2 설치 (아직 설치하지 않은 경우)
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2.2. Gazebo Classic 11 설치

```bash
sudo apt update
sudo apt install -y gazebo libgazebo-dev
```

### 2.3. Python 개발 도구 설치

```bash
sudo apt install -y python3-pip python3-venv
```

## 3. 프로젝트 설치 (Project Installation)

### 방법 1: 자동 설치 스크립트 사용 (권장)

```bash
# 프로젝트 클론
git clone https://github.com/YOUR_USERNAME/air_defense_evasion_drone_project.git
cd air_defense_evasion_drone_project

# 자동 설치 스크립트 실행
./setup_environment.sh
```

### 방법 2: 수동 설치

```bash
# 1. 프로젝트 클론
git clone https://github.com/YOUR_USERNAME/air_defense_evasion_drone_project.git
cd air_defense_evasion_drone_project

# 2. Python 의존성 설치
pip3 install -r requirements.txt

# 3. ROS2 패키지 의존성 설치
sudo apt install -y python3-vcstool
vcs import < dependencies.repos

# 4. rosdep 의존성 설치
cd ros2_ws
sudo rosdep init  # 이미 실행한 경우 에러 무시
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 5. ROS2 워크스페이스 빌드
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ..
```

## 4. 추가 도구 설치 (Optional)

### 4.1. PX4-Autopilot (SITL 시뮬레이션용)

```bash
cd ~/workspace/air_defense_evasion_drone_project

# PX4 클론
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# PX4 의존성 설치
bash ./Tools/setup/ubuntu.sh
```

### 4.2. Micro-XRCE-DDS-Agent (PX4-ROS2 브릿지)

```bash
cd ~/workspace/air_defense_evasion_drone_project

# Micro-XRCE-DDS-Agent 클론 및 빌드
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

## 5. 환경 설정 (Environment Configuration)

### .bashrc 또는 .zshrc에 추가

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash  # or setup.zsh

# Gazebo Classic 11
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

# 프로젝트 워크스페이스
if [ -f ~/workspace/air_defense_evasion_drone_project/ros2_ws/install/setup.bash ]; then
    source ~/workspace/air_defense_evasion_drone_project/ros2_ws/install/setup.bash
fi

# Gazebo 모델 경로
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/workspace/air_defense_evasion_drone_project/models
```

적용:
```bash
source ~/.bashrc  # 또는 source ~/.zshrc
```

## 6. 설치 확인 (Verify Installation)

```bash
# ROS2 확인
ros2 --version

# Gazebo 확인
gazebo --version

# 패키지 확인
ros2 pkg list | grep flight_control

# Python 패키지 확인
python3 -c "import torch; import gymnasium; print('All packages installed!')"
```

## 7. 문제 해결 (Troubleshooting)

### colcon build 실패
```bash
cd ros2_ws
rm -rf build install log
colcon build --symlink-install
```

### Python 패키지 설치 실패
```bash
pip3 install --upgrade pip
pip3 install -r requirements.txt --user
```

### rosdep 오류
```bash
sudo rosdep init
rosdep update
```

## 8. 다음 단계 (Next Steps)

설치가 완료되었다면 [README.md](README.md)를 참고하여 프로젝트를 실행하세요.
