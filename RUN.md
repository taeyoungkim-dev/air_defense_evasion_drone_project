# Run Guide

Step-by-step instructions to launch simulation and train the evasion agent.

For project overview and curriculum demos, see **[README.md](README.md)**.  
For first-time setup, see **[INSTALL.md](INSTALL.md)**.

---

## Pre-flight Checklist

```bash
# ROS 2 Humble
ros2 --version

# Python dependencies
python3 -c "import torch, gymnasium, stable_baselines3; print('OK')"

# PX4-Autopilot (local install, not in repo)
ls ~/PX4-Autopilot

# ROS 2 workspace built
source ~/air_defense_evasion_drone_project/ros2_ws/install/setup.bash
ros2 pkg list | grep flight_control
```

---

## Launch Order (5 Terminals)

### Terminal 1 — Micro-XRCE-DDS-Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

**Role:** Bridges PX4 uORB ↔ ROS 2 DDS.

**Expected:**
```
info | UDPv4AgentLinux.cpp | init | running... | port: 8888
```

---

### Terminal 2 — PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot

# Fast training (recommended): headless, 3× sim speed
HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris

# With GUI (slower)
make px4_sitl gazebo-classic_iris
```

**Expected:** `pxh>` prompt and `uxrce_dds_client synchronized` in the log.

| Variable | Meaning |
| :--- | :--- |
| `HEADLESS=1` | Hide Gazebo GUI |
| `PX4_SIM_SPEED_FACTOR=3` | 3× simulation speed |

---

### Terminal 3 — Turret Simulator

```bash
cd ~/air_defense_evasion_drone_project/ros2_ws
source /opt/ros/humble/setup.bash   # or setup.zsh
source install/setup.bash
ros2 run flight_control turret_sim_new
```

**Verify bullets topic:**
```bash
ros2 topic echo /turret/bullets --once
```

**Note:** The turret waits for `/turret/enable` (published by `only_dodge_env` after reset). For `train.py` / `drone_env_new`, firing may start immediately depending on turret version.

---

### Terminal 4 — Training

**Mission + evasion (`drone_env_new`):**
```bash
cd ~/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control
source /opt/ros/humble/setup.bash
source ~/air_defense_evasion_drone_project/ros2_ws/install/setup.bash
python3 train.py
```

**Lazy Survivor only (`only_dodge_env`):**
```bash
python3 train_only_dodge.py
```

**Expected (healthy start):**
```
>>> Resetting Environment...
>>> Initial State - Nav: 0, Arm: 1
>>> Phase 1: Sending Offboard heartbeat for 2 seconds...
>>> Offboard mode activated!
>>> Armed successfully!
>>> Ready to Fly!
```

Key metrics:
- `ep_len_mean` — survival steps (should increase)
- `ep_rew_mean` — average reward (should increase)

---

### Terminal 5 (Optional) — TensorBoard

```bash
cd ~/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control
tensorboard --logdir=./logs_only_dodge   # or ./logs for train.py
```

Open `http://localhost:6006`

---

## Rule-Based Baseline Test

Test dodge logic without RL:

```bash
cd ~/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control
source /opt/ros/humble/setup.bash
source ~/air_defense_evasion_drone_project/ros2_ws/install/setup.bash
python3 rule_based_test.py
```

---

## Visualization (RViz2)

```bash
ros2 run flight_control visualize_bullets
# In RViz2: add MarkerArray on /visualization_markers, Fixed Frame: map
```

---

## Stop Safely

1. `Ctrl+C` on training terminal (wait for checkpoint save)
2. Turret → PX4 → Agent

**Force kill if stuck:**
```bash
killall -9 python3 px4 gzserver gzclient MicroXRCEAgent
```

---

## Output Locations

```
./logs/YYYYMMDD-HHMMSS/              # train.py TensorBoard logs
./models/YYYYMMDD-HHMMSS/            # train.py checkpoints

./logs_only_dodge/YYYYMMDD-HHMMSS/   # train_only_dodge logs
./models_only_dodge/YYYYMMDD-HHMMSS/   # train_only_dodge checkpoints
```

---

## Troubleshooting

| Symptom | Fix |
| :--- | :--- |
| Stuck at `Nav: 0, Arm: 0` | Restart T1→T2→T3→T4; confirm Agent + PX4 sync |
| Arming fails repeatedly | Kill all processes, cold restart |
| No `/turret/bullets` | Rebuild workspace: `colcon build --symlink-install` |
| `ModuleNotFoundError: flight_control` | `source install/setup.bash` after build |
| Old code still running | `rm -rf __pycache__` and restart Python |

---

## Quick Reference (copy-paste)

```bash
# T1
MicroXRCEAgent udp4 -p 8888

# T2
cd ~/PX4-Autopilot && HEADLESS=1 PX4_SIM_SPEED_FACTOR=3 make px4_sitl gazebo-classic_iris

# T3
cd ~/air_defense_evasion_drone_project/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run flight_control turret_sim_new

# T4
cd ~/air_defense_evasion_drone_project/ros2_ws/src/flight_control/flight_control && source /opt/ros/humble/setup.bash && source ~/air_defense_evasion_drone_project/ros2_ws/install/setup.bash && python3 train_only_dodge.py

# T5 (optional)
tensorboard --logdir=./logs_only_dodge
```
