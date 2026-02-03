from flight_control.drone_env import DroneEnv
import time
import numpy as np

def main():
    # 환경 생성
    env = DroneEnv()
    print("Resetting environment...")
    obs, _ = env.reset()

    print("=== Start Hover Test (Flying UP) ===")
    
    # 100 스텝 (약 10초) 동안 실행
    for i in range(1000):
        # [vx, vy, vz]
        # vz가 -1.0이면 위로 최대 속도로 상승하라는 뜻 (NED 좌표계: 위쪽이 -)
        # 랜덤(sample) 대신 강제로 상승 명령을 줍니다.
        action = np.array([0.0, 0.0, -0.5], dtype=np.float32)
        
        # 환경 한 스텝 진행
        obs, reward, done, _, _ = env.step(action)
        
        # 로그 출력 (드론 높이 확인)
        # obs 구조: [Target_Rel(3), Self_Vel(3), Threat(6)]
        # Target_Rel_Z = Target_Z - Current_Z
        # 따라서 Current_Z를 역산하거나, 그냥 잘 뜨는지 눈으로 확인하면 됩니다.
        print(f"Step: {i}, Action: UP, Reward: {reward:.2f}")
        
        if done:
            print("Episode Finished (Crashed or Target Reached)")
            env.reset()
            break

if __name__ == '__main__':
    main()