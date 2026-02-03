import os
import time
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from flight_control.drone_env import DroneEnv

def main():
    # 1. 로그 및 모델 저장 경로 설정
    # (매번 실행할 때마다 models/1, models/2 ... 식으로 폴더가 생깁니다)
    run_id = time.strftime("%Y%m%d-%H%M%S")
    log_dir = f"./logs/{run_id}"
    model_dir = f"./models/{run_id}"
    
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)

    print(f"Training Start! Logs: {log_dir}, Models: {model_dir}")

    # 2. 환경 생성
    env = DroneEnv()

    # 3. 모델 생성 (PPO 알고리즘)
    # MlpPolicy: 입력이 이미지(Cnn)가 아니라 숫자 벡터(Mlp)이므로 사용
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1,
        tensorboard_log=log_dir,
        learning_rate=0.0003,
        n_steps=2048,      # 한 번 업데이트하기 위해 모으는 데이터 양
        batch_size=64,
        ent_coef=0.01,     # 탐험을 장려하는 계수 (높을수록 이것저것 시도해봄)
        device="cpu"      # CPU 사용
    )

    # 4. 중간 저장 콜백 (10만 스텝마다 모델 저장)
    checkpoint_callback = CheckpointCallback(
        save_freq=100000, 
        save_path=model_dir,
        name_prefix="drone_ppo"
    )

    # 5. 학습 시작! (총 100만 스텝)
    # 터미널에 ep_len_mean(버틴 시간), ep_rew_mean(받은 점수)가 뜹니다.
    model.learn(total_timesteps=1_000_000, callback=checkpoint_callback)

    # 6. 최종 모델 저장
    model.save(f"{model_dir}/drone_ppo_final")
    print("Training Finished!")

if __name__ == '__main__':
    main()