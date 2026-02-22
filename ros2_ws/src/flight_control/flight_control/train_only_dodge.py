import os
import time
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from only_dodge_env import OnlyDodgeEnv #Changeable

def main():
    """
    Lazy Survivor 학습 스크립트
    
    목표: 홈 포지션 근처에서 최소한의 움직임으로 총알을 회피하며 생존
    """
    # 1. 로그 및 모델 저장 경로 설정
    run_id = time.strftime("%Y%m%d-%H%M%S")
    log_dir = f"./logs_only_dodge/{run_id}"
    model_dir = f"./models_only_dodge/{run_id}"
    
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)

    print("="*60)
    print("🎯 Lazy Survivor Training Start!")
    print("="*60)
    print(f"📊 Logs Directory:   {log_dir}")
    print(f"💾 Models Directory: {model_dir}")
    print(f"🎮 Environment:      OnlyDodgeEnv (Home Defense)")
    print("="*60)
    print()

    # 2. 환경 생성
    env = OnlyDodgeEnv()#Changeable

    # 3. 모델 생성 (PPO 알고리즘)
    # Lazy Survivor에 최적화된 하이퍼파라미터
    model = PPO.load(
        "./models_only_dodge/20260219-224246/lazy_survivor_ppo_final.zip",
        env=env,
        tensorboard_log=log_dir
    )
    # model = PPO(
    #     "MlpPolicy", 
    #     env, 
    #     verbose=1,
    #     tensorboard_log=log_dir,
    #     learning_rate=0.0003,      # 학습률
    #     n_steps=2048,              # 업데이트 전 데이터 수집량
    #     batch_size=64,             # 미니배치 크기
    #     ent_coef=0.01,             # 탐험 장려 (낮음 - 보수적 행동 선호)
    #     gamma=0.99,                # 할인 계수
    #     gae_lambda=0.95,           # GAE lambda
    #     clip_range=0.2,            # PPO 클리핑 범위
    #     device="cpu"               # CPU 사용
    # )

    # 4. 중간 저장 콜백 (10만 스텝마다 모델 저장)
    checkpoint_callback = CheckpointCallback(
        save_freq=100000, 
        save_path=model_dir,
        name_prefix="lazy_survivor_ppo"
    )

    # 5. 학습 시작!
    print("Starting Training...")
    print("Metrics to Watch:")
    print("   - ep_len_mean: 생존 스텝 수 (높을수록 좋음)")
    print("   - ep_rew_mean: 평균 보상 (높을수록 좋음)")
    print("   - 목표: 최소 움직임으로 최대 생존")
    print()
    
    model.learn(total_timesteps=500_000, callback=checkpoint_callback, reset_num_timesteps=False)

    # 6. 최종 모델 저장
    final_model_path = f"{model_dir}/lazy_survivor_ppo_final"
    model.save(final_model_path)
    
    print()
    print("="*60)
    print("✅ Training Finished!")
    print("="*60)
    print(f"💾 Final Model Saved: {final_model_path}.zip")
    print(f"📊 TensorBoard: tensorboard --logdir={log_dir}")
    print("="*60)

if __name__ == '__main__':
    main()
