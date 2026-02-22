import time
import numpy as np
from stable_baselines3 import PPO
from only_dodge_env import OnlyDodgeEnv

# 1. 환경 생성
env = OnlyDodgeEnv()

# 2. 저장된 모델 불러오기 (경로 수정하세요!)
# 방금 학습 끝난 모델 경로를 넣으세요
model_path = "./models_only_dodge/20260221-025944/lazy_survivor_ppo_final.zip" 
model = PPO.load(model_path)

print(">>> 🎮 Enjoy Mode Started! Press Ctrl+C to stop.")

obs, _ = env.reset()
done = False

while True:
    # 결정적 행동(deterministic=True)을 하면 std를 무시하고 가장 확률 높은 행동을 함
    # 하지만 지금 모델은 std가 높아서 True로 하면 오히려 잘 못 피할 수도 있음.
    # 일단 False(랜덤성 포함)로 먼저 보세요.
    action, _ = model.predict(obs, deterministic=True) 
    
    obs, reward, done, truncated, info = env.step(action)
    
    if done or truncated:
        print(">>> ⚰️ Dead or Finished! Resetting...")
        obs, _ = env.reset()
        time.sleep(1.0)