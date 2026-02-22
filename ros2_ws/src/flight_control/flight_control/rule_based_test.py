#!/usr/bin/env python3
"""
Simple Rule-Based Dodge Test

단순 테스트:
1. 드론이 (0, 0, -10)에 위치
2. 총알 발사되면 최대 가속도로 회피
3. 단 한 번의 총알 피격/회피 결과 출력
4. 결과 출력 후 대기 (sleep)

사용법:
    python3 rule_based_test.py
"""

import numpy as np
import time
from only_dodge_env import OnlyDodgeEnv

class SimpleRuleBasedController:
    def __init__(self):
        # 회피 방향 (1.0 = 오른쪽, -1.0 = 왼쪽)
        self.dodge_direction = 1.0
        
        # 총알을 한 번이라도 발견했는지 플래그
        self.bullet_ever_detected = False
        
    def get_action(self, obs):
        """
        obs 구조:
        [0:3]  - Home_Rel (홈으로부터의 상대 위치)
        [3:6]  - Self_Vel (드론 속도)
        [6:9]  - Bullet_Rel (총알 상대 위치)
        [9:12] - Bullet_Vel (총알 속도)
        """
        
        # 총알 정보 추출
        bullet_vel = obs[9:12]
        bullet_speed = np.linalg.norm(bullet_vel)
        
        # 실제 총알 감지 (속도 > 0)
        if bullet_speed > 0.1 and not self.bullet_ever_detected:
            self.bullet_ever_detected = True
            bullet_rel = obs[6:9]
            bullet_dist = np.linalg.norm(bullet_rel)
            
            print(f"🚨 총알 최초 감지!")
            print(f"   - 거리: {bullet_dist:.2f}m")
            print(f"   - 속도: {bullet_speed:.2f}m/s")
            print(f"   - 예상 도달 시간: {bullet_dist/bullet_speed:.2f}초")
            print(f"🏃 최대 가속도로 회피 시작! (방향: {'오른쪽' if self.dodge_direction > 0 else '왼쪽'})\n")
        
        # 총알을 한 번이라도 발견했으면 계속 회피!
        if self.bullet_ever_detected:
            return np.array([0.0, self.dodge_direction * 1.0, 0.0], dtype=np.float32)
        else:
            # 아직 총알 없음 → 정지
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)


def main():
    print("\n" + "="*60)
    print("🎯 Simple Rule-Based Dodge Test")
    print("="*60)
    print("\n설정:")
    print("  - 드론 위치: (0, 0, -10)")
    print("  - 총알 감지: 무한대 (항상 감지)")
    print("  - 회피 강도: 최대 (5.0 m/s)")
    print("  - 테스트: 단 한 번의 총알 회피")
    print("\n" + "="*60 + "\n")
    
    # 환경 초기화
    print(">>> 환경 초기화 중...")
    env = OnlyDodgeEnv()
    controller = SimpleRuleBasedController()
    
    try:
        print("\n>>> 드론 이륙 및 위치 잡기...")
        obs, info = env.reset()
        
        print(">>> ✅ 준비 완료! 총알 대기 중...\n")
        
        step_count = 0
        max_steps = 500  # 최대 50초
        
        result = None
        
        while step_count < max_steps:
            step_count += 1
            
            # Rule-based 액션 결정
            action = controller.get_action(obs)
            
            # Step 실행
            obs, reward, done, truncated, info = env.step(action)
            
            # 종료 조건 체크
            if done or truncated:
                print("="*60)
                
                home_dist = np.linalg.norm(obs[0:3])
                
                # 종료 원인 판별
                if home_dist > 1000.0:
                    result = f"❌ 바운더리 이탈 (홈 거리: {home_dist:.2f}m)"
                elif step_count >= max_steps or truncated:
                    result = f"✅ 총알 회피 성공!"
                else:
                    result = f"❌ 총알 피격!"
                
                print(result)
                print("="*60)
                
                if controller.bullet_ever_detected:
                    print(f"📊 테스트 결과:")
                    print(f"   - 생존 시간: {step_count * 0.1:.1f}초 ({step_count} 스텝)")
                    print(f"   - 최종 홈 거리: {home_dist:.2f}m")
                else:
                    print("⚠️ 총알이 발사되지 않았습니다.")
                
                print("="*60)
                
                break
        
        # 결과 없이 종료된 경우
        if result is None:
            print("="*60)
            print("⏱️ 타임아웃 - 총알이 발사되지 않았습니다.")
            print("="*60)
        
        print("\n>>> 테스트 완료. 대기 중... (Ctrl+C로 종료)")
        
        # 무한 대기
        while True:
            time.sleep(1.0)
    
    except KeyboardInterrupt:
        print("\n\n>>> Ctrl+C 감지. 종료합니다...")
    
    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
