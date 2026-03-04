# DDPG main (tf2 subclassing API version)
# coded by St.Watermelon
## DDPG 에이전트를 학습하고 결과를 도시하는 파일

# 필요한 패키지 임포트
import rclpy
from .submodules.ppo_ros_learn import PPOagent
import tensorflow as tf

def main():
    rclpy.init()
    max_episode_num = 2000  # 최대 에피소드 설정
    max_batch_num = 50  # 최대 batch 크기 설정
    agent = PPOagent()  # DDPG 에이전트 객체
    # 학습 진행
    agent.train(max_episode_num, max_batch_num)
    # 학습 결과 도시
    agent.plot_result()
    #agent.load()

if __name__=="__main__":
    main()