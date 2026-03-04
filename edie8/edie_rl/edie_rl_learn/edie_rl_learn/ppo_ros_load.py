# DDPG main (tf2 subclassing API version)
# coded by St.Watermelon
## DDPG 에이전트를 학습하고 결과를 도시하는 파일

# 필요한 패키지 임포트
import rclpy
from .submodules.ppo_ros_learn import PPOagent
import tensorflow as tf

def main():
    rclpy.init()
    agent = PPOagent()  # DDPG 에이전트 객체
    agent.load()

if __name__=="__main__":
    main()