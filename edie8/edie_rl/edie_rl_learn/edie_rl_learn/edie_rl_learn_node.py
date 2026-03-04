# DDPG main (tf2 subclassing API version)
# coded by St.Watermelon
## DDPG 에이전트를 학습하고 결과를 도시하는 파일

# 필요한 패키지 임포트
import rclpy
from .submodules.edie_rl_learn import PPOagent
import tensorflow as tf
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

def main():
    rclpy.init()
    max_episode_num = 500  # 최대 에피소드 설정
    max_batch_num = 15  # 최대 batch 크기 설정
    agent = PPOagent()  # DDPG 에이전트 객체
    executor = MultiThreadedExecutor()
    executor.add_node(agent)
    agent.PrepareTrain(max_episode_num, max_batch_num)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("error")
    agent.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()