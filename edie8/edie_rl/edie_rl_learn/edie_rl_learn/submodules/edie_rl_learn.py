#ros2 service call /edie8/bt/rl_action edie_msgs/srv/BtRl {}\

# 필요한 패키지 임포트
import tensorflow as tf

from tensorflow.keras.models import Model
from tensorflow.keras.layers import Dense, Lambda
from tensorflow.keras.optimizers import Adam

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import *
from std_srvs.srv import *
from edie_msgs.srv import *
#################################################################################################
# <액터 신경망>
class Actor(Model):
    def __init__(self, action_dim, action_bound):
        super(Actor, self).__init__()
        self.action_bound = action_bound
        self.h1 = Dense(64, activation='relu')
        self.h2 = Dense(32, activation='relu')
        self.h3 = Dense(16, activation='relu')
        self.mu = Dense(action_dim, activation='tanh')
        self.std = Dense(action_dim, activation='softplus')
    def call(self, state):
        x = self.h1(state)
        x = self.h2(x)
        x = self.h3(x)
        mu = self.mu(x)
        std = self.std(x)
        # 평균값을 [-action_bound, action_bound] 범위로 조정
        mu = Lambda(lambda x: x*self.action_bound)(mu)
        return [mu, std]
#################################################################################################
# <크리틱 신경망>
class Critic(Model):
    def __init__(self):
        super(Critic, self).__init__()
        self.h1 = Dense(64, activation='relu')
        self.h2 = Dense(32, activation='relu')
        self.h3 = Dense(16, activation='relu')
        self.v = Dense(1, activation='linear')
    def call(self, state):
        x = self.h1(state)
        x = self.h2(x)
        x = self.h3(x)
        v = self.v(x)
        return v
#################################################################################################
class PPOagent(Node):
    def __init__(self):
        super().__init__('edie_rl_learn')
        self.group_bt = MutuallyExclusiveCallbackGroup()
        #self.group_bt_motion = MutuallyExclusiveCallbackGroup()
        self.group_timer = MutuallyExclusiveCallbackGroup()
        self.group_reset = MutuallyExclusiveCallbackGroup()
        self.group_state = MutuallyExclusiveCallbackGroup()
        self.group_action = MutuallyExclusiveCallbackGroup()
        self.group_reward = MutuallyExclusiveCallbackGroup()
        self.group_done = MutuallyExclusiveCallbackGroup()
        self.group_path = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.TimerCallback, callback_group=self.group_timer)
        self.Reset_Client = self.create_client(Empty, '/edie8/rl/reset_service', callback_group=self.group_reset)
        self.State_Client = self.create_client(Reinforcement, '/edie8/rl/state_service', callback_group=self.group_state)
        self.Action_Client = self.create_client(Reinforcement, '/edie8/rl/action_service', callback_group=self.group_action)
        self.Reward_Client = self.create_client(Reinforcement, '/edie8/rl/reward_service', callback_group=self.group_reward)
        self.Done_Client = self.create_client(Reinforcement, '/edie8/rl/done_service', callback_group=self.group_done)
        self.Path_Client = self.create_client(FilePath, '/edie8/rl/path_service', callback_group=self.group_path)
        self.Bt_Service = self.create_service(BtRl, '/edie8/bt/rl_action', self.BtServiceCallback, callback_group=self.group_bt)
        #self.BtMotion_Service = self.create_service(SetBool, '/edie8/bt/motion', self.BtMotionServiceCallback, callback_group=self.group_bt_motion)
        self.rl_on = False
        self.state_resp = np.ndarray([])
        self.reward_resp = np.ndarray([])
        while not self.Reset_Client.wait_for_service(timeout_sec=1.0):
            print('Reset Service not available, waiting again...')
        self.Reset_Client_req = Empty.Request()
        while not self.State_Client.wait_for_service(timeout_sec=1.0):
            print('State Service not available, waiting again...')
        self.State_Client_req = Reinforcement.Request()
        while not self.Action_Client.wait_for_service(timeout_sec=1.0):
            print('Action Service not available, waiting again...')
        self.Action_Client_req = Reinforcement.Request()
        while not self.Reward_Client.wait_for_service(timeout_sec=1.0):
            print('Reward Service not available, waiting again...')
        self.Reward_Client_req = Reinforcement.Request()
        while not self.Done_Client.wait_for_service(timeout_sec=1.0):
            print('Done Service not available, waiting again...')
        self.Done_Client_req = Reinforcement.Request()
        while not self.Path_Client.wait_for_service(timeout_sec=1.0):
            print('Path Service not available, waiting again...')
        self.Path_Client_req = FilePath.Request()
        for i in range(0,1):
            self.Action_Client_req.action.insert(i, 0)
        #---------------------------------------------------------------------------
        # 하이퍼파라미터
        self.GAMMA = 0.95
        self.GAE_LAMBDA = 0.9
        self.ACTOR_LEARNING_RATE = 0.0001
        self.CRITIC_LEARNING_RATE = 0.001
        self.RATIO_CLIPPING = 0.05
        self.EPOCHS = 5
        # 상태변수 차원 (머리 각도 제외)
        self.state_dim = 7
        # 행동 차원
        self.action_dim = 1
        # 행동의 최대 크기
        self.action_bound = np.float32(1.0)
        # 표준편차의 최솟값과 최댓값 설정
        self.std_bound = [1e-2, 1.0]
        # 액터 신경망 및 크리틱 신경망 생성
        self.actor = Actor(self.action_dim, self.action_bound)
        self.critic = Critic()
        self.actor.build(input_shape=(None, self.state_dim))
        self.critic.build(input_shape=(None, self.state_dim))
        self.actor.summary()
        self.critic.summary()
        # 옵티마이저
        self.actor_opt = Adam(self.ACTOR_LEARNING_RATE)
        self.critic_opt = Adam(self.CRITIC_LEARNING_RATE)
        # 에피소드에서 얻은 총 보상값을 저장하기 위한 변수
        self.save_epi_reward = []
        # 실시간 그래프 저장을 위한 변수
        self.png_name = 'ddpg_graph'
        self.png_number = 0
        self.png_number_string = ''
        self.file_path = 'src/'
        # 학습 진행을 위한 플래그
        self.flag = False
        self.falg2 = False
        self.bt_ready_flag = False   #BT로부터 모션 요청 들어오면 True로 변경 -> 해당 변수 True이면 1 episode 학습 진행
        self.learning_done = False
        #---------------------------------------------------------------------------
        #학습 episode 및 batch 크기 결정'
        self.max_episode = 500
        self.max_batch_size = 30
        #현재 episode
        self.ep = 0
    #/////////////////////////////////////////////////////////////////////////////
    def ResetClient(self):
        result = self.Reset_Client.call(self.Reset_Client_req)
        return result
    def StateClient(self):
        result = self.State_Client.call(self.State_Client_req)
        return result
    def ActionClient(self):
        result = self.Action_Client.call(self.Action_Client_req)
        return result
    def RewardClient(self):
        result = self.Reward_Client.call(self.Reward_Client_req)
        return result
    def DoneClient(self):
        result = self.Done_Client.call(self.Done_Client_req)
        return result
    def PathClient(self):
        result = self.Path_Client.call(self.Path_Client_req)
        return result
    #/////////////////////////////////////////////////////////////////////////////
    def TimerCallback(self):
        self.Train()
    #/////////////////////////////////////////////////////////////////////////////
    #def BtMotionServiceCallback(self, request, response):
    #    print("hi")
    #    response.message = "hi"
    #    return response
    def BtServiceCallback(self, request, response):
        self.file_path = self.PathClient().path
        print(self.file_path)
        if self.ep < self.max_episode:
            if not self.bt_ready_flag:
                # 에피소드 초기화
                self.time, self.episode_reward, self.done = 0, 0, False
                # 환경 초기화 및 초기 상태 관측
                self.ResetClient()
                while not self.StateClient().result:
                    #print("waiting for input state")
                    self.state = self.StateClient().state
                self.state = self.StateClient().state
                print("state: "+str(self.state))
                self.state = np.asarray(self.state)
                # 이전 정책의 평균, 표준편차를 계산하고 행동 샘플링
                self.mu_old, self.std_old, self.action = self.get_policy_action(tf.convert_to_tensor([self.state], dtype=tf.float32))
                print("---------------------------------------------")
                print("action (no clip): ")
                print(self.action)
                # 행동 범위 클리핑
                self.action = np.clip(self.action, -self.action_bound, self.action_bound)
                edie_action = self.GetAction(8,self.action+1)
                print("---------------------------------------------")
                print("edie action: ")
                print(edie_action)
                #for i in range(0, self.action_dim):
                    #self.ros.Action_Client_req.action[i] = action[i]
                self.Action_Client_req.action[0] = edie_action
                response.mbti = 1                  #MBTI 학습은 아직 구현 안됨
                response.emotion = edie_action+1   #BT로 보내는 emotion 데이작터는 1부터 값 시작
                self.bt_ready_flag = True
            else:
                print("Pls Wait unitil last episode learn is done")
        else:
            print("load")
            self.file_path = self.PathClient().path
            self.load_weights(self.file_path+'/../edie_rl_learn/edie_rl_learn/save_weights/ppo/')
            # 환경 초기화 및 초기 상태 관측
            self.ResetClient()
            while not self.StateClient().result:
                #print("waiting for input state")
                self.state = self.StateClient().state
            self.state = self.StateClient().state
            print(self.state)
            self.state = np.asarray(self.state)
            print("*************************************")
            print("state: "+str(self.state))
            self.mu_old, self.std_old, self.action = self.get_policy_action(tf.convert_to_tensor([self.state], dtype=tf.float32))
            print("---------------------------------------------")
            print("action (no clip): ")
            print(self.action)
            # 행동 범위 클리핑
            self.action = np.clip(self.action, -self.action_bound, self.action_bound)
            edie_action = self.GetAction(8,self.action+1)
            print("---------------------------------------------")
            print("edie action: ")
            print(edie_action)
            #for i in range(0, self.action_dim):
                #self.ros.Action_Client_req.action[i] = action[i]
            self.Action_Client_req.action[0] = edie_action
            response.mbti = 1                  #MBTI 학습은 아직 구현 안됨
            response.emotion = edie_action+1   #BT로 보내는 emotion 데이작터는 1부터 값 시작
            self.ActionClient()
            reward = self.RewardClient().reward
            while not self.StateClient().result:
                #print("waiting for reward state")
                next_state = self.StateClient().state
            next_state = self.StateClient().state
            next_state = np.asarray(next_state)
            #print("---------------------------------------------")
            #print("state: ")
            #print(state)
            #print("next_state: ")
            #print(next_state)
            reward = np.asarray(reward)
            reward = reward[0]
            #print("reward: ")
            #print(reward)
            self.done = self.DoneClient().done
        return response
    #/////////////////////////////////////////////////////////////////////////////
    def Train(self):
        if self.bt_ready_flag:
            print("learn")
            # 이전 정책의 로그 확률밀도함수 계산
            var_old = self.std_old ** 2
            log_old_policy_pdf = -0.5 * (self.action - self.mu_old) ** 2 / var_old - 0.5 * np.log(var_old * 2 * np.pi)
            log_old_policy_pdf = np.sum(log_old_policy_pdf)
            while not self.done:
                self.ActionClient()
                #print("state: "+str(state))
                # 다음 상태, 보상 관측
                # next_state = np.array(self.ros.StateClient().state)
                reward = self.RewardClient().reward
                while not self.StateClient().result:
                    #print("waiting for reward state")
                    next_state = self.StateClient().state
                next_state = self.StateClient().state
                next_state = np.asarray(next_state)
                #print("---------------------------------------------")
                #print("state: ")
                #print(state)
                #print("next_state: ")
                #print(next_state)
                reward = np.asarray(reward)
                reward = reward[0]
                #print("reward: ")
                #print(reward)
                self.done = self.DoneClient().done
                # shape 변환
                self.state = np.reshape(self.state, [1, self.state_dim])
                self.action = np.reshape(self.action, [1, self.action_dim])
                reward = np.reshape(reward, 1)
                log_old_policy_pdf = np.reshape(log_old_policy_pdf, [1, 1])
                # 학습용 보상 설정
                #train_reward = (reward) / self.state_dim
                train_reward = (reward)
                # 배치에 저장
                self.batch_state.append(self.state)
                self.batch_action.append(self.action)
                self.batch_reward.append(train_reward)
                self.batch_log_old_policy_pdf.append(log_old_policy_pdf)
                # 배치가 채워질 때까지 학습하지 않고 저장만 계속
                if len(self.batch_state) < self.max_batch_size:
                    # 상태 업데이트
                    self.state = next_state
                    self.episode_reward += reward[0]
                    print(str(self.time)+" done//////////////////////////////////////////////////////")
                    self.time += 1
                    continue
                # 배치가 채워지면, 학습 진행
                # 배치에서 데이터 추출
                states = self.unpack_batch(self.batch_state)
                actions = self.unpack_batch(self.batch_action)
                rewards = self.unpack_batch(self.batch_reward)
                log_old_policy_pdfs = self.unpack_batch(self.batch_log_old_policy_pdf)
                # 배치 비움
                self.batch_state, self.batch_action, self.batch_reward, = [], [], []
                self.batch_log_old_policy_pdf = []
                # GAE와 시간차 타깃 계산
                next_v_value = self.critic(tf.convert_to_tensor([next_state], dtype=tf.float32))
                v_values = self.critic(tf.convert_to_tensor(states, dtype=tf.float32))
                gaes, y_i = self.gae_target(rewards, v_values.numpy(), next_v_value.numpy(), self.done)
                # 에포크만큼 반복
                for _ in range(self.EPOCHS):
                    # 액터 신경망 업데이트
                    self.actor_learn(tf.convert_to_tensor(log_old_policy_pdfs, dtype=tf.float32),
                                     tf.convert_to_tensor(states, dtype=tf.float32),
                                     tf.convert_to_tensor(actions, dtype=tf.float32),
                                     tf.convert_to_tensor(gaes, dtype=tf.float32))
                    # 크리틱 신경망 업데이트
                    self.critic_learn(tf.convert_to_tensor(states, dtype=tf.float32),
                                      tf.convert_to_tensor(y_i, dtype=tf.float32))
                # 다음 스텝 준비
                print(str(self.time)+" done//////////////////////////////////////////////////////")
                self.state = next_state
                self.episode_reward += reward
                self.time += 1
                self.flag = True
            # 에피드마다 결과 보상값 출력
            self.ep += 1
            self.bt_ready_flag = False
            print('episode: ', self.ep, 'Time: ', self.time, 'Reward: ', self.episode_reward)
            self.save_epi_reward.append(self.episode_reward)
            # 에피소드마다 신경망 파라미터를 파일에 저장
            if self.ep % 5 == 0:
                self.actor.save_weights(self.file_path+"/../edie_rl_learn/edie_rl_learn/save_weights/ppo/edie_rl_actor.h5")
                self.critic.save_weights(self.file_path+"/../edie_rl_learn/edie_rl_learn/save_weights/ppo/edie_rl_critic.h5")
            if self.ep == self.max_episode:
                self.learning_done = True
                self.plot_result()
                np.savetxt(self.file_path+'/../edie_rl_learn/edie_rl_learn/save_weights/ppo/edie_rl_epi_reward.txt', self.save_epi_reward)
                print(self.save_epi_reward)
            else:
                np.savetxt(self.file_path+'/../edie_rl_learn/edie_rl_learn/save_weights/ppo/edie_rl_epi_reward.txt', self.save_epi_reward)
    #/////////////////////////////////////////////////////////////////////////////
    ## 로그-정책 확률밀도함수 계산
    def log_pdf(self, mu, std, action):
        std = tf.clip_by_value(std, self.std_bound[0], self.std_bound[1])
        var = std ** 2
        log_policy_pdf = -0.5 * (action - mu) ** 2 / var - 0.5 * tf.math.log(var * 2 * np.pi)
        return tf.reduce_sum(log_policy_pdf, 1, keepdims=True)
    ## 액터 신경망으로 정책의 평균, 표준편차를 계산하고 행동 샘플링
    def get_policy_action(self, state):
        mu_a, std_a = self.actor(state)
        mu_a = mu_a.numpy()[0]
        std_a = std_a.numpy()[0]
        std_a = np.clip(std_a, self.std_bound[0], self.std_bound[1])
        ppo_action = np.random.normal(mu_a, std_a, size=self.action_dim)
        #max_action = np.argmax(ppo_action)
        #action = np.zeros(self.action_dim)
        #action[max_action] = 1
        action = ppo_action
        print("mu_a: "+str(mu_a))
        print("std_a: "+str(std_a))
        print("ppo_action: "+str(ppo_action))
        return mu_a, std_a, action
    ## GAE와 시간차 타깃 계산
    def gae_target(self, rewards, v_values, next_v_value, done):
        n_step_targets = np.zeros_like(rewards)
        gae = np.zeros_like(rewards)
        gae_cumulative = 0
        forward_val = 0
        if not done:
            forward_val = next_v_value
        for k in reversed(range(0, len(rewards))):
            delta = rewards[k] + self.GAMMA * forward_val - v_values[k]
            gae_cumulative = self.GAMMA * self.GAE_LAMBDA * gae_cumulative + delta
            gae[k] = gae_cumulative
            forward_val = v_values[k]
            n_step_targets[k] = gae[k] + v_values[k]
        return gae, n_step_targets
    ## 배치에 저장된 데이터 추출
    def unpack_batch(self, batch):
        unpack = batch[0]
        for idx in range(len(batch)-1):
            unpack = np.append(unpack, batch[idx+1], axis=0)
        return unpack
    ## 액터 신경망 학습
    def actor_learn(self, log_old_policy_pdf, states, actions, gaes):
        with tf.GradientTape() as tape:
            # 현재 정책 확률밀도함수
            mu_a, std_a = self.actor(states, training=True)
            #print("actor mu_a: "+str(mu_a))
            #print("actor std_a: "+str(std_a))
            log_policy_pdf = self.log_pdf(mu_a, std_a, actions)
            #print("actor log_policy_pdf: "+str(log_policy_pdf))
            # 현재와 이전 정책 비율
            ratio = tf.exp(log_policy_pdf - log_old_policy_pdf)
            clipped_ratio = tf.clip_by_value(ratio, 1.0-self.RATIO_CLIPPING, 1.0+self.RATIO_CLIPPING)
            surrogate = -tf.minimum(ratio * gaes, clipped_ratio * gaes)
            loss = tf.reduce_mean(surrogate)
        grads = tape.gradient(loss, self.actor.trainable_variables)
        self.actor_opt.apply_gradients(zip(grads, self.actor.trainable_variables))
    ## 크리틱 신경망 학습
    def critic_learn(self, states, td_targets):
        with tf.GradientTape() as tape:
            td_hat = self.critic(states, training=True)
            loss = tf.reduce_mean(tf.square(td_hat-td_targets))

        grads = tape.gradient(loss, self.critic.trainable_variables)
        self.critic_opt.apply_gradients(zip(grads, self.critic.trainable_variables))
    #/////////////////////////////////////////////////////////////////////////////
    ## 신경망 파라미터 로드
    def load_weights(self, path):
        self.actor.load_weights(path + 'edie_rl_actor.h5')
        self.critic.load_weights(path + 'edie_rl_critic.h5')
    #/////////////////////////////////////////////////////////////////////////////
    def normalize_array(self, arr):
        total = sum(arr)
        normalized_arr = [x / total for x in arr]
        return normalized_arr
    def GetAction(self, action_dim, action):
        action_sector = 2/action_dim
        result = 0
        for a in range(int(action_dim)):
            if action[0] > a*action_sector and action[0] <= (a+1)*action_sector:
                result = a
        return result
    #/////////////////////////////////////////////////////////////////////////////
    ## 에피소드와 누적 보상값을 그려주는 함수
    def plot_result(self):
        plt.plot(self.save_epi_reward)
        self.png_number += 1
        self.png_number_string = str(self.png_number)
        self.png_name = self.file_path+'/../edie_rl_learn/edie_rl_learn/results/result_'+self.png_number_string+'.png'
        plt.savefig(self.png_name)
    def PrepareTrain(self, max_epi, max_batch):
         # 배치 초기화
        self.batch_state, self.batch_action, self.batch_reward = [], [], []
        self.batch_log_old_policy_pdf = []
        # episode 및 batch size 결정
        self.max_episode = max_epi
        self.max_batch_size = max_batch
    #/////////////////////////////////////////////////////////////////////////////
#################################################################################################