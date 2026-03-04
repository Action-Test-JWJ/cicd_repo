import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from aeirobot_llm.agents.agent import LangchainAgent

# Configure QoS profile for publishing and subscribing
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)

# Path configurations
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
sys.path.append(current_directory)


# ROS2 Node
class AgentGPTNode(Node):
    def __init__(self):
        super().__init__('agent_gpt_node')

        # config 파일 경로 파라미터 받기
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        if not config_file:
            config_file = os.path.join(current_directory, '..', 'config', 'agent_config.yaml')
            self.get_logger().info(f'Using default config file: {config_file}')
        
        # yaml 파일 로드
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
                if 'aeirobot_llm' in config_data:
                    self.config = config_data['aeirobot_llm']
                else:
                    self.config = {}
                    self.get_logger().error('Invalid config file format')
                    return
            self.get_logger().info(f'Loaded config from: {config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config file: {str(e)}')
            return
        
        
        
        self.user_count = 0
        self.current_user_id = self.generate_user_id()

        self.pub_llm = self.create_publisher(String, self.config["ros__parameters"]["pub"]["pub_llm"] , 10)
        self.pub_llm_full = self.create_publisher(String, self.config["ros__parameters"]["pub"]["pub_llm_full"], 10)
        self.pub_llm_tools = self.create_publisher(String, self.config["ros__parameters"]["pub"]["pub_llm_tools"], 10)

        self.sub_llm_clear = self.create_subscription(Bool, self.config["ros__parameters"]["sub"]["sub_llm_clear"], self.llm_clear_callback, 10)

        self.sub_stt = self.create_subscription(String, self.config["ros__parameters"]["sub"]["sub_stt"], self.stt_callback, 10)

        # New subscriber for process signal
        self.sub_llm_process = self.create_subscription(Bool, self.config["ros__parameters"]["sub"]["sub_llm_process"], self.llm_process_callback, 10)

        # LangchainLlama object initialization
        self.langchain = LangchainAgent(
            publisher=self.pub_llm, 
            publisher_tools=self.pub_llm_tools, 
            thread_id=self.current_user_id,
            config=self.config
            )

        # Store the latest STT input
        self.latest_stt_input = None

        self.sub_initialize = self.create_subscription(
            Bool, self.config["ros__parameters"]["sub"]["sub_llm_initialize"], self.initialize_callback, 10)

    def generate_user_id(self):
        self.user_count += 1
        return f"user_{self.user_count}"

    def stt_callback(self, msg):
        # Store the latest STT input
        self.latest_stt_input = msg.data
        self.get_logger().info(f"Received STT input: {self.latest_stt_input}")

        response_text = self.langchain.process(self.latest_stt_input)
        self.get_logger().info(f"\n{response_text}")

        response = String()
        response.data = str(response_text)
        self.pub_llm_full.publish(response)

    def llm_process_callback(self, msg):
        if msg.data and self.latest_stt_input:
            self.get_logger().info(f"Received LLM process signal: {msg.data}")
            response_text = self.langchain.process(self.latest_stt_input)
            self.get_logger().info(f"\n{response_text}")

            response = String()
            response.data = str(response_text)
            self.pub_llm_full.publish(response)

            # Clear the latest STT input after processing
            self.latest_stt_input = None
        elif not self.latest_stt_input:
            self.get_logger().warn("Received process signal, but no STT input available.")

    def llm_clear_callback(self, msg):
        if msg.data == True:
            self.langchain.delete_memory()

    def state_callback(self, msg):
        self.langchain.state = msg.data

    def initialize_callback(self, msg):
        if msg.data:
            self.current_user_id = self.generate_user_id()
            self.langchain.update_thread_id(self.current_user_id)
            self.latest_stt_input = None
            self.get_logger().info(f'Agent node initialized with new user ID: {self.current_user_id}')

# Main function
def main(args=None):
    rclpy.init(args=args)
    node = AgentGPTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
