import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from langchain_llama import LangchainLlama  # 재구성한 코드에서 LangchainLlama 클래스를 가져옵니다.

class LlmLlamaNode(Node):
    def __init__(self):
        super().__init__('llm_llama_node')

        # LangchainLlama 객체 초기화
        self.langchain = LangchainLlama()

        self.publisher_ = self.create_publisher(String, 'gemini/conversation/llm', 10)
        self.subscription = self.create_subscription(String, 'gemini/conversation/stt', self.stt_callback, 10)
        self.subscription = self.create_subscription(Bool, 'gemini/conversation/llm/clear', self.llm_clear_callback, 10)

    def stt_callback(self, msg):
        response_text = self.langchain.process(msg.data)
        response = String()
        response.data = str(response_text)
        self.publisher_.publish(response)

    def llm_clear_callback(self, msg):
        if msg.data == True:
            self.langchain.delete_memory()

def main(args=None):
    rclpy.init(args=args)
    node = LlmLlamaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
