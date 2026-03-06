import rclpy
import requests
import json
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import ament_index_python.packages
import os
file_path = os.path.dirname(os.path.realpath(__file__))

class LlmNextLabNode(Node):
    def __init__(self):
        super().__init__('llm_nextlab_node')

        self.publisher_ = self.create_publisher(String, '/heroehs/gemini/conversation/llm', 10)
        self.subscription_ = self.create_subscription(String, '/heroehs/gemini/conversation/stt', self.stt_callback, 10)
        self.dest_url = "https://38tisrvjdi.execute-api.ap-northeast-2.amazonaws.com/api/chat"
        self.package_path = ament_index_python.packages.get_package_share_directory('gemini_llm')

    def stt_callback(self, msg):
        response_text = self.post_nextlab(msg.data)
        self.get_logger().info(f"NEXT Lab: {response_text}")

        response = String()
        response.data = str(response_text)
        self.publisher_.publish(response)

    def post_nextlab(self, msg):
        # f = open(f"{file_path}/../resource/prompt_maum.txt", "r")
        # prompt_basic = f.read()

        data = {
            "message": msg,
            "max_length": 30,
            "is_use_cache": True,
            "is_new_question": True,
            "identity": "arobot-test"
        }
        self.get_logger().info("send to server")
        start_time = datetime.now()
        response = requests.post(self.dest_url, json=data)
        self.get_logger().info(f"Response Code: {response.status_code}")
        end_time = datetime.now()
        response_time = end_time - start_time
        self.get_logger().info(f"Server recognize done in {response_time.total_seconds()} sec")

        if response.status_code == 200:
            response_json = response.json()
            content = response_json["object"][0]['answer']
        else:
            content = "서버로부터 정상적인 응답이 도착하지 않았습니다. 죄송합니다."

        return content

def main(args=None):
    rclpy.init(args=args)
    node = LlmNextLabNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
