import rclpy
import requests
import json
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import ament_index_python.packages
import os
file_path = os.path.dirname(os.path.realpath(__file__))

class LlmMaumNode(Node):
    def __init__(self):
        super().__init__('llm_maum_node')

        self.publisher_ = self.create_publisher(String, '/heroehs/gemini/conversation/llm', 10)
        self.subscription_ = self.create_subscription(String, '/heroehs/gemini/conversation/stt', self.stt_callback, 10)
        self.dest_url = "https://norchestra.maum.ai/harmonize/dosmart"
        self.package_path = ament_index_python.packages.get_package_share_directory('gemini_llm')

    def stt_callback(self, msg):
        response_text = self.post_maumai(msg.data)
        self.get_logger().info(f"MAUM AI: {response_text}")

        response = String()
        response.data = str(response_text)
        self.publisher_.publish(response)

    def post_maumai(self, msg):
        f = open(f"{file_path}/../resource/prompt_maum.txt", "r")
        prompt_basic = f.read()

        data = {
            "app_id": "9f7a4c5e-bf8d-5329-b57f-27175e342cc1",
            "name": "test1",
            "item": ["chatgpt-35-turbo-rest"],
            "param": [{
                "model": "gpt-3.5-turbo",
                "messages": [
                    {
                        "role": "system",
                        "content": prompt_basic
                    },
                    {
                        "role": "user",
                        "content": msg
                    }
                ],
                "stream": False
            }]
        }
        self.get_logger().info("send to server")
        start_time = datetime.now()
        response = requests.post(self.dest_url, json=data)
        self.get_logger().info(f"Response Code: {response.status_code}")
        end_time = datetime.now()
        response_time = end_time - start_time
        self.get_logger().info(f"Server recognize done in {response_time.total_seconds()} sec")
        response_json = response.json()
        content = response_json["choices"][0]["message"]["content"]

        return content

def main(args=None):
    rclpy.init(args=args)
    node = LlmMaumNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
