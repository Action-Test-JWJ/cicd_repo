import rclpy
import requests
import json
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import ament_index_python.packages
import os
from urllib.parse import quote
file_path = os.path.dirname(os.path.realpath(__file__))

class LlmAnytalkNode(Node):
    def __init__(self):
        super().__init__('llm_anytalk_node')

        self.publisher_ = self.create_publisher(String, '/heroehs/aimy/dialogue/llm', 10)
        self.publisher_full_ = self.create_publisher(String, '/heroehs/aimy/dialogue/llm/full', 10)
        self.subscription_ = self.create_subscription(String, '/heroehs/aimy/manage/dialogue/stt', self.stt_callback, 10)
        self.dest_url = "https://anytalk.com:8082/message/requestAnytalkAnswer"
        self.package_path = ament_index_python.packages.get_package_share_directory('aimy_llm')
        self.get_logger().info(f"LLM Anytalk Node has been started")

    def stt_callback(self, msg):
        response = String()
        response.data = str("<BOS>")
        self.publisher_.publish(response)

        response_text = self.post_anytalk(msg.data)
        self.get_logger().info(f"Anytalk: {response_text}")

        response = String()
        response.data = str(response_text)
        self.publisher_.publish(response)
        self.publisher_full_.publish(response)

        response = String()
        response.data = str("<EOS>")
        self.publisher_.publish(response)

    def post_anytalk(self, msg):
        encoded_msg = quote(msg)

        url = f"{self.dest_url}?systemToken=1A202B788C13&askStr={encoded_msg}"

        self.get_logger().info("send to server")
        start_time = datetime.now()
        response = requests.get(url, verify=False)
        self.get_logger().info(f"Response Code: {response.status_code}")
        end_time = datetime.now()
        response_time = end_time - start_time
        self.get_logger().info(f"Server recognize done in {response_time.total_seconds()} sec")

        if response.status_code == 200:
            response_json = response.json()
            # self.get_logger().info(f"Response JSON: {response_json}")
            if response_json['resultCode'] == 'ERRK':
                content = response_json['answerStr']
            else:
                content = response_json.get('answerStr', '응답 내용을 찾을 수 없습니다.')
        else:
            content = "서버로부터 정상적인 응답이 도착하지 않았습니다. 죄송합니다."

        return content

def main(args=None):
    rclpy.init(args=args)
    node = LlmAnytalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
