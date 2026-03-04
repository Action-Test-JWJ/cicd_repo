import requests
from rclpy.node import Node 
from pathlib import Path

def call_stt_api(audio_name: str, language: str = None, api_conf: dict = None, node: Node = None):
    filename = Path(audio_name).name 
    
    if node is not None:
        node.get_logger().info("==== STT API 요청 시도 ====")
    else:
        print("==== STT API 요청 시도 ====")  # fallback
    assert node is not None, "ROS2 node (with get_logger) must be provided"

    node.get_logger().info(f'audio_name: {filename}')
    host = api_conf.get("host", None)
    port = api_conf.get("port", None)
    prefix = api_conf.get("prefix", None)

    url = f"http://{host}:{port}{prefix}"
    payload = {"audio_name": filename}
    # if language:
    #     payload["language"] = language

    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        data = response.json()
        return data["text"], data["language"], data["confidence"]

    except Exception as e:
        node.get_logger().error(f"[STT API ERROR] {e}")
        return None, None, 0.0
