import os
from typing import List, Union, Dict, Any, Optional, Tuple
from langchain_core.tools import tool

from aeirobot_llm.utils.call_ros_command import execute_ros_command
import time


emotions = {
        1: "궁금함",    
        2: "졸림",      
        3: "웃김(즐거움)",       
        4: "기쁨(뿌듯함)",     
        5: "슬픔",       
        6: "놀람",       
        7: "매우놀람",     
        8: "실망",        
        9: "사랑",      
        10: "어지러움",        
        11: "아주 어지러움",        
        # "low-dattery": 12,
    }

emotion_str_to_index = {
    "curiosity": 1,
    "sleepiness": 2,
    "amusement": 3,
    "happiness": 4,
    "sadness": 5,
    "surprise": 6,
    "greatsurprise": 7,
    "disappointment": 8,
    "love": 9,
    "dizziness": 10,
    "greatdizziness": 11,
}

def emotion_to_index(emotion: str) -> int:
    """
    감정명(str)을 action_index(int)로 변환합니다.
    매핑이 없거나 잘못된 경우 curiosity(1)로 반환합니다.
    """
    if not isinstance(emotion, str):
        return 1
    return emotion_str_to_index.get(emotion.strip().lower(), 1)

@tool
def call_expression_action(action_index: Optional[int] = 1) -> str:
    """
    Publishes an emotional action command to the Edie robot.

    This tool triggers the Edie robot's emotional response by publishing an `action_index`
    value to the ROS2 topic `/edie8/emotion/action_index`.  
    Each index corresponds to a predefined emotional state of the robot.

    Emotion Mapping:
        1: "curiosity"  
        2: "sleepiness"  
        3: "amusement"  
        4: "happiness"  
        5: "sadness"  
        6: "surprise"  
        7: "greatsurprise"  
        8: "disappointment"  
        9: "love"  
        10: "dizziness"  
        11: "greatdizziness"  

    Args:
        action_index (Optional[int], default=1):  
            The index number representing the desired emotion.  
            Must be an integer between 1 and 11.  
            Example: `action_index=4` → triggers the "happiness" emotion.

    Returns:
        str:  
            A short text message describing the emotion expressed by Edie.  
            Example: `"에디가 자신의 생각과 감정을 행복으로 표현하였습니다."`

    Usage:
        Use this tool when you want Edie to display a specific emotion.
        Choose the corresponding number from the emotion mapping list above.
    """
    
    cmd = f"ros2 topic pub -1 /edie8/emotion/action_index std_msgs/msg/UInt8 'data: {action_index}'"
    
    success, output = execute_ros_command(cmd)
    
    if not success:
        return [output]
    
    # time.sleep(1)
    cmd = "ros2 topic pub -1 /edie8/emotion/motion_done std_msgs/msg/Bool 'data: true'"
    success, output = execute_ros_command(cmd)
    cmd = "ros2 topic pub -1 /edie8/emotion/display_done std_msgs/msg/Bool 'data: true'"
    success, output = execute_ros_command(cmd)
    cmd = "ros2 topic pub -1 /edie8/emotion/sound_done std_msgs/msg/Bool 'data: true'"
    success, output = execute_ros_command(cmd)
    
    return f"에디가 자신의 생각과 감정을 {emotions[action_index]}으로 표현하였습니다."
