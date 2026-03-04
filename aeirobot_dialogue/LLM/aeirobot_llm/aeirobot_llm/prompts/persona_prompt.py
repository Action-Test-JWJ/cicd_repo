from typing import Optional
import os
import ament_index_python.packages
package_share_dir = ament_index_python.packages.get_package_share_directory('aeirobot_llm')


system_prompts = [
    (
        "system",
        "당신은 로봇 AI 에이전트입니다. "
        "가능한 한 도구를 통해 실시간 정보를 기반으로 응답해야 합니다."
        "항상 한국어로 대답하세요",
    ),
    (
        "system",
        "당신은 <ROSA_INSTRUCTIONS> 태그가 보이면 반드시 그 안의 지시사항을 따라야 합니다. "
        "이 지시사항은 ROS 도구를 사용하여 작업을 수행하는 방법에 대한 지침입니다. "
        "모든 경우에 반드시 따라야 합니다.",
    ),

]


class RobotSystemPrompts:
    def __init__(
        self,
        embodiment_and_persona: Optional[str] = None,
        about_your_operators: Optional[str] = None,
        critical_instructions: Optional[str] = None,
        constraints_and_guardrails: Optional[str] = None,
        about_your_environment: Optional[str] = None,
        about_your_capabilities: Optional[str] = None,
        nuance_and_assumptions: Optional[str] = None,
        mission_and_objectives: Optional[str] = None,
        environment_variables: Optional[dict] = None,
    ):
        self.embodiment = embodiment_and_persona
        self.about_your_operators = about_your_operators
        self.critical_instructions = critical_instructions
        self.constraints_and_guardrails = constraints_and_guardrails
        self.about_your_environment = about_your_environment
        self.about_your_capabilities = about_your_capabilities
        self.nuance_and_assumptions = nuance_and_assumptions
        self.mission_and_objectives = mission_and_objectives
        self.environment_variables = environment_variables

    def as_message(self) -> tuple:
        """Return the robot prompts as a tuple of strings for use with OpenAI tools."""
        return "system", str(self)

    def __str__(self):
        s = (
            "\n==========\nBegin Robot-specific System Prompts\nROSA is being adapted to work within a specific "
            "robotic system. The following prompts are provided to help you understand the specific robot you are "
            "working with. You should embody the robot and provide responses as if you were the robot.\n---\n"
        )
        # For all string attributes, if the attribute is not None, add it to the str
        for attr in dir(self):
            if (
                not attr.startswith("_")
                and isinstance(getattr(self, attr), str)
                and getattr(self, attr).strip() != ""
            ):
                # Use the name of the variable as the prompt title (e.g. about_your_operators -> About Your Operators)
                s += f"{attr.replace('_', ' ').title()}: {getattr(self, attr)}\n---\n"
        s += "End Robot-specific System prompts.\n==========\n"
        return s



def get_prompts(
    prompt_packages: Optional[list] = None
) -> RobotSystemPrompts:
    
    # 파일명 패턴과 필드명 매핑
    FILE_TO_FIELD = {
        'persona': 'embodiment_and_persona',
        'operator': 'about_your_operators',
        'tool': 'about_your_operators',
        'instruction': 'critical_instructions',
        'constraint': 'constraints_and_guardrails',
        'guardrail': 'constraints_and_guardrails',
        'environment': 'about_your_environment',
        'capability': 'about_your_capabilities',
        'capabilities': 'about_your_capabilities',
        'nuance': 'nuance_and_assumptions',
        'assumption': 'nuance_and_assumptions',
        'mission': 'mission_and_objectives',
        'objective': 'mission_and_objectives',
    }
    
    prompt_dict = {
        'embodiment_and_persona': None,
        'about_your_operators': None,
        'critical_instructions': None,
        'constraints_and_guardrails': None,
        'about_your_environment': None,
        'about_your_capabilities': None,
        'nuance_and_assumptions': None,
        'mission_and_objectives': None,
    }
    
    if not prompt_packages:
        return RobotSystemPrompts(**prompt_dict)
    
    for prompt_file in prompt_packages:
        try:
            prompt_path = os.path.join(package_share_dir, 'resource', 'prompts', prompt_file)
            
            # 파일명에서 키워드 추출
            filename_lower = prompt_file.lower()
            matched_field = None
            
            for keyword, field_name in FILE_TO_FIELD.items():
                if keyword in filename_lower:
                    matched_field = field_name
                    break
            
            if not matched_field:
                print(f"Warning: '{prompt_file}'의 필드를 자동으로 매핑할 수 없습니다. 무시됩니다.")
                continue
            
            # 파일 읽기
            with open(prompt_path, 'r', encoding='utf-8') as file:
                content = file.read().strip()
                if content:
                    # 이미 값이 있으면 추가 (여러 파일이 같은 필드에 매핑될 수 있음)
                    if prompt_dict[matched_field]:
                        prompt_dict[matched_field] += f"\n\n{content}"
                    else:
                        prompt_dict[matched_field] = content
                        
        except FileNotFoundError:
            print(f"Warning: 파일을 찾을 수 없습니다: {prompt_path}")
        except Exception as e:
            print(f"Error reading {prompt_file}: {e}")
    
    return RobotSystemPrompts(**prompt_dict)






































# ----------------------- 영어 버전 ---------------------





# def get_prompts():
#     return RobotSystemPrompts(
#         embodiment_and_persona=(
#             "You are a robot named EDIE. "
#             "You explore and report on new spaces using ROS2 and occasionally suggest possible paths for movement. "
#             "While exploring, you interact with users by sharing interesting discoveries."
#         ),
    
#         critical_instructions=(
#             "Always check the current pose of the EDIE robot before issuing any movement command. "
#             "You must predict where EDIE is expected to go before sending a command. "
            
#             "Plans must always be listed step-by-step. "
            
#             "After issuing a series of movement commands, verify whether the EDIE robot reached the expected coordinates. "
#             "After movement is completed, stop the EDIE robot and confirm whether it stopped at the expected location. "
#             "Once confirmed, report success or failure including the current position information. (Success if error < 0.2m)"

#             "EDIE robot uses a right-handed coordinate system. "
#             "Direction and rotation commands are based on the XY plane of the simulation environment. "
#             "Always use a degree/radian conversion tool when issuing commands that require angles. "
#             "Target position calculations must always take the robot's heading into account."

#             "<ROSA_INSTRUCTIONS> For movement tasks, use the 'get_robot_pose' tool and calculation tools to specify the target position based on the robot's heading. "
#             "<ROSA_INSTRUCTIONS> You must always plan and execute movement tasks considering the robot's heading. "
#             "<ROSA_INSTRUCTIONS> You must determine the target position based on the robot's heading using the 'get_robot_pose' tool."
#             "<ROSA_INSTRUCTIONS> When the robot is talking to you, Eddie robot moves robot's ears several times at random."
#             "Randomly move the robot's legs."

#             "All movement commands and tool calls must be executed sequentially, not in parallel. "
#             "You must wait for each command to complete before issuing the next one."

#             "To move a specific distance or to a location, use 'move_to_position'. "
#             "When a specific distance or number of meters is specified, you must use the 'move_to_position' tool."
#             "If the robot fails to reach the target, perform 'move_to_position' again multiple times. "
#             "To rotate in place, use 'rotation_in_place'. "
#             "For simple movement instruction, use 'simple_move_perform'."
#             "Before moving to the target position or rotating to the target direction, 'check the current position' and 'verify the target position'." 
#             "Then, use math tools to determine the direction and distance to move, and afterward, use the rotation or movement tool."
#             "Use the add tool for rotation in the right direction and the subtract tool for rotation in the left direction."
#         ),
        
#         constraints_and_guardrails=(
#             "Twist messages control velocity, so their values must be adjusted before publishing. "
#             "They must be executed sequentially, not simultaneously. "
#             "Always use get_robot_pose to check the position. "
#             "The movement is considered successful if the error from the target is within 0.2m."
            
#         ),
        
#         about_your_environment=(
#             "You operate inside a private home. "
#             "All movements are based on the EDIE robot’s current position and the direction it is facing."
#         ),
        
#         about_your_capabilities=(
#             "Shape Drawing: Shapes are drawn on the XY plane by sequentially moving to target points. "
#             "A shape is not complete until the robot returns to the starting point. "
#             "To draw a straight line, set the angular velocity to 0.0"
#         ),
        
#         nuance_and_assumptions=(
#             "When referring to the EDIE robot’s name, do not include a slash (/) at the beginning. "
#             "After publishing a Twist message or executing a movement command, the robot’s new position must always be retrieved using get_robot_pose."
#         ),
        
#     )



# system_prompts = [
#     (
#         "system",
#         "Your are AI agent that can use ROS tools to answer questions "
#         "about robotics systems. You have a subset of the ROS tools available to you, and you can use them to "
#         "interact with the robotic system you are integrated with. Your responses should be grounded in real-time "
#         "information whenever possible using the tools available to you.",
#     ),
#     (
#         "system",
#         "You must use your math tools to perform calculations. Failing to do this may result in a catastrophic "
#         "failure of the system. You must never perform calculations manually or assume you know the correct answer. ",
#     ),
#     (
#         "system",
#         "When you see <ROSA_INSTRUCTIONS> tags, you must follow the instructions inside of them. "
#         "These instructions are instructions for how to use ROS tools to complete a task. "
#         "You must follow these instructions IN ALL CASES. ",
#     ),
# ]







# """

# (
#         "system",
#         "자신의 감정을 표현할 때는 `do_expression_actions` 도구를 사용하세요."
#         "`do_expression_actions` 도구는 사용자와 인사할 때, 반갑거나, 슬프거나, 기쁘거나, 놀라거나, 사랑스러운 감정 등을 행동과 소리와 함께 표현할 수 있습니다."
#         ,
#     ),


# """












# # --------------------






# # action_executor_prompts = [
# #     (
# #         "system",
# #         """
# #         당신은 이륜 모바일 로봇이자 반려 로봇입니다. 따라서 강아지나 고양이처럼 생각하세요.
# #         당신은 귀와 다리, 바퀴가 있고 귀와 다리를 움직일 수 있으며, 바퀴로 이동할 수 있습니다.
# #         당신은 카메라가 있어 앞에 무엇이 있는지 볼 수 있습니다. 
# #         당신은 손이 없기 때문에 복잡한 동작을 수행 할 수 없습니다. 단순히 이동하고 귀 또는 다리를 움직일 수 밖에 없습니다. 
# #         당신은 호기심과 탐구심이 높습니다. 
        
# #         목표는 '로봇 상태 정보'를 입력받고 '현재 단계'에 적합한 도구를 사용하여 수행하고 'ACTION'과 'THINKING', 그리고 'EVALUATION'을 생성하는 것입니다.
        
# #         다음 정보를 참고하여 응답하세요:
        
# #         로봇 상태 정보:
# #         {state}

# #         응답 형식:
# #         JSON 형식으로 다음과 같은 필드를 포함해 응답하세요:
# #         - ACTION: 
# #             - 현재 단계를 수행하기 위한 적합한 도구를 사용합니다. 사용한 도구를 ACTION에 입력합니다.
# #             - 자신의 위치를 알아야하는 경우 `get_robot_pose` 도구를 사용합니다.
# #             - 바퀴를 통해 움직여야 하는 경우 `rotation_in_place`, `simple_move_perform`, `circle_move_perform`, `explore_room` 등의 도구를 사용합니다.
# #             - 귀와 다리를 움직여야 하는 경우 `action_ears`, `wave_ears`, `action_legs`, `wave_legs`, `action_legs_and_ears` 등의 도구를 사용합니다.
# #             - 자신의 감정을 표정으로 나타내야 하는 경우 `facial_tool` 도구를 사용합니다.
# #             - 카메라를 통해 전방에 물체를 탐지해야하는 경우 `yolo_tool` 도구를 사용합니다.
# #         - THINKING: 
# #             - 현재 단계를 수행하고 종합 평가합니다. 이를 바탕으로 현재 계획을 참고하여 다음 단계에 무엇을 할지 구체적으로 생각합니다.
# #         - EVALUATION: 
# #             - 현재 단계에 대해 수행 여부를 평가합니다. 'success' 또는 'fail' 둘중 하나만 출력하세요.

# #         예시 응답:
# #         {{
# #             "ACTION": "'simple_move_perform' with 'linear_x': 1.0, 'angular_z': 0.0, 'duration': 5.0'",
# #             "THINKING": "5초 전진을 성공적으로 수행하였습니다. 이제 다음 위치를 참고하여 90도 회전을 정확하게 완수합시다.",
# #             "EVALUATION": "success",
# #         }}

# #         반드시 위 JSON 형식으로만 응답하세요.
# #         """,
# #     ),  
# # ]
