import time
from typing import Dict, Any

from std_msgs.msg import String

from aeirobot_llm.agents.llm_executor import QAExecutor

from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.outputs import LLMResult

from aeirobot_llm.models.llm import get_llm

from aeirobot_llm.utils.parsing_answer_to_tool import parse_emotion_from_answer
from aeirobot_llm.toolbox.expression import emotion_to_index, call_expression_action

import ament_index_python.packages

# Path configurations
package_share_dir = ament_index_python.packages.get_package_share_directory('aeirobot_llm')


# Custom callback handler
class RosPubHandler(BaseCallbackHandler):
    def __init__(self, publisher):
        super().__init__()
        self.publisher_ = publisher  # publisher 인스턴스 저장
        self.current_string = ""  # 현재까지 누적된 토큰을 저장할 변수

    def on_llm_new_token(self, token: str, **kwargs) -> None:
        if token.startswith(' '):  # 토큰이 공백으로 시작하는 경우
            if self.current_string:  # 이미 누적된 문자열이 있으면 출력
                response_string = String()
                response_string.data = str(self.current_string)
                self.publisher_.publish(response_string)
                self.current_string = token  # 공백 제거 후 새 문자열 시작
            else:
                self.current_string = token  # 공백 제거 후 새 문자열 시작
        else:
            self.current_string += token  # 공백으로 시작하지 않으면 문자열에 추가

    def on_llm_end(self, response: LLMResult, **kwargs) -> None:
        # 처리가 끝났을 때 남은 문자열 출력
        if self.current_string:
            response_string = String()
            response_string.data = str(self.current_string)
            self.publisher_.publish(response_string)

            time.sleep(0.1)
            response_string.data = str("<EOS>")
            self.publisher_.publish(response_string)
            self.current_string = ""


# Main LangchainLLM class
class LangchainLLM:
    def __init__(self, 
                 publisher, 
                 publisher_tools, 
                 thread_id,
                 config
                 ):
        self.publisher_ = publisher
        self.publisher_tools = publisher_tools
        self.thread_id = thread_id
        self.config = config["agent__parameters"]
        
        # LLM 초기화
        self.llm = get_llm(
            model_name=self.config["llm"]["model_name"], 
            temperature=self.config["llm"]["temperature"]
            )
        
        # QAExecutor 초기화 (도구 없는 순수 LLM)
        self.qa_executor = QAExecutor(
            llm=self.llm,
            prompt_packages=self.config["prompts"]["packages"],
            verbose=self.config["session"]["verbose"],
            session_id=self.config["session"]["session_id"],
            memory_chat_history=self.config["session"]["memory_chat_history"],
            )

        print("LLM initialized.")

    def update_thread_id(self, new_thread_id):
        self.thread_id = new_thread_id

    def process(self, input_text):
        response_string = String()
        response_string.data = str("<BOS>")
        self.publisher_.publish(response_string)

        config = {"configurable": {"thread_id": self.thread_id}, "callbacks": [RosPubHandler(self.publisher_)]}
        result = self.qa_executor.chat(
            query=input_text, 
            config=config
            )

        # 감정명 추출
        try:
            print('Parsing emotion from answer...')
            emotion = parse_emotion_from_answer(result)
            if not emotion:
                emotion = "happiness"
        except Exception:
            emotion = "happiness"
        # 인덱스 변환
        try:
            print('Converting emotion to index...')
            action_index = emotion_to_index(emotion)
        except Exception:
            action_index = 1
        # ROS 토픽 발행
        try:
            print('Calling expression action...')
            call_expression_action.func(action_index)
        except Exception as e:
            print(f"call_expression_action failed: {e}")

        return result

    def delete_memory(self):
        """Delete the chat history memory."""
        self.qa_executor.delete_memory()
