import os
import time
import importlib
from typing import Dict, Any

from std_msgs.msg import String

from aeirobot_dialogue.aeirobot_llm.aeirobot_llm.agents.agent_executor import ActionExecutor

from langchain_core.messages import AIMessage, HumanMessage, AIMessageChunk
from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.outputs import LLMResult

from langgraph.checkpoint.memory import MemorySaver
from langchain.agents import create_tool_calling_agent, AgentExecutor



from aeirobot_llm.models.llm import get_llm

import ament_index_python.packages

# Path configurations
package_share_dir = ament_index_python.packages.get_package_share_directory('aeirobot_llm')

prompt_default_path = os.path.join(package_share_dir, 'resource', "prompts", 'prompt_default.txt')
prompt_roboworld2024_path = os.path.join(package_share_dir, 'resource', "prompts", 'prompt_roboworld2024.txt')
city_names_path = os.path.join(package_share_dir, 'resource', 'city_names.yaml')

# Custom callback handler
class RosPubHandler(BaseCallbackHandler):
    def __init__(self, publisher):
        super().__init__()
        self.publisher_ = publisher  # publisher 인스턴스 저장
        self.current_string = ""  # 현재까지 누적된 토큰을 저장할 변수

    def on_llm_new_token(self, token: str, **kwargs) -> None:
        if token.startswith(' '):  # 토큰이 공백으로 시작하는 경우
            if self.current_string:  # 이미 누적된 문자열이 있으면 출력
                # print(self.current_string)
                response_string = String()
                response_string.data = str(self.current_string)
                self.publisher_.publish(response_string)
                # self.current_string = token.strip()  # 공백 제거 후 새 문자열 시작
                self.current_string = token  # 공백 제거 후 새 문자열 시작
            else:
                # self.current_string = token.strip()  # 공백 제거 후 새 문자열 시작
                self.current_string = token  # 공백 제거 후 새 문자열 시작
        else:
            self.current_string += token  # 공백으로 시작하지 않으면 문자열에 추가

    def on_llm_end(self, response: LLMResult, **kwargs) -> None:
        # 처리가 끝났을 때 남은 문자열 출력
        if self.current_string:
            # print(self.current_string)
            response_string = String()
            response_string.data = str(self.current_string)
            self.publisher_.publish(response_string)

            time.sleep(0.1)
            response_string.data = str("<EOS>")
            self.publisher_.publish(response_string)
            self.current_string = ""

    def on_tool_start(
        self, serialized: Dict[str, Any], input_str: str, **kwargs: Any
    ) -> Any:
        """Run when tool starts running."""
        print("serialized : ", serialized)
        print("input_str : ", input_str)
        # print("kwargs : ", kwargs)
        # print("Tool start")

# Main LangchainGPT class
class LangchainAgent:
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
        
        # 동적으로 tool 모듈 import
        tool_packages = []
        for tool_module_name in self.config["tools"]["packages"]:
            try:
                tool_module = importlib.import_module(tool_module_name)
                tool_packages.append(tool_module)
            except ImportError as e:
                print(f"Warning: Failed to import tool module '{tool_module_name}': {e}")
        
        
        # ActionExecutor 초기화
        self.agent_executor = ActionExecutor(
            llm=self.llm,
            # add_prompt=topic_prompts,
            prompt_packages=self.config["prompts"]["packages"],
            tool_packages=tool_packages,
            verbose=self.config["session"]["verbose"],
            session_id=self.config["session"]["session_id"],
            memory_chat_history=self.config["session"]["memory_chat_history"],
            )


        # os.system('clear')
        print("GPT initialized.")

    def update_thread_id(self, new_thread_id):
        self.thread_id = new_thread_id

    def process(self, input_text):
        response_string = String()
        response_string.data = str("<BOS>")
        self.publisher_.publish(response_string)

        config = {"configurable": {"thread_id": self.thread_id}, "callbacks": [RosPubHandler(self.publisher_)]}
        result = self.agent_executor.chat(
            query=input_text, 
            config=config
            )

        return result

    def delete_memory(self):
        """Delete the chat history memory."""
        self.agent_executor.delete_memory()
