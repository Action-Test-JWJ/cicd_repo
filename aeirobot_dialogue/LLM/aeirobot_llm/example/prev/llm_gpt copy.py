import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from dotenv import load_dotenv

# from .langchain_gpt import LangchainGPT
# from .langchain_gpt_conversation import LangchainGPT
# from .langchain_gpt_original import LangchainGPT

import os

'''
conversationchain 사용하는 summary 방법
'''

# import deepl
import time
import json
from os.path import join

import requests
# from langchain.llms import OpenAI
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import ConversationChain
from langchain.memory import ConversationSummaryBufferMemory, ConversationTokenBufferMemory
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.outputs import LLMResult

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
resoruce_path = join(current_directory, '..', '..', 'share', 'gemini_llm', 'resource')
CACHE_PATH = join(resoruce_path,'langchain.db')
prompt_file_path = os.path.join(current_directory, '..', 'resource', 'prompt_template.txt')

# class MyCustomHandler(BaseCallbackHandler):
#     def on_llm_new_token(self, token: str, **kwargs) -> None:
#         print(f"My custom handler, token: {token}")


class MyCustomHandler(BaseCallbackHandler):
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

class LangchainGPT:
    def __init__(self, publisher):
        self.publisher_ = publisher  # 여기서 publisher_를 저장

        # Load environment variables
        current_file_path = os.path.abspath(__file__)
        current_directory = os.path.dirname(current_file_path)
        env_path = os.path.join(current_directory, '..', '.env')
        load_dotenv(env_path)

        self.llm_gpt = self.initialize_gpt()
        self.prompt = self.initialize_prompt()
        self.memory_summary = ConversationSummaryBufferMemory(llm=self.llm_gpt, return_messages=False, ai_prefix="에이미", human_prefix="Human", max_token_limit=40)
        self.memory_token = ConversationTokenBufferMemory(llm=self.llm_gpt, max_token_limit=1000)
        PROMPT = PromptTemplate(input_variables=["summary", "new_lines"], template=self.prompt)
        self.Conversation_summary_buf = ConversationChain(
            prompt=PROMPT,
            llm=self.llm_gpt,
            memory=self.memory_token,
            verbose=False,
        )
        # os.system('clear')
        print("GPT initialized.")

    def initialize_gpt(self):
        llm = ChatOpenAI(model="gpt-4o-mini",
                         temperature = 0.8,
                         streaming=True,
                         callbacks=[MyCustomHandler(self.publisher_)])
                        #  model_kwargs={"frequency_penalty": 1.2})

        return llm

    def initialize_prompt(self):
        with open(prompt_file_path, 'r') as file:
            prompt_basic = file.read()

        template = """
        Answer in Korean.
        Current conversation:
        {history}
        Human: {input}
        에이미:"""
        prompt_data = prompt_basic + template

        return prompt_data

    def delete_memory(self):
        self.memory_token.clear()
        self.memory_summary.clear()

    def process(self, input_text):
        time.sleep(0.1)

        response_string = String()
        response_string.data = str("<BOS>")
        self.publisher_.publish(response_string)

        start_time_chat = time.time()
        res = self.Conversation_summary_buf.predict(input=input_text)
        end_time_chat = time.time()
        chat_duration = end_time_chat - start_time_chat

        return res

class LlmGPTNode(Node):
    def __init__(self):
        super().__init__('llm_gpt_node')

        self.publisher_ = self.create_publisher(String, '/heroehs/aimy/dialogue/llm', 10)
        self.publisher_full = self.create_publisher(String, '/heroehs/aimy/dialogue/llm/full', 10)

        self.sub_llm_clear = self.create_subscription(Bool, '/heroehs/aimy/commander/llm/clear', self.llm_clear_callback, 10)
        self.sub_stt = self.create_subscription(String, '/heroehs/aimy/manage/dialogue/stt', self.stt_callback, 10)

        # LangchainLlama 객체 초기화
        self.langchain = LangchainGPT(publisher=self.publisher_)

    def stt_callback(self, msg):
        response_text = self.langchain.process(msg.data)
        # self.get_logger().info(f"GPT: {response_text}")
        self.get_logger().info(f"\n{response_text}")

        response = String()
        response.data = str(response_text)
        self.publisher_full.publish(response)

    def llm_clear_callback(self, msg):
        if msg.data == True:
            self.langchain.delete_memory()

def main(args=None):
    rclpy.init(args=args)
    node = LlmGPTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
