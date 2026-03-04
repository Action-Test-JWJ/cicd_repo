'''
conversationchain 사용하는 summary 방법
'''

# import deepl
import time
import json
from os.path import join

import requests
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain import PromptTemplate, LLMChain
from langchain.chains import ConversationChain
from langchain.memory import ConversationSummaryBufferMemory
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
    def __init__(self):
        super().__init__()
        self.current_string = ""  # 현재까지 누적된 토큰을 저장할 변수

    def on_llm_new_token(self, token: str, **kwargs) -> None:
        if token.startswith(' '):  # 토큰이 공백으로 시작하는 경우
            if self.current_string:  # 이미 누적된 문자열이 있으면 출력
                print(self.current_string)
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
            print(self.current_string)
            self.current_string = ""

class LangchainGPT:
    def __init__(self):
        os.environ['OPENAI_API_KEY'] = "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"
        self.llm_gpt = self.initialize_gpt()
        self.prompt = self.initialize_prompt()
        self.memory_gpt = ConversationSummaryBufferMemory(llm=self.llm_gpt, return_messages=True, ai_prefix="에이미", human_prefix="Human",)
        PROMPT = PromptTemplate(input_variables=["summary", "new_lines"], template=self.prompt)
        self.Conversation_summary_buf = ConversationChain(
            prompt=PROMPT,
            llm=self.llm_gpt,
            memory=self.memory_gpt,

            # verbose=True,
        )
        os.system('clear')
        print("GPT initialized.")

    def initialize_gpt(self):
        llm = ChatOpenAI(model="gpt-3.5-turbo",
                         temperature = 0.8,
                         streaming=True,
                         callbacks=[MyCustomHandler()],
                         model_kwargs={"frequency_penalty": 1.2})

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
        self.memory_gpt.clear()

    def process(self, input_text):
        start_time_chat = time.time()
        # print(self.Conversation_summary_buf.predict(input=input_text))
        # print(res)
        # res = self.Conversation_summary_buf.predict(input=input_text)
        # res_full = res
        end_time_chat = time.time()
        chat_duration = end_time_chat - start_time_chat

        return self.Conversation_summary_buf.predict(input=input_text)
