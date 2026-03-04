'''
conversationchain 사용하는 기본적인 방법
'''

import deepl
import time
import json
from os.path import join

import requests
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain import PromptTemplate, LLMChain
from langchain.chains import ConversationChain
from langchain.memory import ConversationBufferMemory
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
resoruce_path = join(current_directory, '..', '..', 'share', 'gemini_llm', 'resource')
CACHE_PATH = join(resoruce_path,'langchain.db')
prompt_file_path = os.path.join(current_directory, '..', 'resource', 'prompt_template.txt')

class LangchainGPT:
    def __init__(self):
        os.environ['OPENAI_API_KEY'] = "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"
        self.llm_gpt = self.initialize_gpt()
        self.prompt = self.initialize_prompt()
        self.memory_gpt = ConversationBufferMemory(ai_prefix="에이미", human_prefix="Human",)
        PROMPT = PromptTemplate(input_variables=["history", "input"], template=self.prompt)
        self.llm_conversation_chain = ConversationChain(prompt=PROMPT, llm=self.llm_gpt, memory=self.memory_gpt)
        os.system('clear')
        print("GPT initialized.")

    def initialize_gpt(self):
        llm = ChatOpenAI(model="gpt-3.5-turbo",
                         temperature = 0.8,
                         model_kwargs={"frequency_penalty": 1})
        return llm

    def initialize_prompt(self):
        with open(prompt_file_path, 'r') as file:
            prompt_basic = file.read()

        template = """
        Answer in Korean.
        Current conversation:
        {history}
        
        Human: {input}
        에이미:
        """
        prompt_data = prompt_basic + template
        
        return prompt_data


    def delete_memory(self):
        self.memory_gpt.clear()

    def process(self, input_text):
        start_time_chat = time.time()
        res=self.llm_conversation_chain.predict(input=input_text)
        end_time_chat = time.time()

        chat_duration = end_time_chat - start_time_chat
        print(f"\nGPT: {chat_duration:.3f}")

        return res