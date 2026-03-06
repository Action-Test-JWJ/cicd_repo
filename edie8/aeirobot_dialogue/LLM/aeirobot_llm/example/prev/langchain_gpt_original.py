'''
오리지널 방법 (텍스트파일 불러오는 부분 변경)
'''

import deepl
import time
import json
from os.path import join

import requests
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain import PromptTemplate, LLMChain
from langchain.memory import ConversationBufferMemory
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

file_path = os.path.dirname(os.path.realpath(__file__))

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
resoruce_path = join(current_directory, '..', '..', 'share', 'gemini_llm', 'resource')
# home_directory = expanduser("~")
# llm_folder_path = join(home_directory,'git','gemini','5_conversation','gemini_llm','gemini_llm')
CACHE_PATH = join(resoruce_path,'langchain.db')
prompt_file_path = os.path.join(current_directory, '..', 'resource', 'prompt_template.txt')

class LangchainGPT:
    def __init__(self):
        os.environ['OPENAI_API_KEY'] = "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"
        self.llm_gpt = self.initialize_gpt()
        self.prompt = self.initialize_prompt()
        self.memory_gpt = ConversationBufferMemory(memory_key="chat_history")
        self.llm_chain_gpt = LLMChain(prompt=self.prompt, llm=self.llm_gpt, memory=self.memory_gpt)
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


        # f = open(PROMPT_PATH, "r")    
        # prompt_basic = f.read()
            
        template = """
        Answer in Korean.
        previous conversation:

        Question: {question}, Instruction : Use at most 30 words and at most 2 sentences, Don't verbatim sentences that consist of instructions

        Answer:
        """
        prompt_data = prompt_basic + template
        prompt = PromptTemplate.from_template(prompt_data)
        return prompt

    def delete_memory(self):
        self.memory_gpt.clear()

    def process(self, input_text):
        start_time_chat = time.time()
        # prompt = f"Question: {Question_eng}, Answer:"
        output = self.llm_chain_gpt({"question": input_text})
        end_time_chat = time.time()
        # print(output)

        chat_duration = end_time_chat - start_time_chat
        print("GPT:\n", output['text'])
        print(f"\nGPT: {chat_duration:.3f}")

        return output['text']