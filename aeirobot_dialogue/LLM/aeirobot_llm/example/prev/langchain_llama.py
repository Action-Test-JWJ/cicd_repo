import deepl
import time
import json

import requests
from langchain.llms import LlamaCpp
from langchain import PromptTemplate, LLMChain
from langchain.memory import ConversationBufferMemory
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

file_path = os.path.dirname(os.path.realpath(__file__))

class LangchainLlama:
    def __init__(self):
        self.deepl_auth_key = "fb0c1acc-602f-68e1-a1cc-fda74fddaa38"
        self.translate_deepl = deepl.Translator(self.deepl_auth_key)
        self.llm_llama = self.initialize_llama()
        self.prompt = self.initialize_prompt()
        self.memory_llama = ConversationBufferMemory(memory_key="chat_history")
        self.llm_chain_llama = LLMChain(prompt=self.prompt, llm=self.llm_llama, memory=self.memory_llama)
        os.system('clear')
        print("Llama2 initialized.")

    def initialize_llama(self):
        llm_llama = LlamaCpp(
            model_path="/home/yh/ros2_ws/src/gemini_conversation/gemini_conversation/llama-cpp/llama-2-7b-chat.Q5_K_M.gguf",
            # model_path="/home/yh/ros2_ws/src/gemini_conversation/gemini_conversation/llama-cpp/Llama-2-ko-7B-chat-gguf-q4_1.bin",
            n_ctx=4096,
            n_batch=512,
            max_tokens=8192,
            n_gpu_layers=35,
            echo=True,
            temperature=0.85,
            top_p=1,
        )
        llm_llama.client.verbose = False

        return llm_llama

    def initialize_prompt(self):
        f = open(f"{file_path}/prompt_template.txt", "r")
        prompt_basic = f.read()
        template = """
        Answer in English

        previous conversation:
        {chat_history}

        Question: {question} Answer following Instruction : Summerize Answer in 25 words and no more than two sentences.
        Answer:
        """
        prompt_data = prompt_basic + template
        prompt = PromptTemplate.from_template(prompt_data)

        return prompt

    def delete_memory(self):
        self.memory_llama.clear()

    def translate_papago(self, text, source='ko', target='en'):
        CLIENT_ID, CLIENT_SECRET = '5bjfrxPr5Zf6OHAeHoUN', 'oit6QRMcrA'
        url = 'https://openapi.naver.com/v1/papago/n2mt'
        headers = {
            'Content-Type': 'application/json',
            'X-Naver-Client-Id': CLIENT_ID,
            'X-Naver-Client-Secret': CLIENT_SECRET
        }
        data = {'source': source, 'target': target, 'text': text}
        response = requests.post(url, json.dumps(data), headers=headers)

        return response.json()['message']['result']['translatedText']

    def process(self, input_text):

        start_time_eng_trans = time.time()
        # Question_eng = translator.translate_text(Question, target_lang="EN-US")
        Question_eng = self.translate_papago(input_text)
        end_time_eng_trans = time.time()

        start_time_chat = time.time()
        # prompt = f"Question: {Question_eng}, Answer:"
        output = self.llm_chain_llama({"question": Question_eng})
        end_time_chat = time.time()
        # print(output)

        start_time_kor_trans = time.time()
        result_deepl = self.translate_deepl.translate_text(output['text'], target_lang="KO").text
        end_time_kor_trans = time.time()

        eng_trans_duration = end_time_eng_trans - start_time_eng_trans
        chat_duration = end_time_chat - start_time_chat
        kor_trans_duration = end_time_kor_trans - start_time_kor_trans
        print("llama:\n", result_deepl)
        print(f"\nLlama2: {chat_duration+eng_trans_duration+kor_trans_duration:.3f} / Chat: {chat_duration:.3f} seconds, Eng: {eng_trans_duration:.3f}, Kor: {kor_trans_duration:.3f} \n")

        return result_deepl
