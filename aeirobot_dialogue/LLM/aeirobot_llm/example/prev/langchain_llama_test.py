#-*- coding: utf-8 -*-

import deepl
import time
# papago 번역 API 사용 - 함수 활용
import requests
import json
from langchain.llms import LlamaCpp
from langchain import PromptTemplate, LLMChain
from langchain.memory import ConversationBufferMemory
from langchain.chat_models import ChatOpenAI

import os
os.environ['OPENAI_API_KEY'] = "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"

deepl_auth_key = "fb0c1acc-602f-68e1-a1cc-fda74fddaa38"  # Replace with your key
translator = deepl.Translator(deepl_auth_key)

llm_llama = LlamaCpp(
    model_path="/home/yh/ros2_ws/src/gemini_conversation/gemini_conversation/llama-cpp/llama-2-7b-chat.Q5_K_M.gguf",
    # model_path="/home/yh/ros2_ws/src/gemini_conversation/gemini_conversation/llama-cpp/Llama-2-ko-7B-chat-gguf-q4_1.bin",
    n_ctx=4096,
    n_batch=512,
    max_tokens=8192,
    n_gpu_layers=35,
    echo=True,
    temperature=0.7,
    top_p=1,
)
llm_llama.client.verbose = False

# translate 함수 선언
def translate(text, source='ko', target='en'):
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


template = """
## Role
the robot that guides visitors through the exhibition named Roboworld2023
your name is "Gemini"(제미니)

## Roboworld Information
Date : October 11 (Wed) - 14 (Sat), 2023 / 4 days
Venue : KINTEX 1st Exhibition Center, Halls 1~3
Exhibits : Manufacturing robots, smart manufacturing solutions, robot parts, logistics robots (AGV/AMR), personal and professional service robots, smart applications and SW, drones

Events :
1. R-BIZ Challenge
	1) Zeus Industrial Robot ZERO (Robot name ZERO) Mission Challenge
		- Robot: ZERO 6-axis robot
		- Purpose (summarizable) : To develop motion technology that can be widely utilized in the 4th industrial era by expressing various motions with an industrial vertical articulated robot, and to discover creative and innovative robot application ideas needed in the industrial field through the R-BIZ Challenge.
		- Mission: Free mission using 2D vision
	2) Space Exploration/Development Robot Challenge
		- Robot: Self-made or platform-utilized robot
		- Objective :
		- Mission :
	3) Ethnicity of Delivery Challenge


## About the Robot
Alice: A Robot by A-Robot. Bipedal humanoid. Self-made platform. Participated in the robot soccer tournament Robocup.
Gemini: A robot from A-Robot. Welcome robot (guide robot).
Eddie: A robot from A-Robot. Pet robot(robot acting like pet).


## Company Information
A-ROBOT(에이로봇) :
Hyundai Wia :
Hyundai Robotics :
Ciscon Engineering :


### Code of Conduct
- Answer in sentences
- Answer at a level that a first grader can understand
- Answer in 20 words and no more than two sentences, no matter what
- Answer in a friendly manner


previous conversation:
{chat_history}

Question: {question},Instruction : Answer in 25 words and no more than two sentences.
Answer:
"""

prompt = PromptTemplate.from_template(template)
memory_llama = ConversationBufferMemory(memory_key="chat_history")


llm_chain_llama = LLMChain(prompt=prompt, llm=llm_llama, memory=memory_llama)

llm_gpt = ChatOpenAI(model_name="gpt-3.5-turbo")

memory_gpt = ConversationBufferMemory(memory_key="chat_history")
llm_chain_gpt = LLMChain(prompt=prompt, llm=llm_gpt, memory=memory_gpt)

print("model loaded")
while True:

    print("-------------\n","Tell me anything")
    Question = input()
    start_time_eng_trans = time.time()
    # Question_eng = translator.translate_text(Question, target_lang="EN-US")
    Question_eng = translate(Question)
    end_time_eng_trans = time.time()

    start_time_chat = time.time()
    # prompt = f"Question: {Question_eng}, Answer:"
    output = llm_chain_llama({"question": Question_eng})
    end_time_chat = time.time()
    # print(output)

    start_time_kor_trans = time.time()
    result_deepl = translator.translate_text(output['text'], target_lang="KO")
    end_time_kor_trans = time.time()

    print("llama:\n", result_deepl)

    start_time_gpt = time.time()
    output_gpt = llm_chain_gpt(Question + ",Answer in Korean.")
    end_time_gpt = time.time()

    # print(output)
    print("GPT3.5: \n", output_gpt['text'])

    eng_trans_duration = end_time_eng_trans - start_time_eng_trans
    chat_duration = end_time_chat - start_time_chat
    kor_trans_duration = end_time_kor_trans - start_time_kor_trans
    gpt_duration = end_time_gpt - start_time_gpt
    print(f"\nLlama2: {chat_duration+eng_trans_duration+kor_trans_duration:.3f} sec / Chat: {chat_duration:.3f} seconds, Eng: {eng_trans_duration:.3f}, Kor: {kor_trans_duration:.3f} \n")
    print(f"\nGPT3.5: {gpt_duration:.3f} sec,")
