import os
import time
from os.path import expanduser, join

import langchain
from langchain.cache import SQLiteCache

from langchain.embeddings.openai import OpenAIEmbeddings
from langchain.vectorstores import FAISS
from langchain.agents.agent_toolkits import create_retriever_tool

from langchain.chat_models import ChatOpenAI
from langchain.agents.openai_functions_agent.agent_token_buffer_memory import AgentTokenBufferMemory

from langchain.schema.messages import SystemMessage
from langchain.agents.openai_functions_agent.base import OpenAIFunctionsAgent
from langchain.prompts import MessagesPlaceholder
from langchain.agents import AgentExecutor

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
resoruce_path = join(current_directory, '..', '..', 'share', 'gemini_llm', 'resource')
print(resoruce_path)
# home_directory = expanduser("~")
# llm_folder_path = join(home_directory,'git','gemini','5_conversation','gemini_llm','gemini_llm')
CACHE_PATH = join(resoruce_path,'langchain.db')
DB_PATH = join(resoruce_path,'faiss_db')
PROMPT_PATH = join(resoruce_path,'prompt_template.txt')

langchain.llm_cache = SQLiteCache(database_path=CACHE_PATH)

class LangchainGPT:
    def __init__(self):
        os.environ['OPENAI_API_KEY'] = "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"
        self.memory_key = "history"
        self.tools = self.initialize_tool()
        self.llm, self.memory = self.initialize_gpt()
        self.prompt = self.initialize_prompt()
        self.agent_executor = self.initialize_agent()
        os.system('clear')
        print("GPT initialized.")

    def initialize_tool(self):

        embeddings = OpenAIEmbeddings(model="text-embedding-ada-002")
        # faiss_db 로 로컬에 로드하기
        db = FAISS.load_local(DB_PATH, embeddings)

        retriever = db.as_retriever()

        tool = create_retriever_tool(
            retriever=retriever,
            name="search_robot_companies",
            description="Searches and returns documents regarding the robot companies and their information"
        )
        tools = [tool]

        return tools

    def initialize_gpt(self):
        llm = ChatOpenAI(model="gpt-3.5-turbo",
                         cache=True,
                         temperature = 0.8,
                         max_tokens=500,
                         model_kwargs={"frequency_penalty": 1.5},
                         )
        memory = AgentTokenBufferMemory(memory_key=self.memory_key, llm=llm, max_token_limit=2000)

        return  llm, memory

    def initialize_prompt(self):
        f = open(PROMPT_PATH, "r")
        prompt_basic = f.read()

        system_message = SystemMessage(content=prompt_basic)

        prompt = OpenAIFunctionsAgent.create_prompt(
                system_message=system_message,
                extra_prompt_messages=[MessagesPlaceholder(variable_name=self.memory_key)]
            )
        return prompt

    def initialize_agent(self):
        agent = OpenAIFunctionsAgent(llm=self.llm, tools=self.tools, prompt=self.prompt)

        agent_executor = AgentExecutor(agent=agent, tools=self.tools, memory=self.memory,
                                       verbose=False, return_intermediate_steps=True)

        return agent_executor

    def delete_memory(self):
        self.memory.clear()

    def process(self, input_text):
        start_time_chat = time.time()
        input_data = input_text + "두 문장 또는 세 문장으로 요약해서 말하여라"
        output = self.agent_executor({"input": input_data})
        end_time_chat = time.time()

        chat_duration = end_time_chat - start_time_chat
        print("GPT:\n", output['output'])
        print(f"\nGPT: {chat_duration:.3f}")

        return output['output']
