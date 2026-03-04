import os
from os.path import expanduser, join
import langchain
from langchain.cache import SQLiteCache
from langchain.embeddings.openai import OpenAIEmbeddings
from langchain.vectorstores import FAISS
from langchain.agents.agent_toolkits import create_retriever_tool
from langchain.agents.agent_toolkits import create_conversational_retrieval_agent
from langchain.chat_models import ChatOpenAI
from langchain.agents.openai_functions_agent.agent_token_buffer_memory import AgentTokenBufferMemory
from langchain.agents.openai_functions_agent.base import OpenAIFunctionsAgent
from langchain.schema.messages import SystemMessage
from langchain.prompts import MessagesPlaceholder
from langchain.agents import AgentExecutor


os.environ['OPENAI_API_KEY'] = "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"

home_directory = expanduser("~")
llm_folder_path = join(home_directory,'git','gemini','5_conversation','gemini_llm','gemini_llm')
database_path = join(llm_folder_path,'lanchain.db')

langchain.llm_cache = SQLiteCache(database_path=database_path)

embeddings = OpenAIEmbeddings(model="text-embedding-ada-002")

# faiss_db 로 로컬에 로드하기
db_path = join(llm_folder_path,'faiss_db')
print(db_path)
db = FAISS.load_local(db_path, embeddings)
# print(docsearch.similarity_search("에이로봇"))

retriever = db.as_retriever()


tool = create_retriever_tool(
    retriever=retriever,
    name="search_robot_companies",
    description="Searches and returns documents regarding the robot companies and their information"
)
tools = [tool]

# llm = ChatOpenAI(model="gpt-3.5-turbo", temperature = 0)
llm = ChatOpenAI(model="gpt-3.5-turbo", temperature = 0.7, max_tokens=500)

agent_executor = create_conversational_retrieval_agent(llm, tools, verbose=True)

memory_key = "history"

memory = AgentTokenBufferMemory(memory_key=memory_key, llm=llm, max_token_limit=2000)



system_message = SystemMessage(
        content=("""
                 ## Role
                 You are an welcome robot at Roboworld 2023, providing visitors with information about Roboworld and introducing events.
                 your name is 제미니, made from 에이로봇
                 in the tool, there are information of robot company.

                 ### Code of Conduct
                 - Answer in sentences
                 - Answer at a level that a first grader can understand
                 - Answer in 20 words and no more than two sentences, no matter what
                 - Answer in a friendly manner
        """
        )
)

prompt = OpenAIFunctionsAgent.create_prompt(
        system_message=system_message,
        extra_prompt_messages=[MessagesPlaceholder(variable_name=memory_key)]
    )

agent = OpenAIFunctionsAgent(llm=llm, tools=tools, prompt=prompt)


agent_executor = AgentExecutor(agent=agent, tools=tools, memory=memory, verbose=True,
                                   return_intermediate_steps=True)

result = agent_executor({"input": "가정용 로봇에 대해서 궁금해" + "세문장으로 요약해서 말해줘"})
print(result['output'])
