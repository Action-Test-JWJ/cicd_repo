import getpass
import os
from dotenv import load_dotenv

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
env_path = os.path.join(current_directory, '..', '.env')
load_dotenv(env_path)

from langchain_openai import ChatOpenAI

model = ChatOpenAI(model="gpt-3.5-turbo")

from langchain_core.messages import HumanMessage

model.invoke([HumanMessage(content="Hi! I'm Bob")])

