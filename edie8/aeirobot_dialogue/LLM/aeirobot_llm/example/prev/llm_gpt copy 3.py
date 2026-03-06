import os
import sys
import time
import json
from typing import Sequence
from typing_extensions import Annotated, TypedDict

from dotenv import load_dotenv
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.messages import AIMessage, HumanMessage, BaseMessage, trim_messages, AIMessageChunk
from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.outputs import LLMResult

from langgraph.graph import START, MessagesState, StateGraph
from langgraph.graph.message import add_messages
from langgraph.checkpoint.memory import MemorySaver

# Path configurations
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
sys.path.append(current_directory)

resource_path = os.path.join(current_directory, '..', '..', 'share', 'gemini_llm', 'resource')
CACHE_PATH = os.path.join(resource_path, 'langchain.db')
prompt_file_path = os.path.join(current_directory, '..', 'resource', 'prompt_template.txt')
env_path = os.path.join(current_directory, '..', '.env')


from langchain_community.tools.tavily_search import TavilySearchResults
from langgraph.prebuilt import create_react_agent

# Type definitions
class State(TypedDict):
    messages: Annotated[Sequence[BaseMessage], add_messages]
    language: str

# Custom callback handler
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

# Main LangchainGPT class
class LangchainGPT:
    def __init__(self, publisher):
        self.publisher_ = publisher  # 여기서 publisher_를 저장

        # Load environment variables
        current_file_path = os.path.abspath(__file__)
        current_directory = os.path.dirname(current_file_path)
        env_path = os.path.join(current_directory, '..', '.env')
        load_dotenv(env_path)

        search = TavilySearchResults(max_results=2)
        self.tools = [search]

        self.llm_gpt = self.init_gpt()
        # self.llm_gpt = self.init_gpt_tools()

        self.prompt = self.initialize_prompt()

        self.memory = MemorySaver()

        self.agent_executor = create_react_agent(self.llm_gpt, self.tools, checkpointer=self.memory)

        # os.system('clear')
        print("GPT initialized.")


    def init_gpt(self):
        llm = ChatOpenAI(model="gpt-4o-mini")

        return llm

    def init_gpt_tools(self):
        llm = ChatOpenAI(model="gpt-4o-mini")

        llm_tools = llm.bind_tools(self.tools)

        return llm_tools

    def initialize_prompt(self):
        with open(prompt_file_path, 'r') as file:
            prompt_basic = file.read()

        prompt = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    "You are a helpful assistant. Answer all questions to the best of your ability in {language}. 온도 단위는 섭씨(℃)이다.",
                ),
                (
                    "system",
                    prompt_basic,
                ),
                MessagesPlaceholder(variable_name="messages"),
            ]
        )

        return prompt

    def call_model(self, state: State):
        chain = self.prompt | self.llm_gpt
        trimmed_messages = self.trimmer.invoke(state["messages"])
        response = chain.invoke(
            {"messages": trimmed_messages, "language": state["language"]},
        )
        return {"messages": [response]}


    def process(self, input_text):
        response_string = String()
        response_string.data = str("<BOS>")
        self.publisher_.publish(response_string)


        config = {"configurable": {"thread_id": "abc111"}, "callbacks": [MyCustomHandler(self.publisher_)]}
        query = input_text
        language = "Korean"

        result = ""
        input_messages = [HumanMessage(query)]
        # for chunk in self.agent_executor.stream(
        #     {"messages": [HumanMessage(content="hi im bob! and i live in sf")]}, config
        # ):
        #     print(chunk)
        #     print("----")

        for chunk in self.agent_executor.stream(
            {"messages": input_messages, "language": language},
            config,
            stream_mode="messages",
        ):
            print(chunk)
            print("----")
            if isinstance(chunk, AIMessageChunk):  # Filter to just model responses
                result += chunk.content

        return result

    def delete_memory(self):
        self.memory_token.clear()
        self.memory_summary.clear()

# ROS2 Node
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

# Main function
def main(args=None):
    rclpy.init(args=args)
    node = LlmGPTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
