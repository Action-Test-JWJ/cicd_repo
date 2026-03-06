import random
from langchain_ollama import ChatOllama

from langchain_core.prompts import ChatPromptTemplate
from langchain_core.messages import AIMessage, HumanMessage  
from langchain_core.runnables import Runnable, RunnableConfig
from langchain.prompts import MessagesPlaceholder

from langchain.agents.format_scratchpad.openai_tools import format_to_openai_tool_messages
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain.agents import create_tool_calling_agent, AgentExecutor

from typing import Any, AsyncIterable, List, Dict, Literal, Optional, Union

from aeirobot_llm.toolbox import ToolBox
from aeirobot_llm.prompts.persona_prompt import system_prompts, get_prompts

class ActionExecutor:
    def __init__(
        self, 
        llm,
        prompt_packages: Optional[list] = None,
        add_prompt: Optional[list] = None,
        tools: Optional[list] = None,
        tool_packages: Optional[list] = None,
        session_id: str = "default", 
        accumulate_chat_history: bool = True,
        memory_chat_history: int = 20,
        verbose: bool = True,
    ):
        self.chat_history = []  
        self.memory_key = "chat_history"
        self.scratchpad = "agent_scratchpad"
        self.accumulate_chat_history = accumulate_chat_history
        self.memory_chat_history = memory_chat_history
        self.session_id = session_id
        self.verbose = verbose                                                                        # 마지막 응답을 저장할 변수
        
        self.llm = llm
        self.prompts = self.set_prompts(prompt_packages=prompt_packages, add_prompt=add_prompt)              # 프롬프트 템플릿 생성
        self.toolbox = self.get_tools(packages=tool_packages, tools=tools)                                   # 도구상자 세팅
        self.tools = self.toolbox.get_tools()
        self.llm_with_tools = self.llm.bind_tools(self.tools)                                                # LLM과 도구상자의 도구 결합
        self.agent = self.get_agent()                                                                        # 에이전트 생성   
        self.executor = self.get_executor(verbose=self.verbose)                                              # 에이전트 실행기 생성

        
        
    # 1. 내가 원하는 도구들을 골라 도구상자에 담는 함수
    def get_tools(
        self,
        packages: Optional[list],
        tools: Optional[list],
    ) -> ToolBox:
        """Create a ROSA tools object with the specified ROS version, tools, packages, and blacklist."""
        rosa_tools = ToolBox()
        if tools:
            rosa_tools.add_tools(tools)
        if packages:
            rosa_tools.add_packages(packages)
        
        return rosa_tools
    
    # 2. agent 프롬프트 생성 함수
    def set_prompts(self, prompt_packages, add_prompt=None) -> ChatPromptTemplate:
        """Create a chat prompt template from the system prompts and robot-specific prompts."""
        # Start with default system prompts
        prompts = system_prompts # action_executor_prompts
        embodied_prompt = get_prompts(prompt_packages)
        
        prompts.append(embodied_prompt.as_message())
        
        if add_prompt:
            prompts += add_prompt

        template = ChatPromptTemplate.from_messages(
            prompts
            + 
            [
                MessagesPlaceholder(variable_name=self.memory_key),
                ("user", "{input}"),
                MessagesPlaceholder(variable_name=self.scratchpad),
            ]
        )
        return template
    
    # 3. 에이전트 생성 함수
    def get_agent(self):
        """Create and return an agent for processing user inputs and generating responses."""
        # agent = create_tool_calling_agent(self.__llm, self.__tools, self.__prompts)

        agent = (
            {
                "input": lambda x: x["input"],
                "agent_scratchpad": lambda x: format_to_openai_tool_messages(x["intermediate_steps"]),
                "chat_history": lambda x: x.get("chat_history", []),
            }
            | self.prompts
            | self.llm_with_tools
            | OpenAIToolsAgentOutputParser()
        )
        
        return agent
    
    # 4. 에이전트 실행기 생성 함수
    def get_executor(self, verbose: bool) -> AgentExecutor:
        """Create and return an executor for processing user inputs and generating responses."""
        executor = AgentExecutor(
            agent=self.agent,
            tools=self.tools,
            verbose=verbose,
            return_intermediate_steps=True,
            handle_parsing_errors=True,
            max_iterations=15,
            # max_execution_time=30,
        )

        return executor
    
    
    # 5. 채팅 기록 저장 함수
    def record_chat_history(self, query: str, response: str):
        """Record the chat history if accumulation is enabled."""
        if self.accumulate_chat_history:
            self.chat_history.extend(
                [HumanMessage(content=query), AIMessage(content=response)]
            )
        if len(self.chat_history) > self.memory_chat_history:
            self.chat_history = self.chat_history[-self.memory_chat_history : ]
    
    def delete_memory(self):
        """Delete the chat history memory."""
        self.chat_history = []
    
    # 6. 에이전트 간단한 채팅 함수
    def chat(self, query: str, config: Optional[RunnableConfig]) -> str:
        
        if self.memory_chat_history > 0:
            result = self.executor.invoke(
                {"input": query, "chat_history": self.chat_history},
                config=config
                )   
        
            output_text = result["output"]
            self.record_chat_history(query, output_text)
        else:
            result = self.executor.invoke(
                    {"input": query},
                    config=config,
                )

            output_text = result["output"]

        return output_text
    
    # 7. 에이전트 모니터링을 위한 결과 return 및 대화 내역을 저장 및 행동 관찰하는 에이전트 생성 함수
    def invoke(self, query: str, config: Optional[RunnableConfig]) -> str:
        
        result = self.executor.invoke(
            {"input": query, "chat_history": self.chat_history},
            config=config
            )   
        
        output_text = result["output"]
        intermediate_steps = result.get("intermediate_steps", [])
        structured_steps = [
            {
                "log": getattr(action, "log", ""),
                "observation": observation
            }
            for action, observation in intermediate_steps
        ]
        
        self.record_chat_history(query, output_text)
        
        return {
            "output": output_text,
            "agent_scratchpad": structured_steps
        }
        
    # 8. 어떤 도구를 사용하는지 모니터링 하면서 채팅하는 함수
    def invoke_iter(self, input: str, config: Optional[RunnableConfig]) -> str:
        structured_steps = []
        for step in self.executor.iter({"input": input, "chat_history": self.chat_history}, config=config):
    
            if output := step.get("intermediate_step"):
                action, value = output[0]
                print(f"\n다음과 같은 도구를 사용하였습니다:{action.log}")
                structured_step = {
                        "log": action.log,
                        "observation": value
                    }
                structured_steps.append(structured_step)

        output_text = step["output"]
        self.record_chat_history(input, output_text)
        
        return {
            "output": output_text,
            "agent_scratchpad": structured_steps
        }
        
        

# === 메인 실행부 ===
if __name__ == "__main__":
    

    verbose = True 
    
    chat_ID = str(random.randrange(1,100)) # input("ID를 입력해주세요: ")
    
    model_name = "qwen3:1.7b" # qwen3:4b
    temperature = 0.6
    
    llm = ChatOllama(
                model=model_name, 
                temperature=temperature,
                # base_url = base_url
                )
    
    executor = ActionExecutor(llm=llm, session_id=chat_ID, verbose=verbose)

    
    
    cur_state = {
        'THOUGHT': '주변을 더 꼼꼼히 살펴볼 필요가 있어 보입니다. 다양한 객체와 장소들이 눈에 들어오네요.',
        'EMOTION': '흥미',
        'GOAL': '단순 이동하기 - 주변을 더 꼼꼼히 살펴보기',
        'PLAN': ['x축 방향으로 1m 이동', 'y축 방향으로 1m 이동'],
        'STEP': ['x축 방향으로 1m 이동']
    }
    
    state_input = "\n".join([f"{k}: {v}" for k, v in cur_state.items()])
    
    final_answer = executor.invoke(state_input)
    
    # while True:
    #     # 새 텍스트 입력 전에 이전에 공유했던 카메라 피드 창이 열려있다면 닫습니다.
        
    #     # stream 옵션에 따라 호출
    #     final_answer = executor.invoke(thought, goal, plan, step, thinking)
        
    #     print(final_answer)
        