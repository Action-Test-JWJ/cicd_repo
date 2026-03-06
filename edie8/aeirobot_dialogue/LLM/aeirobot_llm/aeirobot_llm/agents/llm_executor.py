from typing import Optional

from langchain_core.prompts import ChatPromptTemplate
from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.runnables import RunnableConfig
from langchain.prompts import MessagesPlaceholder

from aeirobot_llm.prompts.persona_prompt import system_prompts, get_prompts


class QAExecutor:
    """도구 없이 순수 LLM QA 방식으로 작동하는 Executor"""
    
    def __init__(
        self, 
        llm,
        prompt_packages: Optional[list] = None,
        add_prompt: Optional[list] = None,
        session_id: str = "default", 
        accumulate_chat_history: bool = True,
        memory_chat_history: int = 20,
        verbose: bool = True,
    ):
        self.chat_history = []  
        self.memory_key = "chat_history"
        self.accumulate_chat_history = accumulate_chat_history
        self.memory_chat_history = memory_chat_history
        self.session_id = session_id
        self.verbose = verbose
        
        self.llm = llm
        self.prompts = self.set_prompts(prompt_packages=prompt_packages, add_prompt=add_prompt)
        self.chain = self.prompts | self.llm  # 도구 없는 단순 체인

    # 프롬프트 생성 함수
    def set_prompts(self, prompt_packages, add_prompt=None) -> ChatPromptTemplate:
        """Create a chat prompt template from the system prompts and robot-specific prompts."""
        # Start with default system prompts
        prompts = system_prompts.copy()
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
            ]
        )
        return template
    
    # 채팅 기록 저장 함수
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
    
    # 채팅 함수
    def chat(self, query: str, config: Optional[RunnableConfig] = None) -> str:
        
        if self.memory_chat_history > 0:
            result = self.chain.invoke(
                {"input": query, "chat_history": self.chat_history},
                config=config
            )
        else:
            result = self.chain.invoke(
                {"input": query},
                config=config,
            )

        output_text = result.content
        self.record_chat_history(query, output_text)

        return output_text
