import os
import sys

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
# from langchain_google_genai import ChatGoogleGenerativeAI

from dotenv import load_dotenv

# Path configurations
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
sys.path.append(current_directory)

env_path = os.path.join(current_directory, '..', '..', '.env')
load_dotenv(env_path)


def get_llm(model_name="qwen2.5:0.5b", num_thread=4, temperature=0.3):
    """
    LLM Model

    Args:
        model (str): LLM 모델 이름 // "gpt-3.5-turbo", "gpt-4o", "gpt-4o-mini", "gpt-4o-mini"

    Returns:
        llm: LLM instance
    """
    
    # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
    
    if model_name.startswith("gpt"):
        local=False
    else:
        local=True
    
    if local:
        llm = ChatOllama(
            model=model_name, 
            temperature=temperature,
            num_gpu=-1,
            num_thread=num_thread,
            streaming=True
            )
    else:
        llm = ChatOpenAI(
            model_name=model_name, 
            temperature=temperature,
            streaming=True 
            )
        # llm = ChatGoogleGenerativeAI(model="gemini-2.0-flash", temperature=temperature)
        # gemini-2.5-pro-preview-03-25, gemini-2.5-flash-preview-04-17
    
    return llm


def get_openai_api(model_name="gpt-4o-mini",temperature=0.3):
    """
    OpenAI LLM Model

    Args:
        model (str): LLM 모델 이름 // "gpt-3.5-turbo", "gpt-4o", "gpt-4o-mini", "gpt-4o-mini"

    Returns:
        llm: OpenAI LLM instance
    """
    llm = ChatOpenAI(model_name=model_name, temperature=temperature)

    return llm

def get_aeirobot_api(model_name="gpt-4o-mini",temperature=0.3):
    """
    OpenAI LLM Model

    Args:
        model (str): LLM 모델 이름 // "gpt-3.5-turbo", "gpt-4o", "gpt-4o-mini", "gpt-4o-mini"

    Returns:
        llm: OpenAI LLM instance
    """
    base_url = "http://localhost:11500" 
    # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
    llm = ChatOllama(
        model=model_name, 
        temperature=temperature,
        base_url = base_url,
        )

    return llm

def get_ollama_local(model_name="qwen2.5:0.5b",temperature=0.3):
    # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)

    # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
    llm = ChatOllama(
        model=model_name, 
        temperature=temperature,
        num_gpu=-1,
        num_thread=4
        )
    return llm


# def get_llm_api(model_name="gpt-3.5-turbo",temperature=0.3):
#     # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)
    
#     llm = ChatOpenAI(model_name=model_name, temperature=temperature)

#     return llm

# def get_llm_server(model_name="qwen3:14b",temperature=0.3):
#     # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)

#     base_url = "http://localhost:11500" 
#     # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
#     llm = ChatOllama(
#         model=model_name, 
#         temperature=temperature,
#         base_url = base_url,
#         )
#     return llm

# def get_llm_local(model_name="qwen2.5:0.5b",temperature=0.3):
#     # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)

#     # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
#     llm = ChatOllama(
#         model=model_name, 
#         temperature=temperature,
#         num_gpu=-1,
#         num_thread=4
#         )
#     return llm


# def get_llm(local=False, model_name="qwen2.5:0.5b", num_thread=4, temperature=0.3):
#     # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)

#     # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
    
#     if local:
#         llm = ChatOllama(
#             model=model_name, 
#             temperature=temperature,
#             num_gpu=-1,
#             num_thread=4
#             )
#     else:
#         llm = ChatOpenAI(model_name=model_name, temperature=temperature)
#         # llm = ChatGoogleGenerativeAI(model="gemini-2.0-flash", temperature=temperature)
#         # gemini-2.5-pro-preview-03-25, gemini-2.5-flash-preview-04-17
    
#     return llm

# def get_env_variable(var_name: str) -> str:
#     value = os.getenv(var_name)
#     if value is None:
#         raise ValueError(f"Environment variable {var_name} is not set.")
#     return value
