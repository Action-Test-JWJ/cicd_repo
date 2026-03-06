# retriever.py
import os
import sys
from langchain_core.tools import tool
from langchain_community.vectorstores import Chroma
from langchain_openai import OpenAIEmbeddings


ros_ws = os.environ.get('ROS_WS')

DB_PATH = f"{ros_ws}/src/aeirobot_dialogue/aeirobot_llm/example/chroma_db"

vectorstore = Chroma(
    persist_directory=DB_PATH,
    embedding_function=OpenAIEmbeddings(),
    collection_name="rag-chroma",
)

@tool
def retrieve_science_center_knowledge(query: str):
    """Search and return information with similarity scores."""

    # similarity_search_with_score 사용 (k=4)
    retrieved_docs_with_scores = vectorstore.similarity_search_with_score(query, k=4)
    # 내용과 점수를 함께 직렬화
    serialized = "\n\n".join(
        (f"Content: {doc.page_content}\nSimilarity Score: {score:.4f}")
        for doc, score in retrieved_docs_with_scores
    )
    
    
    return serialized
