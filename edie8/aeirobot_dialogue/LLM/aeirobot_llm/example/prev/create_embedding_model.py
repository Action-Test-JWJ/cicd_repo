from langchain.document_loaders import CSVLoader
from langchain.text_splitter import CharacterTextSplitter
from langchain.embeddings.openai import OpenAIEmbeddings
from langchain.vectorstores import FAISS

import os
from os.path import join

os.environ['OPENAI_API_KEY'] = "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"

current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
resoruce_path = join(current_directory, '..', 'resource')
DATA_PATH = join(resoruce_path,'roboworld2023_data.csv')

loader = CSVLoader(DATA_PATH, encoding='cp949')
documents = loader.load()

# 데이터를 불러와서 텍스트를 일정한 수로 나누고 구분자로 연결하는 작업
text_splitter = CharacterTextSplitter(
	  chunk_size=1000,
    chunk_overlap=0,
    separator="\n"
    )
texts = text_splitter.split_documents(documents)

print(len(texts))

embeddings = OpenAIEmbeddings(model="text-embedding-ada-002")

# # 임베딩 모델 로드
# index = FAISS.from_documents(
# 	documents=texts,
# 	embedding=embeddings,
# 	)

# # faiss_db 로 로컬에 저장하기
# index.save_local("faiss_db")

# faiss_db 로 로컬에 로드하기
docsearch = FAISS.load_local("faiss_db", embeddings)
# print(docsearch.similarity_search("에이로봇"))
