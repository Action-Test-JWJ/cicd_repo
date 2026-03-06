from langchain_community.tools import TavilySearchResults
from langchain_core.tools import tool


import os
import sys
from dotenv import load_dotenv

load_dotenv()

@tool
def search_web(query: str) -> str:
    """Search the web for the given query and return summarized results."""
    search_tool = TavilySearchResults()
    results = search_tool.run(query)
    return results