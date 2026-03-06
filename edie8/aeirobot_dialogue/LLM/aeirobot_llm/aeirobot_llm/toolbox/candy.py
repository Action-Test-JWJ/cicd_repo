import os
import sys
import yaml
import requests
from datetime import datetime, timedelta
from typing import List, Tuple, Union


from langchain.agents import tool



@tool
def give_candy(flavor: str, color: str = "") -> str:
    "Define a candy color based on flavor and color and publish the color in red, yellow, blue. Flavor : strawberry, lemon, pineapple. Strawberry : red, lemon : yellow, pineapple : blue"
    flavor = flavor.lower()
    color = color.lower()

    if not color:
        if flavor in ["strawberry", "딸기", "딸기맛"]:
            color = "red"
        elif flavor in ["lemon", "레몬", "레몬맛"]:
            color = "yellow"
        elif flavor in ["pineapple", "파인애플", "파인애플맛"]:
            color = "blue"
        else:
            return "Invalid flavor. Please choose strawberry, lemon, or pineapple."

    if color not in ["red", "yellow", "blue"]:
        return "Invalid color. Please choose red, yellow, or blue."

    # msg = String()
    # msg.data = color
    # self.publisher_tools.publish(msg)

    return color