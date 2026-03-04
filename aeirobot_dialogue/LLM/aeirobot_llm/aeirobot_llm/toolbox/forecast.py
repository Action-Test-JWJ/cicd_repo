import os
import sys
import yaml
import requests
from datetime import datetime, timedelta
from typing import List, Tuple, Union


from langchain.agents import tool


# Path configurations
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
sys.path.append(current_directory)

city_names_path = os.path.join(current_directory, '..', '..', 'resource', 'city_names.yaml')


# YAML 파일에서 도시 이름 매핑 로드
def load_city_names():
    with open(city_names_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)

city_name_map = load_city_names()

@tool
def get_current_datetime() -> str:
    "Get the current date, time, and day of the week from the system."
    current_datetime = datetime.now()
    weekdays = ["월요일", "화요일", "수요일", "목요일", "금요일", "토요일", "일요일"]
    weekday = weekdays[current_datetime.weekday()]
    return current_datetime.strftime(f"%Y년 %m월 %d일 {weekday} %H시 %M분 %S초")

@tool
def get_current_weather(city: str) -> str:
    "Get the current weather for a given city or country."
    # 입력된 도시 름이 한글인 경우 영어로 변환
    city_eng = city_name_map.get(city, city)  # 매핑이 없으면 원래 이름 사용

    api_key = "4159bea3571d44a2955f1628c3eb28b9"
    base_url = "http://api.openweathermap.org/data/2.5/weather"
    params = {
        "q": city_eng,
        "appid": api_key,
        "units": "metric",
        # "lang": "kr"  # 한국어로 설정
    }

    try:
        print(f"날씨 정보 요청 시작: {city} (영문: {city_eng})")
        response = requests.get(base_url, params=params)
        response.raise_for_status()
        print(f"API 응답 받음: 상태 코드 {response.status_code}")
        data = response.json()
        print("JSON 데이터 파싱 완료")

        weather_description = data['weather'][0]['description']
        temperature = data['main']['temp']
        humidity = data['main']['humidity']
        wind_speed = data['wind']['speed']
        print(f"날씨 데이터 추출: {weather_description}, {temperature}°C, {humidity}%, {wind_speed}m/s")

        weather_info = (
            f"{city}의 현재 날씨:\n"
            f"날씨: {weather_description}\n"
            f"온도: {temperature}°C\n"
            f"습도: {humidity}%\n"
            # f"풍속: {wind_speed}m/s"
        )
        print("날씨 정보 문자열 생성 완료")
        return weather_info
    except requests.RequestException as e:
        print(f"날씨 정보 요청 실패: {str(e)}")
        return f"날씨 정보를 가져오는 데 실패했습니다: {str(e)}"

@tool
def get_forecast(city: str, days: int = 1) -> str:
    "Get the weather forecast for a given city for a specific day in the future and summarize it to one sentence."

    """
    Get weather forecast for a given city for a specific day in the future.

    Args:
    city (str): The name of the city.
    days (int): Number of days in the future (1 for tomorrow, 2 for day after tomorrow, etc.). Default is 1.

    Returns:
    str: Weather forecast information for the specified day.
    """
    city_eng = city_name_map.get(city, city)  # 한글 도시 이름을 영어로 변환

    api_key = "4159bea3571d44a2955f1628c3eb28b9"
    base_url = "http://api.openweathermap.org/data/2.5/forecast"

    params = {
        "q": city_eng,
        "appid": api_key,
        "units": "metric",
        # "lang": "kr"  # 한국어로 설정
    }

    try:
        print(f"API 요청 시작: {base_url}")
        response = requests.get(base_url, params=params)
        response.raise_for_status()
        print(f"API 응답 받음: 상태 코드 {response.status_code}")
        data = response.json()
        print("JSON 데이터 파싱 완료")

        target_date = datetime.now().date() + timedelta(days=days)
        forecast = f"{city}의 {days}일 후 ({target_date.strftime('%Y-%m-%d')}) 날씨 예보:\n\n"
        print(f"날씨 예보 생성 시작: {city}, 목표 날짜: {target_date}")

        day_forecast = []

        for item in data['list']:
            dt = datetime.fromtimestamp(item['dt'])
            if dt.date() == target_date:
                temp = item['main']['temp']
                feels_like = item['main']['feels_like']
                description = item['weather'][0]['description']
                humidity = item['main']['humidity']
                wind_speed = item['wind']['speed']

                forecast_entry = (f"{dt.strftime('%H:%M')} - "
                                    f"기온: {temp:.1f}°C (체감 {feels_like:.1f}°C), "
                                    f"날씨: {description}, "
                                    f"습도: {humidity}%, "
                                    f"풍속: {wind_speed}m/s")
                day_forecast.append(forecast_entry)

        if day_forecast:
            forecast += "\n".join(day_forecast)
        else:
            forecast += "해당 날짜의 예보 정보를 찾을 수 없습니다."

        print("날씨 예보 생성 완료")
        print(forecast)
        return forecast

    except requests.RequestException as e:
        print(f"날씨 정보 요청 실패: {str(e)}")
        return f"날씨 정보를 가져오는 데 실패했습니다: {str(e)}"
