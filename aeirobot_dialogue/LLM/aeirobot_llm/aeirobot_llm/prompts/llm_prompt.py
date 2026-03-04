import os
import pandas as pd

ros_ws = os.environ.get('ROS_WS')

files = [
    f"{ros_ws}/src/aeirobot_dialogue/aeirobot_llm/example/exhibition_b.csv",
    f"{ros_ws}/src/aeirobot_dialogue/aeirobot_llm/example/exhibition_g.csv",
    f"{ros_ws}/src/aeirobot_dialogue/aeirobot_llm/example/exhibition_o.csv",
    f"{ros_ws}/src/aeirobot_dialogue/aeirobot_llm/example/exhibition_r.csv",
]

# 각 파일별 데이터프레임 생성 및 첫 번째 열 추출
df_a = pd.read_csv(files[0])
df_b = pd.read_csv(files[1])
df_c = pd.read_csv(files[2])
df_d = pd.read_csv(files[3])

# 각 파일의 첫 번째 열을 리스트로 변환하여 변수에 저장
topics_b = df_a.iloc[:, 0].tolist()
topics_g= df_b.iloc[:, 0].tolist()
topics_o = df_c.iloc[:, 0].tolist()
topics_r = df_d.iloc[:, 0].tolist()


topic_prompts = [
    (
        "system",
        f"Topics: {topics_b}, {topics_g}, {topics_o}, {topics_r}",
    ),  
]