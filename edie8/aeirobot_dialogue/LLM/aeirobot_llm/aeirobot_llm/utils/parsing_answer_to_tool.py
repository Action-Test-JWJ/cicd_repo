import re

def parse_emotion_from_answer(answer: str) -> str:
    """
    '[Emotion]: happiness'와 같은 문자열에서 감정명(happiness)만 추출하여 반환합니다.
    감정명이 없거나 형식이 맞지 않으면 빈 문자열을 반환합니다.
    """
    if not isinstance(answer, str):
        return ""
    match = re.search(r"\[Emotion\]:\s*([a-zA-Z가-힣]+)", answer)
    if match:
        return match.group(1).strip()
    return ""


