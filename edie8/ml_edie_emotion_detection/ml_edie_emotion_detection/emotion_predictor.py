import os
import warnings
import numpy as np
import pickle

from ml_edie_emotion_detection.emotional_mesh_detection import EmotionalMeshDetection
from ament_index_python.packages import get_package_share_directory

class Emotion:
    def __init__(self):
        # Predicted emotion and percentage
        self.label = None
        self.class_name = None
        self.probability = None
        
        # Bounding box in source frame
        # Format: [cx_min, cy_min, cx_max, cy_max]
        self.bounding_box = np.zeros(4)

class EmotionPredictor:
    def __init__(self, model_algorithm="MLP", max_num_faces=1,
                 neutral_min_conf: float = 0.60,
                 neutral_margin_th: float = 0.15):
        # Emotional mesh to extract angles data
        self.emotional_mesh_detection = EmotionalMeshDetection(max_num_faces=max_num_faces)

        # Model and PCA to predict
        self.model = None
        self.pca = None
        self.model_algorithm = model_algorithm
        self.__initialize_model_and_pca(model_algorithm)

        # Neutral 판정 임계치(확률과 top1-top2 마진)
        # - 확률은 0~1 스케일, 마진은 확률 차(0~1)
        self.neutral_min_conf = max(0.0, min(1.0, float(neutral_min_conf)))
        self.neutral_margin_th = max(0.0, min(1.0, float(neutral_margin_th)))

        # Angry↔Surprise 후처리 휴리스틱 스위치/임계치
        # - 값들은 경험 기반 기본값이며, 환경에 따라 조정 가능
        self._use_heuristics = True
        # Surprise 특징: 눈이 큼, 눈썹이 올라감, 입이 큼
        self._S_EYE_LOW = 0.20      # 이보다 작으면 Surprise로 보기 어려움
        self._S_BROW_LOW = 0.075    # 이보다 작으면 Surprise로 보기 어려움
        self._S_MOUTH_HIGH = 0.075  # 입이 크게 열렸다고 판단
        # Anger 특징: 눈이 작고(찌푸림), 눈썹이 내려감
        self._A_EYE_HIGH = 0.23     # 이보다 크면 Anger보단 Surprise 경향
        self._A_BROW_HIGH = 0.09    # 이보다 크면 Anger보단 Surprise 경향

    def predict(self, frame):
        emotions = []
        self.emotional_mesh_detection.process(frame)
        emotional_meshes = self.emotional_mesh_detection.get_emotional_meshes()

        # 가장 큰 얼굴 1개만 선택하여 처리
        if len(emotional_meshes) == 0:
            return emotions
        if len(emotional_meshes) > 1:
            def area(bb):
                return max(0, (bb[2] - bb[0])) * max(0, (bb[3] - bb[1]))
            e_mesh = max(emotional_meshes, key=lambda m: area(m.bounding_box))
            selected_meshes = [e_mesh]
        else:
            selected_meshes = emotional_meshes

        for e_mesh in selected_meshes:
            # Prediction with angles of emotional mesh
            angles = e_mesh.angles
            angles = self.pca.transform(angles)
            if self.model_algorithm == "SVM":
                emotion_index = self.model.predict(angles)
                predicted_emotion = self.__evaluate_prediction(emotion_index)
                probability = 0
            else:
                probabilities = self.model.predict_proba(angles)
                probs = probabilities[0]
                # top1/top2 및 마진 계산
                order = np.argsort(probs)[::-1]
                top1 = int(order[0])
                top2 = int(order[1]) if probs.size > 1 else int(order[0])
                p1 = float(probs[top1])
                p2 = float(probs[top2])
                margin = p1 - p2

                emotion_index = top1
                predicted_emotion = self.__evaluate_prediction_proba(emotion_index)
                probability = p1

            # Save data in object Face()
            emotions.append(Emotion())
            # Neutral 규칙: 비 SVM일 때 확률/마진이 낮으면 Neutral 할당
            if self.model_algorithm != "SVM":
                try:
                    if probability < self.neutral_min_conf or margin < self.neutral_margin_th:
                        predicted_label = "Neutral"
                    else:
                        predicted_label = predicted_emotion
                except Exception:
                    predicted_label = predicted_emotion
            else:
                predicted_label = predicted_emotion

            # Angry↔Surprise 오분류 보정 휴리스틱
            # - 입이 크지만(놀람 유사) 눈이 작고 눈썹이 낮으면 Anger로 교정
            # - Anger로 나왔는데 눈이 크고 눈썹이 높으면 Surprise로 교정
            if self._use_heuristics:
                try:
                    metrics = e_mesh.compute_normalized_metrics()
                    eye_open = float(metrics.get('eye_open', 0.0))
                    brow_gap = float(metrics.get('brow_gap', 0.0))
                    mouth_open = float(metrics.get('mouth_open', 0.0))

                    if predicted_label == "Surprise":
                        # 강한 교정: 눈이 작고 눈썹이 낮으면 입 크기와 무관하게 Anger
                        if eye_open < self._S_EYE_LOW or brow_gap < self._S_BROW_LOW:
                            predicted_label = "Anger"
                    # elif predicted_label == "Anger":
                    #     if eye_open > self._A_EYE_HIGH and brow_gap > self._A_BROW_HIGH:
                    #         predicted_label = "Surprise"
                except Exception:
                    pass

            emotions[-1].class_name = predicted_label
            emotions[-1].probability = round(probability*100, 2)
            if self.model_algorithm == "SVM":
                emotions[-1].label = emotions[-1].class_name
            else:
                emotions[-1].label = "{}: {} %".format(emotions[-1].class_name, emotions[-1].probability)
            emotions[-1].bounding_box = e_mesh.bounding_box
        return emotions

    # Private methods ----------------------------------------------------------------------
    def __initialize_model_and_pca(self, model_algorithm):
        share_dir = get_package_share_directory("ml_edie_emotion_detection")

        def try_load(alg: str):
            path = os.path.join(share_dir, "models", f"model_{alg}.pkl")
            if not os.path.exists(path):
                raise FileNotFoundError(path)
            with open(path, 'rb') as f:
                return pickle.load(f)

        # 우선순위: 요청 알고리즘 → MLP → SVM → KNN
        order = [str(model_algorithm).upper()]
        for cand in ["MLP", "SVM", "KNN"]:
            if cand not in order:
                order.append(cand)

        last_err = None
        for alg in order:
            try:
                loaded = try_load(alg)
                self.model = loaded['model']
                self.pca = loaded['pca_fit']
                if alg != str(model_algorithm).upper():
                    warnings.warn(f"emotion model '{model_algorithm}' load failed; fallback to '{alg}'")
                self.model_algorithm = alg
                return
            except Exception as e:
                last_err = e
                continue

        raise RuntimeError(f"failed to load any emotion model({order}): {last_err}")

    def __evaluate_prediction_proba(self, emotion_index):
        if emotion_index == 0:
            return "Anger"
        elif emotion_index == 1:
            return "Happiness"
        elif emotion_index == 2:
            return "Sadness"
        elif emotion_index == 3:
            return "Surprise"
        return None

    def __evaluate_prediction(self, emotion_index):
        if emotion_index == 1:
            return "Anger"
        elif emotion_index == 5:
            return "Happiness"
        elif emotion_index == 6:
            return "Sadness"
        elif emotion_index == 7:
            return "Surprise"
        return None