import mediapipe as mp
import cv2

from ml_edie_emotion_detection.emotional_mesh import EmotionalMesh

class EmotionalMeshDetection:
    def __init__(self, static=False, max_num_faces=1, refine=False,
                 min_det_conf=0.5, min_track_conf=0.5):
        # Initializing FaceMesh variables
        self.mp_face_mesh = mp.solutions.face_mesh
        # MediaPipe가 얼굴 전체 468개 랜드마크를 추적해 주는 "얼굴 메시" 객체 생성
        # 얼굴 메시: 굴 위에 작은 점들을 실로 쭉쭉 이어서 만든 “그물망”
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=static,
            max_num_faces=max_num_faces,
            refine_landmarks=refine,
            min_detection_confidence=min_det_conf,
            min_tracking_confidence=min_track_conf)

        # Detected faces (list of EmotionalMesh())
        self.emotional_meshes = []
    
    def process(self, frame):
        self.emotional_meshes = []
        results = self.face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                self.emotional_meshes.append(EmotionalMesh(frame.shape))
                self.emotional_meshes[-1].update_landmarks(face_landmarks)

    def get_emotional_meshes(self):
        return self.emotional_meshes