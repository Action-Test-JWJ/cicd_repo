import numpy as np
import math
import cv2

class EmotionalMesh:
    def __init__(self, frame_shape):
        # Source frame
        self.frame_shape = frame_shape

        # Indexes of 'Emotional Mesh' in  Mediapipe
        '''
        - MediaPipe FaceMesh(468 pts) 기준의 얼굴 랜드마크로, 입·코·눈·눈썹의 대표 점들
        - 468개 랜드마크 중 27개 포인트를 선택해 얼굴 특징을 추출 
        - 향후 선택한 27개 랜드마크를 사용해 21개 각도 특징을 계산
        ------------------------------------------------
        61: 왼쪽 입꼬리(바깥 입술 고리)
        291: 오른쪽 입꼬리
        0: 얼굴 중앙 기준점(미간/비근 근처의 중심점)
        17: 윗입술 정중앙(큐피드 보우)
        50, 48: 왼쪽 입술 상·하 측면 포인트
        280, 278: 오른쪽 입술 상·하 측면 포인트
        4: 코끝
        206, 426: 오른쪽 눈 바깥쪽/관자·광대 쪽 보조 포인트
        133: 왼쪽 눈 안쪽 눈구석(내안각)
        145: 왼쪽 눈 위눈꺼풀 중앙 근처
        159: 왼쪽 눈 아래눈꺼풀 중앙 근처
        130: 왼쪽 눈과 눈썹 사이 상측 보조 포인트
        362: 오른쪽 눈 안쪽 눈구석(내안각)
        386: 오른쪽 눈 위눈꺼풀 중앙 근처
        374: 오른쪽 눈 아래눈꺼풀 중앙 근처
        359: 오른쪽 눈과 눈썹 사이 상측 보조 포인트
        122: 왼쪽 눈썹 내측 근처
        351: 오른쪽 눈썹 내측 근처
        46: 왼쪽 눈 바깥 눈구석(외안각) 근처
        105, 107: 왼쪽 눈 위/아래 눈꺼풀의 측면 포인트
        276: 오른쪽 눈 바깥 눈구석 근처
        334, 336: 오른쪽 눈 위/아래 눈꺼풀의 측면 포인트
        ------------------------------------------------
        '''
        self.indexes = [61, 291, 0, 17, 50, 280, 48, 4, 278,
            206, 426, 133, 130, 159, 145, 362, 359, 386, 374, 122,
            351, 46, 105, 107, 276, 334, 336]
        
        # Coordinates of 'Emotional Mesh' - Tuple (x, y)
        self.coordinates = []

        # Relative coordinates in source frame of 'Emotional Mesh' - Tuple (x, y)
        self.rcoordinates = []

        # Bounding box in source frame
        # Format: [cx_min, cy_min, cx_max, cy_max]
        self.bounding_box = np.zeros(4)

        # Angles
        # Format: [angle1, angle2, angle3, ...]
        self.num_angles = 21
        self.angles = np.zeros((1, self.num_angles))
    
    # emotional_mesh_detection.py 에서 호출
    def update_landmarks(self, face_landmarks):
        height, width, _ = self.frame_shape
        for index in self.indexes:
            x = face_landmarks.landmark[index].x
            y = face_landmarks.landmark[index].y
            self.coordinates.append((x, y))
            self.rcoordinates.append((int(x*width), int(y*height)))
        self.__calculate_angles()
        self.__calculate_bounding_box(face_landmarks)

    # 눈/눈썹/입의 정규화 지표 계산
    # 반환: {
    #   'eye_open': float,          # 좌/우 EOR 평균. Eye Open Ratio. (높을수록 눈이 큼)
    #   'brow_gap': float,          # 좌/우 눈썹-윗눈꺼풀 거리 평균
    #   'mouth_open': float         # 좌/우 입술 상하 거리 평균
    # }
    # 모든 값은 눈 사이 거리(내안각-내안각)를 기준으로 정규화됨
    def compute_normalized_metrics(self):
        # 안전 가드
        if len(self.rcoordinates) < 27:
            return {'eye_open': 0.0, 'brow_gap': 0.0, 'mouth_open': 0.0}

        # 로컬 인덱스(설명은 파일 상단 인덱스 표 참조)
        idx = {
            'L_INNER': 11,  # 133. 왼쪽 눈 안쪽 눈구석
            'L_TOP': 14,    # 145. 왼쪽 눈 위눈꺼풀 중앙 근처
            'L_BOTTOM': 13, # 159. 왼쪽 눈 아래눈꺼풀 중앙 근처
            'L_OUTER': 21,  # 46. 왼쪽 눈 바깥쪽 눈구석
            'L_BROW': 12,   # 130. 왼쪽 눈과 눈썹 사이 상측 보조 포인트
            'R_INNER': 15,  # 362. 오른쪽 눈 안쪽 눈구석
            'R_TOP': 17,    # 386. 오른쪽 눈 위눈꺼풀 중앙 근처
            'R_BOTTOM': 18, # 374. 오른쪽 눈 아래눈꺼풀 중앙 근처
            'R_OUTER': 24,  # 276. 오른쪽 눈 바깥쪽 눈구석
            'R_BROW': 16,   # 359. 오른쪽 눈과 눈썹 사이 상측 보조 포인트
            'L_MOUTH_UP': 4,  # 50. 윗입술 정중앙(큐피드 보우)
            'L_MOUTH_LOW': 6, # 48. 왼쪽 입술 상·하 측면 포인트
            'R_MOUTH_UP': 5,  # 280. 오른쪽 입술 상·하 측면 포인트
            'R_MOUTH_LOW': 8  # 278. 오른쪽 입술 상·하 측면 포인트
        }

        def dist(a, b):
            ax, ay = self.rcoordinates[a]
            bx, by = self.rcoordinates[b]
            return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

        # 정규화 기준: 두 눈 내안각 사이 거리
        inter_ocular = dist(idx['L_INNER'], idx['R_INNER'])
        if inter_ocular <= 1e-6:
            inter_ocular = 1.0

        # EOR 유사치(세로/가로)
        # EOR: Eye Open Ratio
        eor_left = dist(idx['L_TOP'], idx['L_BOTTOM']) / max(1e-6, dist(idx['L_INNER'], idx['L_OUTER']))
        eor_right = dist(idx['R_TOP'], idx['R_BOTTOM']) / max(1e-6, dist(idx['R_INNER'], idx['R_OUTER']))
        eye_open = (eor_left + eor_right) * 0.5

        # 눈썹-윗눈꺼풀 간격 (크면 눈썹이 올라간 상태)
        brow_gap_left = dist(idx['L_BROW'], idx['L_TOP']) / inter_ocular
        brow_gap_right = dist(idx['R_BROW'], idx['R_TOP']) / inter_ocular
        brow_gap = (brow_gap_left + brow_gap_right) * 0.5

        # 입 개폐(MAR 대용량) - 양옆의 상하 입술 거리 평균
        mouth_left = dist(idx['L_MOUTH_UP'], idx['L_MOUTH_LOW']) / inter_ocular
        mouth_right = dist(idx['R_MOUTH_UP'], idx['R_MOUTH_LOW']) / inter_ocular
        mouth_open = (mouth_left + mouth_right) * 0.5

        return {
            'eye_open': float(eye_open),
            'brow_gap': float(brow_gap),
            'mouth_open': float(mouth_open)
        }

    # 동작: 세 점을 이루는 삼각형의 내각을 코사인 법칙으로 계산해 총 21개 각도를 산출
    #      각도 벡터는 바로 분류기 입력으로 사용.[emotion_predictor.py predict 함수 참고]
    # 입력: update_landmarks에서 채워진 랜드마크 좌표(self.coordinates)
    # 출력: self.angles[0][0..20]에 순서대로 기록(부동소수, degree)
    def __calculate_angles(self):
        index = 0

        # Angle 0
        self.angles[0][index] = self.__angle(self.coordinates[7], self.coordinates[1], 
            self.coordinates[2])
        index += 1
        # Angle 1
        self.angles[0][index] = self.__angle(self.coordinates[2], self.coordinates[1], 
            self.coordinates[3])
        index += 1
        # Angle 2
        self.angles[0][index] = self.__angle(self.coordinates[0], self.coordinates[2], 
            self.coordinates[1])
        index += 1
        # Angle 3
        self.angles[0][index] = self.__angle(self.coordinates[1], self.coordinates[7], 
            self.coordinates[8])
        index += 1
        # Angle 4
        self.angles[0][index] = self.__angle(self.coordinates[0], self.coordinates[7], 
            self.coordinates[1])
        index += 1
        # Angle 5
        self.angles[0][index] = self.__angle(self.coordinates[8], self.coordinates[5], 
            self.coordinates[1])
        index += 1
        # Angle 6
        self.angles[0][index] = self.__angle(self.coordinates[8], self.coordinates[10], 
            self.coordinates[1])
        index += 1
        # Angle 7
        self.angles[0][index] = self.__angle(self.coordinates[18], self.coordinates[5], 
            self.coordinates[8])
        index += 1
        # Angle 8
        self.angles[0][index] = self.__angle(self.coordinates[8], self.coordinates[7], 
            self.coordinates[20])
        index += 1
        # Angle 9
        self.angles[0][index] = self.__angle(self.coordinates[26], self.coordinates[7], 
            self.coordinates[23])
        index += 1
        # Angle 10
        self.angles[0][index] = self.__angle(self.coordinates[7], self.coordinates[20], 
            self.coordinates[18])
        index += 1
        # Angle 11
        self.angles[0][index] = self.__angle(self.coordinates[20], self.coordinates[18], 
            self.coordinates[5])
        index += 1
        # Angle 12
        self.angles[0][index] = self.__angle(self.coordinates[17], self.coordinates[16], 
            self.coordinates[18])
        index += 1
        # Angle 13
        self.angles[0][index] = self.__angle(self.coordinates[26], self.coordinates[25], 
            self.coordinates[24])
        index += 1
        # Angle 14
        self.angles[0][index] = self.__angle(self.coordinates[20], self.coordinates[26], 
            self.coordinates[25])
        index += 1
        # Angle 15
        self.angles[0][index] = self.__angle(self.coordinates[25], self.coordinates[24], 
            self.coordinates[16])
        index += 1
        # Angle 16
        self.angles[0][index] = self.__angle(self.coordinates[24], self.coordinates[16], 
            self.coordinates[17])
        index += 1
        # Angle 17
        self.angles[0][index] = self.__angle(self.coordinates[18], self.coordinates[20], 
            self.coordinates[26])
        index += 1
        # Angle 18
        self.angles[0][index] = self.__angle(self.coordinates[5], self.coordinates[1], 
            self.coordinates[10])
        index += 1
        # Angle 19
        self.angles[0][index] = self.__angle(self.coordinates[10], self.coordinates[1], 
            self.coordinates[7])
        index += 1
        # Angle 20
        self.angles[0][index] = self.__angle(self.coordinates[10], self.coordinates[8], 
            self.coordinates[5])

    # Private methods ----------------------------------------------------------------
    def __angle(self, point1, point2, point3):
        side1 = self.__distance(point2, point3)
        side2 = self.__distance(point1, point3)
        side3 = self.__distance(point1, point2)
        
        angle = math.degrees(math.acos((side1**2+side3**2-side2**2)/(2*side1*side3)))
        return angle

    def __distance(self, point1, point2):
        x0 = point1[0]
        y0 = point1[1]
        x1 = point2[0]
        y1 = point2[1]
        return math.sqrt((x0 - x1)**2+(y0 - y1)**2)

    def __calculate_bounding_box(self, face_landmarks):
        h, w, _ = self.frame_shape
        cx_min = w
        cy_min = h
        cx_max = cy_max = 0
        for id, lm in enumerate(face_landmarks.landmark):
            cx, cy = int(lm.x * w), int(lm.y * h)
            if cx < cx_min:
                cx_min = cx
            if cy < cy_min:
                cy_min = cy
            if cx > cx_max:
                cx_max = cx
            if cy > cy_max:
                cy_max = cy
        self.bounding_box = [cx_min, cy_min, cx_max, cy_max]
    
    def draw(self, frame):
        for coord in self.rcoordinates:
            cv2.circle(frame, (coord[0], coord[1]), 2, (0, 255, 0), -1)