from typing import Optional, List

import math     
import cv2
import os
import numpy as np
import time  
from numpy import ndarray
from box import BoundingBox
from gyurinet.opencv2 import Point, Color                                                                               # 주목! in opencv_struct.py

CLASS_NAMES = (
    "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog",
    "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
)

h_fov_per_pixel = 0.124375  # Horizontal FOV per pixel [deg/pixel]        
v_fov_per_pixel = 0.133335  # Vertical FOV per pixel [deg/pixel]          
h_fov_per_pixel = np.deg2rad(h_fov_per_pixel)
v_fov_per_pixel = np.deg2rad(v_fov_per_pixel)

# 이미지 프레임에 선속도[m/s]와 각속도[rad/s] 표시용
max_linear_x = 0.2          # 1.0 -> 0.76 -> 0.2
min_linear_x = 0.1          # 0.0 -> 0.1
search_linear_x = 0.0       # 0.18 -> 0.0
search_angular_z = 1.0      # 0.5 -> 1.0
stop_angular_z = 0.05
rotate_angular_z = 1.0      # 0.35 -> 1.0

# 사람 인식 관련 파라미터
confidence_threshold = 0.3
# rotate_threshold = 0.36     # 0.31 -> 0.36[rad] = 21[deg]
# deviation_threshold = 60    # 20 -> 60
# box_size_threshold = 0.85   # 0.95(box 높이) -> 0.85(box 대각선)
similar_box_threshold = 20

class BoxDrawer:
    def __init__(self):
        # Initialize with the current time
        self.last_person_detected_time = time.time()
        self.person_detected = False

    def find_person(self, cv_mat, frame_center_x, frame_center_y):
        if self.person_detected == False and time.time() - self.last_person_detected_time > 3:
            ss = "No person detected," + f"Angular z: {search_angular_z}"
            cv2.putText(cv_mat, ss, Point(frame_center_x-200, frame_center_y+100), cv2.FONT_ITALIC, 0.8, Color.GREEN, thickness=3)
    
    def draw(self, cv_mat: ndarray, bounding_boxes: Optional[List[BoundingBox]], draw_label: bool = True):
        # 현재 프레임에서 'person' 객체 찾기 위한 buffer 
        self.person_buffer = []
        # max box 크기와 유사한 'person' 객체 box 저장할 buffer  
        self.score_buffer = []

        # 각 프레임의 시작에서 person_detected를 초기화
        self.person_detected = False

        # return 값 초기화
        center_point = None
        max_width, max_height = None, None
        
        num_person = 0

        max_box_size = 0
        min_dst = 987654321

        # 이미지 프레임의 중심점
        frame_center_x = cv_mat.shape[1] // 2
        frame_center_y = cv_mat.shape[0] // 2
        
        # bounding_boxes가 None일 경우
        if bounding_boxes is None:
            # 3초 지남 유무
        #     self.find_person(cv_mat, frame_center_x, frame_center_y)        
            return None, None, None
           
        # 현재 프레임에서 'person' 객체 찾기 및 가장 큰 사람의 bounding box 찾기
        for box in bounding_boxes:
            start_point = Point(int(box.x1), int(box.y1))
            end_point = Point(int(box.x2), int(box.y2))

            cv2.rectangle(cv_mat, start_point, end_point, Color.BLUE)

            if CLASS_NAMES[box.label] == "person" and box.score >= confidence_threshold:
                # 사람 감지 유무 업데이트
                self.person_detected = True
                # Update the last detected time
                self.last_person_detected_time = time.time()

                ss = f"{CLASS_NAMES[box.label]} {box.score:.4f}"
                cv2.putText(cv_mat, ss, start_point, cv2.FONT_ITALIC, .5, Color.GREEN, thickness=2)

                num_person+=1
                # print(f'{num_person}번째 사람')
                # print(f'box.score: {box.score}')

                # box_width, box_height 구하기 
                box_width = abs(box.x1 - box.x2)
                box_height = abs(box.y1 - box.y2)
                box_size = box_width * box_height

                # max_box_size 업데이트
                if box_size > max_box_size:
                    # print(f'이전 max_box_size: {max_box_size}')
                    max_box_size = box_size
                    # print(f'업데이트된 max_box_size: {max_box_size}')
                    # print('--------------------------------------------------------')

                # 'person' 객체 box 저장
                self.person_buffer.append(box)
            else:
                # 사람 감지 유무 업데이트
                self.person_detected  = False
        
        # 추종 객체 선정 알고리즘
        if self.person_detected and len(self.person_buffer) > 0:
            self.score_buffer = [0] * len(self.person_buffer)

            # 1차 필터링:
            # TH 활용해서 개별 box_size가 max_box_size와 차이가 적은 놈만 score 증가
            for pbox in self.person_buffer:
                pbox_width = abs(pbox.x1 - pbox.x2)
                pbox_height = abs(pbox.y1 - pbox.y2)
                pbox_size = pbox_width * pbox_height  

                if abs(max_box_size - pbox_size) <= similar_box_threshold:
                    # print(f'abs(max_box_size - pbox_size): {abs(max_box_size - pbox_size)}')
                    # print(f'similar_box_threshold: {similar_box_threshold}')
                    # print("--------------------------------------------------------")
                    pidx = self.person_buffer.index(pbox)
                    self.score_buffer[pidx] += 1

            # 2차 필터링:
            # 1차 필터링 된 놈들 중에서 프레임 중심점으로부터 가장 가까운 사람에 대해 score 증가
            for pidx, pscore in enumerate(self.score_buffer):
                if pscore > 0:
                    # 'person' 객체 box 중심점
                    box = self.person_buffer[pidx]
                    box_center_x, box_center_y = int((box.x1 + box.x2) / 2), int((box.y1 + box.y2) / 2)
                    xdist = abs(box_center_x - frame_center_x)
                    ydist = abs(box_center_y - frame_center_y)
                    dst = math.sqrt(xdist**2 + ydist**2)  

                    # 최소 거리 업데이트
                    if min_dst > dst:
                        min_dst = dst
                        self.score_buffer[pidx] += 1
            
            if len(self.score_buffer) > 0:
                # 최종 box_score 최대값 인덱스 구하기
                max_idx = self.score_buffer.index(max(self.score_buffer))
                # print(f'max(self.score_buffer):{max(self.score_buffer)}')
                # print(f'max_idx: {max_idx}')
                # print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
                # print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')

                max_box = self.person_buffer[max_idx]
                # bounding box 중심점 구하기
                max_box_center_x, max_box_center_y = int((max_box.x1 + max_box.x2) / 2), int((max_box.y1 + max_box.y2) / 2)
                
                # 최적화된 (x1*, y1*, x2*, y2*) check_position() 함수에 입력으로 전달 
                center_point = Point(int(max_box_center_x), int(max_box_center_y))
                # print(f'Center point x: {max_box_center_x} & Center point y: {max_box_center_y}')       

                max_width = abs(max_box.x1 - max_box.x2)
                max_height = abs(max_box.y1 - max_box.y2)
                # print(f'max_width: {max_width} & max_height: {max_height}')       

        return center_point, max_width, max_height

