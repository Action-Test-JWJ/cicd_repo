#!/usr/bin/env python3
from typing import List, Optional
import os, cv2, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest, Image
from edie_msgs.msg import HumanDetection2DArray
from edie_vora_common_python.base_infer_client import BaseInferClient
from cv_bridge import CvBridge

PERSON_ID = 14

class HumanDetectionClient(BaseInferClient):
    def __init__(self):
        super().__init__('human_detection_client', mode='human')
        
        # 출력 퍼블리셔
        # self.pub_point = self.create_publisher(Point, '/edie8/vision/closest_human_point', 10)
        self.pub_roi   = self.create_publisher(RegionOfInterest, '/edie8/vision/closest_human_roi', 10)
        
        self.bridge = CvBridge()

        self.declare_parameter('similar_box_threshold', 20.0)
        self.declare_parameter('confidence_threshold', 0.3)
        self.similar_th = float(self.get_parameter('similar_box_threshold').value)
        self.conf_th    = float(self.get_parameter('confidence_threshold').value)

        """ 윈도우 창 초기화 """
        self.window_name = 'HumanDetection'
        self.window_width = 640
        self.window_height = 480
        # 창 표시 여부 파라미터 선언
        self.declare_parameter('show_window', False)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.get_logger().info(f"[HumanDetectionClient.__init__] show_window: {self.show_window}")
        """ 키 입력 딜레이 """
        self.wait_key_delay = 1  # space로 0/1 토글

        """ OpenCV HighGUI 초기화 """
        if self.show_window:
            self.InitOpenCVWindow()

        self.get_logger().info('HumanDetectionClient ready')

    def InitOpenCVWindow(self) -> None:
        """
        OpenCV HighGUI 초기화
        Returns:
        """
        # cv2.WINDOW_AUTOSIZE: 영상 크기에 맞도록 자동으로 윈도우 크기 조정 (사용자가 크기 조정 불가)
        # cv2.WINDOW_GUI_NORMAL: 상태나 도구바 없는 윈도우로 출력
        # cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE + cv2.WINDOW_GUI_NORMAL)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        # 지정한 _window_name을 갖는 윈도우 크기를 window_width X window_height 크기로 변경
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        # 지정한 _window_name을 갖는 윈도우 위치를 (400, 600) 위치로 이동
        cv2.moveWindow(self.window_name, 400, 600)

    # Base hook
    def HandleHumanInferenceResult(self, dets: HumanDetection2DArray):
        if not dets.human_det:
            return

        # 현재 프레임에서 'person' 객체 찾기 및 가장 큰 사람의 bounding box 찾기
        ## 1) person + score >= 0.3
        person_buffer = []
        max_box_size = 0.0
        min_dst = 987654321

        # 이미지 프레임의 중심점
        frame_center_x = self.window_width / 2
        frame_center_y = self.window_height / 2

        for d in dets.human_det:
            # 로그: 각 후보 요약
            # self.get_logger().info(f'label={d.label} score={d.score:.3f} roi=({d.roi.x_offset},{d.roi.y_offset},{d.roi.width},{d.roi.height})')
            if (d.label == PERSON_ID or str(d.label) == str(PERSON_ID) or str(getattr(d, 'label', '')) == 'person') and d.score >= self.conf_th:
                # max_box_size 업데이트
                box_size = float(d.roi.width) * float(d.roi.height)
                if box_size > max_box_size: 
                    max_box_size = box_size

                person_buffer.append((d, box_size))

        if not person_buffer:
            return

        # 추종 객체 선정 알고리즘        
        ## 1차 필터링:
        ## TH 활용해서 개별 box_size가 max_box_size와 차이가 적은 놈만 score 증가
        score_buffer = [0] * len(person_buffer)
        for i, (_, box_size) in enumerate(person_buffer):
            if abs(max_box_size - box_size) <= self.similar_th:
                score_buffer[i] += 1

        ## 2차 필터링:
        ## 1차 필터링 된 놈들 중에서 프레임 중심점으로부터 가장 가까운 사람에 대해 score 증가
        for pidx, pscore in enumerate(score_buffer):
            if pscore > 0:
                # 'person' 객체 box 중심점
                box = person_buffer[pidx][0].roi
                box_center_x = box.x_offset + box.width * 0.5
                box_center_y = box.y_offset + box.height * 0.5
                xdist = abs(box_center_x - frame_center_x)
                ydist = abs(box_center_y - frame_center_y)
                dst = math.sqrt(xdist**2 + ydist**2)  

                # 최소 거리 업데이트
                if min_dst > dst:
                    min_dst = dst
                    score_buffer[pidx] += 1

        # # 프레임 크기(타임스탬프 이미지가 없으므로 ROI 기준 중심만 사용)
        # # 근사 중심 비교: ROI center가 (cx,cy)에 가장 가까운 것 +1
        # # 단, 실제 프레임 크기를 알고 싶으면 Base에서 최근 이미지 크기를 tap해 저장하면 됨
        # best_d2, best_i = 1e18, -1
        # for i, (d, _) in enumerate(person_buffer):
        #     cx = d.roi.x_offset + d.roi.width * 0.5
        #     cy = d.roi.y_offset + d.roi.height * 0.5
        #     # 프레임 중심을 모르면 가장 큰 박스 중심끼리 상대 비교만
        #     # 여기서는 간단히 (0,0) 대비 거리 최소로 근사
        #     d2 = cx*cx + cy*cy
        #     if d2 < best_d2:
        #         best_d2 = d2
        #         best_i = i

        # if best_i >= 0: 
        #     score_buffer[best_i] += 1

        # 4) 최종 점수 최고 선택
        pick = max(range(len(score_buffer)), key=lambda i: score_buffer[i])
        d, _ = person_buffer[pick]

        # 로그: 최종 추종 대상 정보
        cx = d.roi.x_offset + d.roi.width * 0.5
        cy = d.roi.y_offset + d.roi.height * 0.5
        # self.get_logger().info(f"TRACK target: label={d.label} score={d.score:.3f} center=({cx:.1f},{cy:.1f}) size=({d.roi.width}x{d.roi.height})")

        # publish point & roi
        # p = Point()
        # p.x = float(cx) 
        # p.y = float(cy)
        # p.z = 0.0
        # self.pub_point.publish(p)

        roi = RegionOfInterest()
        roi.x_offset = int(cx) #d.roi.x_offset
        roi.y_offset = int(cy) #d.roi.y_offset
        roi.width = int(d.roi.width)
        roi.height = int(d.roi.height) 
        roi.do_rectify = False
        self.pub_roi.publish(roi)

        # 주석 이미지 시각화(OpenCV)
        if self.show_window:
            try:
                last_img: Image = getattr(self, '_last_image', None)
                if last_img is None:
                    return
                #frame = self.bridge.imgmsg_to_cv2(last_img, desired_encoding='bgr8')
                frame = self.bridge.imgmsg_to_cv2(last_img, desired_encoding='mono8')
                x, y, w, h = d.roi.x_offset, d.roi.y_offset, d.roi.width, d.roi.height
                # 바운딩 박스 & 중심점 & 라벨 텍스트
                cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(h+y)), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)
                cv2.putText(frame, f"person {d.score:.2f}", (int(x), max(0, int(y)-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                cv2.imshow(self.window_name, frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f'annotate failed: {e}')


def main():
    rclpy.init()
    node = HumanDetectionClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        