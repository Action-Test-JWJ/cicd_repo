#!/usr/bin/env python3
import os, sys, queue, time, threading, cv2
import numpy as np
from functools import partial
from types import SimpleNamespace
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import RegionOfInterest, Image
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory

# ---- 모듈 검색 경로 추가 ----
# 설치 공간: 패키지 share/common 추가 + 소스 공간 fallback
SHARE_DIR = None
try:
    SHARE_DIR = get_package_share_directory('hailo_edie_human_detection')
except Exception:
    SHARE_DIR = None

paths_to_add = []
if SHARE_DIR:
    COMMON_DIR = os.path.join(SHARE_DIR, 'common')
    TRACKER_DIR = os.path.join(COMMON_DIR, 'tracker')
    paths_to_add.extend([SHARE_DIR, COMMON_DIR, TRACKER_DIR])

# 소스 트리에서 실행될 때를 대비한 fallback (…/edie_hailo/common)
try:
    source_common = Path(__file__).resolve().parents[3] / 'common'
    source_tracker = source_common / 'tracker'
    for p in [str(source_common), str(source_tracker)]:
        if os.path.isdir(p):
            paths_to_add.append(p)
except Exception:
    pass

for p in paths_to_add:
    if p not in sys.path:
        sys.path.insert(0, p)

from common.tracker.byte_tracker import BYTETracker
from common.hailo_inference import HailoAsyncInference
from common.toolbox import init_input_source, get_labels, load_json_file, preprocess, visualize, default_preprocess
from .object_detection_post_process import inference_result_handler  # <- 프로젝트의 후처리

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('hailo_edie_human_detection')

        # -------- Parameters (launch에서 주입) --------
        self.declare_parameter('net', 'yolov8n.hef')
        self.declare_parameter('labels', str(Path(__file__).parent.parent / 'common' / 'coco.txt'))
        self.declare_parameter('batch_size', 1)
        self.declare_parameter('resolution', 'sd')            # sd|hd|fhd (입력은 ROS 이미지지만, 스케일링 기준)
        self.declare_parameter('track', False)                # BYTETracker 사용 여부
        self.declare_parameter('show_fps', False)             # FPS 로그
        self.declare_parameter('show_window', False)          # OpenCV 윈도우 표시 여부
        self.declare_parameter('measure_latency', True)       # 추론 지연 측정

        self.net_path          = self.get_parameter('net').get_parameter_value().string_value
        self.labels_path       = self.get_parameter('labels').get_parameter_value().string_value
        self.batch_size        = self.get_parameter('batch_size').get_parameter_value().integer_value
        self.resolution        = self.get_parameter('resolution').get_parameter_value().string_value
        self.enable_tracking   = self.get_parameter('track').get_parameter_value().bool_value
        self.show_fps          = self.get_parameter('show_fps').get_parameter_value().bool_value
        self.show_window       = self.get_parameter('show_window').get_parameter_value().bool_value
        self.measure_latency    = self.get_parameter('measure_latency').get_parameter_value().bool_value

        self.get_logger().info(f'net_path: {self.net_path}')
        self.get_logger().info(f'labels_path: {self.labels_path}')
        self.get_logger().info(f'batch_size: {self.batch_size}')
        self.get_logger().info(f'resolution: {self.resolution}')
        self.get_logger().info(f'enable_tracking: {self.enable_tracking}')
        self.get_logger().info(f'show_fps: {self.show_fps}')
        self.get_logger().info(f'show_window: {self.show_window}')

        # -------- Validate & setup --------
        if not os.path.exists(self.net_path):
            raise FileNotFoundError(f'HEF not found: {self.net_path}')
        if not os.path.exists(self.labels_path):
            raise FileNotFoundError(f'Labels not found: {self.labels_path}')

        self.bridge = CvBridge()
        self.labels = get_labels(self.labels_path)
        # config.json 경로 결정: 설치공간 우선, 없으면 소스 루트 fallback
        config_path = None
        if SHARE_DIR:
            maybe = os.path.join(SHARE_DIR, 'config.json')
            if os.path.exists(maybe):
                config_path = maybe
        if config_path is None:
            maybe = str(Path(__file__).resolve().parents[2] / 'config.json')
            if os.path.exists(maybe):
                config_path = maybe
        if not config_path:
            raise FileNotFoundError('config.json not found in share or source tree')
        self.config_data = load_json_file(config_path)

        # Tracker (옵션)
        self.tracker = None
        if self.enable_tracking:
            tracker_cfg = self.config_data.get('visualization_params', {}).get('tracker', {})
            self.tracker = BYTETracker(SimpleNamespace(**tracker_cfg))

        # Hailo inference (입력/출력 큐)
        self.input_queue: queue.Queue = queue.Queue()
        self.output_queue: queue.Queue = queue.Queue()

        # inference 콜백(모델 출력 -> output_queue로 push)
        # output_queue 인자를 미리 고정해 Hailo 런타임이 요구하는 콜백 시그니처에 맞추어 전달.
        self.inference_callback_fn = partial(self._inference_callback, output_queue=self.output_queue)

        # Hailo 런타임 객체
        # 큐에는 전처리된 프레임 리스트만 넣으므로 원본 프레임 동반 전달 비활성화
        self.hailo = HailoAsyncInference(
            self.net_path, self.input_queue, self.inference_callback_fn,
            self.batch_size, send_original_frame=False
        )
        # 모델 입력 크기 (H, W, C)
        self.input_h, self.input_w, _ = self.hailo.get_input_shape() # h: 640, w: 640 나옴.
        self.get_logger().info(f'input_h: {self.input_h}, input_w: {self.input_w}')
        self.get_logger().info('--------------------------------------------------------------')

        # FPS
        self._frame_counter = 0
        self._t0 = time.time()

        # ROS Pub/Sub
        self.pub_human_center_point = self.create_publisher(Point, '/edie8/vision/closest_human_point', 10)
        self.pub_human_roi   = self.create_publisher(RegionOfInterest, '/edie8/vision/closest_human_roi', 10)
        self.sub_img   = self.create_subscription(Image, '/edie8/vision/image_raw', self.ImageCallback, 10)

        # 후처리/발행 스레드 기동
        self._post_thread = threading.Thread(target=self._postprocess_worker, daemon=True)
        self._post_thread.start()

        # 추론 스레드 기동
        self._infer_thread = threading.Thread(target=self.hailo.run, daemon=True)
        self._infer_thread.start()

        self.get_logger().info('hailo_edie_human_detection node is up.')

    # ---------- Subscriber: Image ----------
    def ImageCallback(self, msg: Image):
        # self.get_logger().info(f'ImageCallback: {msg.width}, {msg.height}')
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        pre = default_preprocess(frame_bgr, self.input_w, self.input_h)  # 패딩으로 (input_w,input_h)에 맞춤
        if self.measure_latency:
            t_enqueue = time.perf_counter_ns()
            self.input_queue.put([(pre, {'t_enqueue': t_enqueue})])
        else:
            self.input_queue.put([pre])
        # self.get_logger().info(f'Enqueue: {pre.shape}, qsize(after put)={self.input_queue.qsize()}')

    # ---------- Hailo inference callback ----------
    def _inference_callback(self, completion_info, bindings_list: list, input_batch: list, output_queue: queue.Queue):
        if completion_info.exception:
            self.get_logger().error(f'Inference error: {completion_info.exception}')
            return

        # self.get_logger().info(f'[Inference callback] bindings_list: {bindings_list}')
        # self.get_logger().info(f'[Inference callback] input_batch: {input_batch}')

        # 각 배치의 결과를 output_queue로 전달
        for i, bindings in enumerate(bindings_list):
            if len(bindings._output_names) == 1:
                result = bindings.output().get_buffer()
            else:
                result = {
                    name: np.expand_dims(bindings.output(name).get_buffer(), axis=0)
                    for name in bindings._output_names
                }

            # latency stamp
            frame_item, meta = input_batch[i] if (isinstance(input_batch[i], tuple) and len(input_batch[i])==2) else (input_batch[i], {})
            if self.measure_latency:
                meta = dict(meta)
                t_end = time.perf_counter_ns()
                meta['t_infer_end'] = t_end
                t_start = meta.get('t_infer_start')
                if t_start:
                    meta['infer_ms'] = (t_end - t_start) / 1e6
            # (원본프레임, 모델원시출력, 메타) push
            output_queue.put(((frame_item, meta), result))
            # self.get_logger().info(f'[Inference callback] output_queue.qsize(): {output_queue.qsize()}')
            # self.get_logger().info(f'[Inference callback] Q id: {id(output_queue)}')
            # self.get_logger().info('---------------------------------------------\n')

            self._frame_counter += 1

            if self.show_fps and (self._frame_counter % 100 == 0):
                dt = time.time() - self._t0
                fps = self._frame_counter / max(1e-6, dt)
                self.get_logger().info(f'~ {self._frame_counter} frames, ~{fps:.2f} FPS')

    # ---------- Postprocess & Publish worker ----------
    def _postprocess_worker(self):
        """
        output_queue에서 (frame, result)를 꺼내 후처리 핸들러에 전달.
        사람(bbox) 중 가장 큰 면적을 'closest'로 간주하여 center/ROI publish.
        """
        # self.get_logger().info(f'Postprocess worker started')
        # self.get_logger().info(f'[Postprocess worker] Q id: {id(self.output_queue)}')
        # self.get_logger().info('---------------------------------------------\n')

        # 목적: 후처리 함수가 매번 동일한 labels/config/tracker를 받도록 미리 바인딩해 호출부를 단순화.
        post_cb = partial(
            inference_result_handler,
            labels=self.labels,
            config_data=self.config_data,
            tracker=self.tracker
        )

        while rclpy.ok():
            # self.get_logger().info(f'[Postprocess worker] Q id: {id(self.output_queue)}')
            # # self.get_logger().info(f'[Postprocess worker] output_queue: {self.output_queue.qsize()}')
            # self.get_logger().info('======================================================\n')

            item = self.output_queue.get()
            if item is None:
                self.get_logger().info('[Postprocess worker] None}')
                break

            (frame_item, meta), raw_result = item
            frame = frame_item

            # ⚠️ 여기 반환 형식 맞추기:
            # 프로젝트의 inference_result_handler가 (annotated_frame, detections) 혹은 detections만 리턴할 수 있음.
            try:
                processed = post_cb(frame, raw_result)
                # self.get_logger().info(f'[Postprocess worker] isinstance(processed, tuple): {isinstance(processed, tuple)}')
            except Exception as e:
                self.get_logger().error(f'postprocess error: {e}')
                continue

            annotated_frame = None
            detections = None
            if isinstance(processed, tuple) and len(processed) == 2:
                annotated_frame, detections = processed
            else:
                detections = processed
                
            # latency log (preprocess, inference, postprocess, total)
            if self.measure_latency and isinstance(meta, dict):
                t_put = meta.get('t_enqueue')
                t_start = meta.get('t_infer_start')
                t_end = meta.get('t_infer_end')
                t_post_end = time.perf_counter_ns()
                meta['t_post_end'] = t_post_end
                if t_put and t_start and t_end:
                    pre_ms = (t_start - t_put) / 1e6
                    infer_ms = meta.get('infer_ms') if meta.get('infer_ms') is not None else (t_end - t_start) / 1e6
                    post_ms = (t_post_end - t_end) / 1e6
                    total_ms = (t_post_end - t_put) / 1e6
                    self.get_logger().info(f'latency ms -> pre:{pre_ms:.2f}, infer:{infer_ms:.2f}, post:{post_ms:.2f}, total:{total_ms:.2f}')

            # 기대하는 detections 포맷 예시:
            # [{'label':'person','score':0.92,'bbox':[x,y,w,h]} ...]
            # 혹은 [{'cls_id':0,'conf':0.92,'x':...,'y':...,'w':...,'h':...}]
            # 안전한 빈값 판정: NumPy 배열은 truth value 모호성 회피
            if (
                detections is None or
                (isinstance(detections, np.ndarray) and detections.size == 0) or
                (isinstance(detections, (list, tuple, dict)) and len(detections) == 0)
            ):
                continue

            # 사람만 필터링
            person_names = {'person', 'Person', 'PERSON'}
            def is_person(det):
                # dict류에만 키 검사를 수행하고, 그 외 타입(np.ndarray 등)은 False 처리
                if isinstance(det, dict):
                    if 'label' in det:
                        try:
                            return det['label'] in person_names or str(det['label']).lower() == 'person'
                        except Exception:
                            return False
                    if 'cls_id' in det:
                        try:
                            cls_id = int(det['cls_id'])
                            if 0 <= cls_id < len(self.labels):
                                return self.labels[cls_id].lower() == 'person'
                        except Exception:
                            return False
                return False

            person_dets = [d for d in detections if is_person(d)]
            # self.get_logger().info(f'[Postprocess worker] person_dets: {person_dets}')
            # self.get_logger().info(f'[Postprocess worker] len(person_dets): {len(person_dets)}')
            # self.get_logger().info(f'[Postprocess worker] =====4444=====')
            if not person_dets:
                continue

            # 면적 최대(가장 가까운 것으로 가정)
            def bbox_area(d):
                if 'bbox' in d:
                    x, y, w, h = d['bbox']
                else:
                    x = d.get('x', 0); y = d.get('y', 0)
                    w = d.get('w', 0); h = d.get('h', 0)
                return max(0, float(w)) * max(0, float(h))

            best = max(person_dets, key=bbox_area)

            # 좌표 꺼내기
            if 'bbox' in best:
                x, y, w, h = best['bbox']
            else:
                x = best.get('x', 0); y = best.get('y', 0)
                w = best.get('w', 0); h = best.get('h', 0)

            # 안전 클램핑 (프레임 경계 내)
            H, W = frame.shape[:2]
            x = float(max(0.0, min(float(x), max(0, W - 1))))
            y = float(max(0.0, min(float(y), max(0, H - 1))))
            w = float(max(0.0, min(float(w), max(0, W - x))))
            h = float(max(0.0, min(float(h), max(0, H - y))))

            cx = x + w / 2.0
            cy = y + h / 2.0

            # Publish center point
            self.PubHumanCenterPoint(cx, cy)
            # Publish ROI
            self.PubHumanROI(x, y, w, h)
            
            # Show window
            if self.show_window:
                to_show = annotated_frame if isinstance(annotated_frame, np.ndarray) else frame
                try:
                    cv2.imshow('Human Detection', to_show)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.show_window = False
                        try:
                            cv2.destroyAllWindows()
                        except Exception:
                            pass
                except Exception:
                    pass

    def PubHumanCenterPoint(self, x, y):
        msg = Point()
        msg.x, msg.y, msg.z = float(x), float(y), 0.0

        self.pub_human_center_point.publish(msg)

    def PubHumanROI(self, x, y, w, h):
        msg = RegionOfInterest()
        msg.x_offset, msg.y_offset = int(x), int(y)
        msg.width, msg.height = int(w), int(h)
        msg.do_rectify = False
		
        self.pub_human_roi.publish(msg)

    # ---------- Shutdown ----------
    def destroy_node(self):
        try:
            self.output_queue.put(None)
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        
        super().destroy_node()

def main(argv=None):
    if argv is None:     # new   
        argv = sys.argv  # argv가 None이면 sys.argv로 설정

    rclpy.init(args=argv)
    node = HumanDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()