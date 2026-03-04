#!/usr/bin/env python3

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import StaticTransformBroadcaster
import numpy as np
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, PoseArray
from std_msgs.msg import Bool
import math
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import numpy as np
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import time
from edie_aruco.utils import Utils

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from rclpy.duration import Duration
from collections import deque

class ArucoDetectionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_detection_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.12,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_1000",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/edie8/vision/image_raw", #"/image/compressed"
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/edie8/vision/camera_info", # "/camera_info"
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="camera_link",  
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="aruco_marker_frame",
            value="aruco_marker",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="visualize",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="To use Rviz2 visualize.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        self.aruco_marker_frame = (
            self.get_parameter("aruco_marker_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"Aruco marker frame: {self.aruco_marker_frame}")

        self.visualize = (
            self.get_parameter("visualize").get_parameter_value().bool_value
        )
        self.get_logger().info(f"visualize: {self.visualize}")
        self.get_logger().info(f"cv2 version: {cv2.__version__}")

        self.mtx = None
        self.dst = None

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            # if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
            if type(dictionary_id) != type(cv2.aruco.DICT_6X6_1000):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        # 카메라 정보 구독 설정
        self.sub_camera_info = self.create_subscription(
            CameraInfo, info_topic, self.CameraInfoCallback, 10
        )

        # 이미지 구독 설정
        self.create_subscription(
            Image, image_topic, self.CameraImageRawCallback, 10
        )

        # Set up publishers
        self.pub_robot_pose = self.create_publisher(PoseStamped, "/edie8/localization/robot_pose", 10)

        self.tfbroadcaster = TransformBroadcaster(self)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        
        self.base_to_camera_matrix = np.array([
            [ 0.000, -1.000,  0.000, -0.000],
            [ 0.643,  0.000, -0.766,  0.069],
            [ 0.766,  0.000,  0.643,  -0.088],
            [ 0.000,  0.000,  0.000,  1.000]
        ])

        # TF 질의용 버퍼‧리스너
        self.tf_buffer     = Buffer()
        self.tf_listener   = TransformListener(self.tf_buffer, self)

        # marker_0 → odom 동적 퍼블리셔
        self.mk0_odom_pub  = TransformBroadcaster(self)

        # 최소 퍼블리시 주기(선택)
        self.min_period    = 0.05        # 20 Hz 한도
        self.last_pub      = self.get_clock().now()

        self.get_logger().info(f"Transformation matrix:\n{self.base_to_camera_matrix}")

        self.bridge = CvBridge()

        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.static_table = {
            0: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            25 : ((0.83, 0.235, 0.42), (0.0, -1.57, 0.0)),
            50 : ((0.83, 0.235, 1.1), (0.0, -1.57, 0.0)),
            75 : ((0.83, 0.235, 2.01), (0.0,  -1.57, 0.0)),
            100: ((0.0, 0.0, 3.065), (0.0,  3.14, 0.0)),
            125: ((-0.9, 0.0, 3.065), (0.0,  3.14, 0.0)),
            150: ((-1.675,  0.185, 2.22), (0.0,  1.57, 0.0)),
            175: ((-1.675,  0.185, 1.32), (0.0,  1.57, 0.0)),
            200: ((-1.675,  0.185, 0.42), (0.0,  1.57, 0.0)),
            225: ((-0.9,  0.235, 0.0), (0.0,  0.0, 0.0)),
        }

        self.last_seen     = {}                          # 🆕 {id: rclpy.Time}
        self.expire_sec    = 0.5                         # 만료 시간 (필요시 조정)
        self.create_timer(0.1, self.prune_marker_tf)     # 🆕 10 Hz 로 만료 검사

        self.utils = Utils()

        # ==== ⬇⬇ 필터 추가 (Median -> LowPass) ⬇⬇ ====
        self.filter_enable = True

        # 시간축 중앙값(이상치 억제)
        self.mf_x   = self.utils.MedianFilter(window=11)
        self.mf_y   = self.utils.MedianFilter(window=11)
        self.mf_yaw = self.utils.MedianFilter(window=11)  # 언랩 각에 적용

        # 저역통과(EMA) — cutoff(Hz) 기반 동적 α
        self.lp_x   = self.utils.LowPass(cutoff=3.0, freq=30.0)
        self.lp_y   = self.utils.LowPass(cutoff=3.0, freq=30.0)
        self.lp_yaw = self.utils.LowPass(cutoff=2.0, freq=30.0)

        # yaw 언랩 상태
        self._yaw_unwrap_prev = None

    def CameraInfoCallback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.mtx = self.intrinsic_mat
        self.dst = 0.0
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.sub_camera_info)

    def CameraImageRawCallback(self, img_msg):
        buffer_x = []
        buffer_y = []
        buffer_yaw = []
        
        self.BroadcastOriginStaticTf()

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        try:
            img_np = image_msg_to_mat_step(img_msg)   # (H,W) 또는 (H,W,3)
        except Exception as e:
            self.get_logger().error(f"image_msg_to_mat_step failed: {e}")
            return

        # 2) 검출용 입력 보장: mono8
        if img_np.ndim == 3 and img_np.shape[2] == 3:
            img_det = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)
        else:
            img_det = img_np

        # (선택) 연속 메모리 보장
        if not img_det.flags['C_CONTIGUOUS']:
            img_det = img_det.copy()

        # 3) 검출은 그레이만 사용
        corners, marker_ids, rejected = self.aruco_detector.detectMarkers(img_det)

        if marker_ids is not None:
            # ArUco marker has been detected

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.mtx, self.dst
            )

            for i, marker_id in enumerate(marker_ids):
                # ①  marker → camera  (OpenCV 결과)
                T_marker_cam = np.eye(4)
                T_marker_cam[:3,:3] = cv2.Rodrigues(rvecs[i][0])[0]
                T_marker_cam[:3, 3] = tvecs[i][0]
                self.get_logger().info(f"\n{T_marker_cam}")

                # ②  camera → marker  (역행렬)
                T_cam_marker = np.linalg.inv(T_marker_cam)
                # self.get_logger().info(f"T_cam_marker: {T_cam_marker}")

                # ③  camera → base  (TF에서 조회해둔 값)
                T_base_cam   = self.base_to_camera_matrix    # ← 이미 camera 기준 base 임

                # ④  최종 : marker → base
                T_marker_base = T_cam_marker @ T_base_cam

                # if marker_id == 0:
                #     T_marker0_markerX = np.eye(4)                 # 자기 자신
                # else:
                #     xyz, rpy = self.static_table[int(marker_id)]
                #     T_marker0_markerX = self.static_tf_as_matrix(xyz, rpy)
                if marker_id == 0:
                    T_marker0_markerX = np.eye(4)
                else:
                    mid = int(marker_id)
                    if mid not in self.static_table:
                        self.get_logger().warn(f"Marker ID {mid} not in static table, skipping")
                        continue
                    xyz, rpy = self.static_table[mid]
                    T_marker0_markerX = self.static_tf_as_matrix(xyz, rpy)

                # --- 최종: marker0 → base ---------------------------------
                T_marker0_base = T_marker0_markerX @ T_marker_base

                # 위치·자세 추출 (marker0 기준)
                pos0 = T_marker0_base[:3, 3]
                yaw0, pitch0, roll0 = R.from_matrix(
                        T_marker0_base[:3, :3]).as_euler('zyx', degrees=True)

                self.get_logger().info(
                    f"[id:{marker_id}] x={pos0[0]:.3f} y={pos0[1]:.3f} z={pos0[2]:.3f}  "
                    f"yaw={yaw0:+.1f}° pitch={pitch0:+.1f}° roll={roll0:+.1f}°")

                self.BroadcastMarkerDynamicTf(int(marker_id))       # TF 브로드캐스트 시도


                try:
                    tf_odom_base = self.tf_buffer.lookup_transform(
                        'odom', 'base_footprint',                       # 최신 시각
                        rclpy.time.Time()
                    )
                except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                    self.get_logger().warn(f'Waiting for TF odom->base_footprint: {ex}')
                    return

                # TF 메시지를 4×4 행렬로
                T_odom_base = self.tf_to_mat(tf_odom_base.transform)

                # marker_0 → odom 변환 행렬
                T_mk0_odom = T_marker0_base @ np.linalg.inv(T_odom_base)

                # 퍼블리시 주기 제한(선택)
                now = self.get_clock().now()
                if (now - self.last_pub).nanoseconds * 1e-9 < self.min_period:
                    continue
                self.last_pub = now

                # 행렬 → TF 메시지로 변환해 브로드캐스트
                ts = TransformStamped()
                ts.header.stamp    = now.to_msg()
                ts.header.frame_id = 'marker_0'
                ts.child_frame_id  = 'odom'
                ts.transform       = self.mat_to_tf(T_mk0_odom)
                self.mk0_odom_pub.sendTransform(ts)

                buffer_x.append(-pos0[2])
                buffer_y.append(-pos0[0])

                # yaw0는 "deg" 이므로, 먼저 라디안으로 변환
                yaw_rad = math.radians(yaw0)

                # 축을 x=-z, y=-x로 바꿨으니 yaw도 -90°(= -pi/2)만큼 보정
                yaw_robot_rad = yaw_rad - math.pi/2

                #  -pi ~ pi 로 정규화(옵션)
                yaw_robot_rad = (yaw_robot_rad + math.pi) % (2*math.pi) - math.pi

                buffer_yaw.append(yaw_robot_rad)   # 라디안으로 저장

                if self.visualize:
                    axis_len = self.marker_size * 0.5   # 축 길이(원하는 만큼)

                    cv2.drawFrameAxes(
                        img_det,
                        self.mtx,          # camera matrix
                        self.dst,          # dist coeffs
                        rvecs[i][0],       # rotation vec
                        tvecs[i][0],       # translation vec
                        axis_len
                    )

                    # Draw the axes on the marker
                    cv2.aruco.drawDetectedMarkers(img_det, corners, marker_ids, (0, 255, 0))
                    # Update the ArUco marker offset 
                    M = cv2.moments(corners[0][0])
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
            
            if len(buffer_x) > 0:
                x = float(np.mean(buffer_x))
                y = float(np.mean(buffer_y))

                # 각도 원형 평균(라디안)
                sin_avg = float(np.mean([math.sin(a) for a in buffer_yaw]))
                cos_avg = float(np.mean([math.cos(a) for a in buffer_yaw]))
                yaw_avg_rad = math.atan2(sin_avg, cos_avg)

                # ===== 🔻 필터 파이프라인: Median -> LowPass(EMA) =====
                if self.filter_enable:
                    t_now = self.get_clock().now().nanoseconds * 1e-9

                    # 1) 위치: 중앙값으로 이상치 억제
                    x_med = self.mf_x.filt(x)
                    y_med = self.mf_y.filt(y)

                    # 2) 위치: 저역통과(EMA)
                    x = float(self.lp_x.filt(x_med, t_now))
                    y = float(self.lp_y.filt(y_med, t_now))

                    # 3) 각도: 언랩 → 중앙값 → EMA → 랩
                    yaw_unwrap = self._unwrap_angle(yaw_avg_rad)
                    yaw_med    = self.mf_yaw.filt(yaw_unwrap)
                    yaw_filt   = float(self.lp_yaw.filt(yaw_med, t_now))
                    yaw_avg_rad = self._wrap_pi(yaw_filt)
                # yaw_deg = math.degrees(yaw_avg_rad)
                self.PublishRobotPose(x, y, yaw_avg_rad)
  
        else:
            # ArUco marker has been NOT detected
            aruco_detected_flag = False
            self._yaw_unwrap_prev = None
            self.lp_x.reset(); self.lp_y.reset(); self.lp_yaw.reset()

        if (self.visualize):
            cv2.imshow("camera", img_det)
            cv2.waitKey(1)

    def PubStaticTf(
        self,
        parent_frame: str,
        child_frame: str,
        translation: tuple = (0.0, 0.0, 0.0),
        rotation_euler_rpy: tuple = (0.0, 0.0, 0.0),  # in radians
        rotation_order: str = 'zyx'  # default: yaw-pitch-roll (Z-Y-X)
    ):
        """
        퍼블리시 static TF (rclpy + tf2_ros)
        
        Args:
            parent_frame (str): 상위 프레임 ID (예: 'aruco_marker')
            child_frame (str): 하위 프레임 ID (예: 'odom')
            translation (tuple): (x, y, z) 위치 [단위: meter]
            rotation_euler_rpy (tuple): (roll, pitch, yaw) [단위: rad]
            rotation_order (str): 'xyz', 'zyx' 등 회전 순서 (기본값: 'zyx' → YPR)
        """

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame

        tf_msg.transform.translation.x = translation[0]
        tf_msg.transform.translation.y = translation[1]
        tf_msg.transform.translation.z = translation[2]

        r = R.from_euler(rotation_order, rotation_euler_rpy)
        quat = r.as_quat()

        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        
        self.static_broadcaster.sendTransform(tf_msg)

    def BroadcastOriginStaticTf(self):
        self.PubStaticTf(
            parent_frame="map",
            child_frame="marker_0",
            translation=(0.0, 0.0, 0.0),
            rotation_euler_rpy=(3.14, -1.5708, -1.5708)
        )

    def BroadcastMarkerDynamicTf(self, marker_id):
        if marker_id not in self.static_table:
            return                                   # 목록 외 마커 무시

        xyz, rpy = self.static_table[marker_id]

        ts = TransformStamped()
        ts.header.stamp    = self.get_clock().now().to_msg()
        ts.header.frame_id = "marker_0"
        ts.child_frame_id  = f"marker_{marker_id}"
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = xyz
        q = R.from_euler('xyz', rpy).as_quat()
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = q

        self.tfbroadcaster.sendTransform(ts)
        self.last_seen[marker_id] = self.get_clock().now()   # 🆕 마지막 관측 시각 갱신

    def prune_marker_tf(self):
        now = self.get_clock().now()
        for mid, t in list(self.last_seen.items()):
            if (now - t).nanoseconds * 1e-9 > self.expire_sec:
                # self.get_logger().info(f"marker_{mid} TF expired")
                del self.last_seen[mid]               # 더 이상 sendTransform 하지 않음

    def static_tf_as_matrix(self, xyz, rpy, order='zyx'):
        """(x,y,z)+(roll,pitch,yaw) → 4×4 동차좌표 행렬 반환"""
        T = np.eye(4)
        T[:3, 3] = np.array(xyz)
        T[:3, :3] = R.from_euler(order, rpy).as_matrix()
        return T
        
    def PublishRobotPose(self, x, y, yaw):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # position 설정
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0

        # yaw(라디안) → quaternion 변환
        q = quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pub_robot_pose.publish(pose_msg)

    @staticmethod
    def tf_to_mat(trans):
        """geometry_msgs/Transform → 4×4 행렬"""
        T = np.eye(4)
        T[:3, 3] = [trans.translation.x,
                    trans.translation.y,
                    trans.translation.z]
        q = [trans.rotation.x,
             trans.rotation.y,
             trans.rotation.z,
             trans.rotation.w]
        T[:3, :3] = R.from_quat(q).as_matrix()
        return T

    @staticmethod
    def mat_to_tf(T):
        """4×4 행렬 → geometry_msgs/Transform"""
        trans = TransformStamped().transform   # 껍데기 하나 빌려서 생성
        trans.translation.x, trans.translation.y, trans.translation.z = T[:3, 3]
        trans.translation.y = 0.0
        q = R.from_matrix(T[:3, :3]).as_quat()
        trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w = q
        return trans

    def _wrap_pi(self, a):
        return (a + math.pi) % (2*math.pi) - math.pi

    def _unwrap_angle(self, a):
        if self._yaw_unwrap_prev is None:
            self._yaw_unwrap_prev = a
            return a
        da = a - self._yaw_unwrap_prev
        da = self._wrap_pi(da)
        a_unwrap = self._yaw_unwrap_prev + da
        self._yaw_unwrap_prev = a_unwrap
        return a_unwrap


def image_msg_to_mat_step(msg: Image) -> np.ndarray:
    """
    C++ 코드와 동일한 로직:
      - bpp = step // width 로 추정
      - stride(행 패딩) 허용
      - bpp==1 -> (H,W) mono8, bpp==3 -> (H,W,3) BGR8 로 해석
    """
    if msg.width == 0 or msg.height == 0 or msg.step == 0:
        raise ValueError(f"Invalid image dims/step (w={msg.width}, h={msg.height}, step={msg.step})")

    w, h, step = msg.width, msg.height, msg.step
    bpp = step // w
    # 원본 버퍼를 (행, stride) 2D로 본 뒤 유효 폭만 슬라이스
    row_view = np.ndarray(shape=(h, step), dtype=np.uint8, buffer=msg.data)

    if bpp == 1:
        img = row_view[:, :w]                              # (H, W)
        return img if img.flags['C_CONTIGUOUS'] else img.copy()
    elif bpp == 3:
        strip = row_view[:, :w*3]
        img = strip.reshape(h, w, 3)                       # (H, W, 3)
        return img if img.flags['C_CONTIGUOUS'] else img.copy()
    else:
        # 필요하면 bpp==2(16UC1), bpp==4(RGBA)도 여기서 확장 가능
        raise ValueError(f"Unsupported bpp={bpp} (enc='{msg.encoding}', step={step}, w={w})")

def main():
    rclpy.init()
    node = ArucoDetectionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
