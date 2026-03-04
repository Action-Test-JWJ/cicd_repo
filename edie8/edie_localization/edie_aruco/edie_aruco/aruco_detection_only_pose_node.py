#!/usr/bin/env python3

import math
import numpy as np
# Monkey patch for numpy.float
# tf_transformations uses np.float, which is removed in numpy 1.24+
# This is a workaround to make it compatible.
if not hasattr(np, 'float'):
    np.float = np.float64
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped, PoseArray, Pose
from edie_msgs.msg import PoseWithInfoStamped
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import UInt8, String

from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener, Buffer

from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from tf_transformations import quaternion_from_euler

from edie_aruco.utils import Utils
from collections import deque

class ArucoDetectionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_detection_node")

        # Declare and read parameters
        self.declare_parameter("sim_mode", False)  # 시뮬레이션 모드 파라미터 추가
        self.declare_parameter("gap_size", 0.02)
        self.declare_parameter("corner_refinement", "SUBPIX")
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_1000")
        self.declare_parameter("image_topic", "/edie8/vision/image_undistorted")  # 기본값
        self.declare_parameter("camera_info_topic", "/edie8/vision/camera_info")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("aruco_marker_frame", "aruco_marker")
        self.declare_parameter("visualize", True)
        
        # Parameters for different marker types
        self.declare_parameter('docking_marker_size', 0.07)
        self.declare_parameter('localization_marker_size', 0.3)
        self.declare_parameter('docking_marker_ids', [1, 2])
        self.declare_parameter('localization_marker_ids', [0, 101, 201, 301])
        self.declare_parameter('docking_board_pose', rclpy.Parameter.Type.DOUBLE_ARRAY)
        
        # Declare parameters for marker poses - these will be populated from YAML
        self.declare_parameter('aruco_marker_position.0', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('aruco_marker_position.101', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('aruco_marker_position.201', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('aruco_marker_position.301', rclpy.Parameter.Type.DOUBLE_ARRAY)

        # 시뮬레이션 모드 확인
        sim_mode = self.get_parameter("sim_mode").value
        
        # YAML 파일에서 실제 image_topic 값 읽어오기
        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.visualize = self.get_parameter("visualize").value
        
        # 시뮬레이션 모드일 때는 강제로 올바른 토픽 사용
        if sim_mode:
            image_topic = "/edie8/vision/image_raw"
            self.get_logger().info("Running in SIMULATION mode - using /edie8/vision/image_raw")
        else:
            self.get_logger().info(f"Running in REAL ROBOT mode - using {image_topic}")
            
        self.get_logger().info(f"Final image topic: {image_topic}")
        self.get_logger().info(f"Final camera info topic: {info_topic}")

        self.docking_marker_size = self.get_parameter("docking_marker_size").value
        self.localization_marker_size = self.get_parameter("localization_marker_size").value
        self.docking_marker_ids = self.get_parameter("docking_marker_ids").value
        self.localization_marker_ids = self.get_parameter("localization_marker_ids").value

        try:
            pose_list = self.get_parameter("docking_board_pose").value
            if pose_list and len(pose_list) == 6:
                xyz, rpy = pose_list[:3], pose_list[3:]
                self.T_map_from_dock_board = self.static_tf_as_matrix(xyz, rpy, order='xyz')
                self.get_logger().info(f"Loaded docking board pose: {pose_list}")
            else:
                self.get_logger().error("Docking board pose is invalid or empty. Using identity.")
                self.T_map_from_dock_board = np.eye(4)
        except rclpy.exceptions.ParameterNotDeclaredException:
            self.get_logger().error("Parameter 'docking_board_pose' not found. Using identity.")
            self.T_map_from_dock_board = np.eye(4)

        self.get_logger().info(f"Docking marker size: {self.docking_marker_size}, ids: {self.docking_marker_ids}")
        self.get_logger().info(f"Localization marker size: {self.localization_marker_size}, ids: {self.localization_marker_ids}")

        self.aruco_marker_position = {}
        for marker_id in self.localization_marker_ids:
            param_name = f'aruco_marker_position.{marker_id}'
            try:
                pose_param = self.get_parameter(param_name)
                pose_list = pose_param.value
                if pose_list and len(pose_list) == 6:
                    xyz = pose_list[:3]
                    rpy = pose_list[3:]  # [roll, pitch, yaw]
                    self.aruco_marker_position[marker_id] = self.static_tf_as_matrix(xyz, rpy, order='xyz')
                    self.get_logger().info(f"Loaded pose for marker {marker_id}: {pose_list}")
                else:
                    self.get_logger().warn(f"Parameter '{param_name}' is invalid or empty.")
            except rclpy.exceptions.ParameterNotDeclaredException:
                 self.get_logger().warn(f"Parameter '{param_name}' not found.")

        self.gap_size = self.get_parameter("gap_size").value
        self.get_logger().info(f"Gap size: {self.gap_size}")

        corner_refinement_method_str = self.get_parameter("corner_refinement").value.upper()
        self.get_logger().info(f"Corner refinement method: {corner_refinement_method_str}")

        dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        self.mtx, self.dst = None, None

        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
        except AttributeError:
            self.get_logger().error(f"Bad aruco_dictionary_id: {dictionary_id_name}")
            return

        self.sub_camera_info = self.create_subscription(CameraInfo, info_topic, self.CameraInfoCallback, 10)
        self.create_subscription(Image, image_topic, self.CameraImageRawCallback, 10)

        self.pub_robot_pose = self.create_publisher(PoseStamped, "/edie8/localization/aruco_marker/robot_pose", 10)
        self.pub_robot_pose_info = self.create_publisher(PoseWithInfoStamped, "/edie8/localization/aruco_marker/robot_pose_info", 10)
        self.pub_pose_samples_viz = self.create_publisher(PoseArray, "/edie8/localization/aruco_marker/pose_samples_for_viz", 10)
        self.pub_center_info = self.create_publisher(PointStamped, '/edie8/localization/aruco_marker/center_info', 10)
        self.pub_marker_count = self.create_publisher(UInt8, "/edie8/localization/aruco_marker/visible_marker_count", 10)

        self.info_msg = None
        self.target_marker_id = None
        self.aruco_dictionary = aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = aruco.DetectorParameters()
        
        refinement_methods = {
            "NONE": aruco.CORNER_REFINE_NONE, "SUBPIX": aruco.CORNER_REFINE_SUBPIX,
            "CONTOUR": aruco.CORNER_REFINE_CONTOUR, "APRILTAG": aruco.CORNER_REFINE_APRILTAG
        }
        self.aruco_parameters.cornerRefinementMethod = refinement_methods.get(corner_refinement_method_str, aruco.CORNER_REFINE_SUBPIX)
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        
        # Array for visualization
        self.pose_array_for_viz = PoseArray()
        self.pose_array_for_viz.header.frame_id = "map"

        # T_optical_from_base: base_link -> camera_optical_frame
        self.T_optical_from_base = np.array([
            [ 0.000, -1.000,  0.000, -0.000],
            [ 0.643,  0.000, -0.766,  0.069],
            [ 0.766,  0.000,  0.643,  -0.088],
            [ 0.000,  0.000,  0.000,  1.000]
        ])
        self.T_base_from_optical = np.linalg.inv(self.T_optical_from_base)

        self.get_logger().info(f"Using hardcoded T_optical_from_base:\n{self.T_optical_from_base}")

        self.tf_broadcaster = TransformBroadcaster(self) 
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Broadcast static TFs for localization markers
        self.broadcast_static_tfs()

        self.filter_enable = True
        self.win_size, self.min_samples = 21, 3
        self.hx, self.hy, self.hyaw = deque(maxlen=self.win_size), deque(maxlen=self.win_size), deque(maxlen=self.win_size)
        self.mf_x, self.mf_y, self.mf_yaw = Utils.MedianFilter(window=21), Utils.MedianFilter(window=21), Utils.MedianFilter(window=21)
        self.lp_x, self.lp_y, self.lp_yaw = Utils.LowPass(cutoff=3.0, freq=30.0), Utils.LowPass(cutoff=3.0, freq=30.0), Utils.LowPass(cutoff=2.0, freq=30.0)
        self._yaw_unwrap_prev = None

        # Subscribe to the target marker ID
        self.create_subscription(String, '/edie8/behavior/target_marker_id', self.target_marker_id_callback, 10)

    def target_marker_id_callback(self, msg):
        try:
            self.target_marker_id = int(msg.data)
        except (ValueError, TypeError):
            self.target_marker_id = None

    def CameraInfoCallback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        # Since using /image_undistorted, distortion is already corrected.
        # Set distortion coefficients to None for solvePnP.
        self.distortion = None 
        self.mtx, self.dst = self.intrinsic_mat, self.distortion
        self.destroy_subscription(self.sub_camera_info)

    def CameraImageRawCallback(self, img_msg):
        if self.info_msg is None: return
        try: img_np = image_msg_to_mat_step(img_msg)
        except Exception as e: self.get_logger().error(f"image_msg_to_mat_step error: {e}"); return

        is_color = img_np.ndim == 3 and img_np.shape[2] == 3
        img_gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY) if is_color else img_np
        img_viz = img_np if is_color else (cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR) if self.visualize else None)
        if not img_gray.flags['C_CONTIGUOUS']: img_gray = img_gray.copy()

        corners, ids, _ = self.aruco_detector.detectMarkers(img_gray)

        count_msg = UInt8()
        count_msg.data = 0
        pose_detected = False
        if ids is not None:
            # --- Start of new visualization code ---
            if self.visualize and img_viz is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    marker_size = 0.0
                    if marker_id in self.docking_marker_ids:
                        marker_size = self.docking_marker_size
                    elif marker_id in self.localization_marker_ids:
                        marker_size = self.localization_marker_size
                    else:
                        continue # Skip if marker size is unknown

                    # Estimate pose of single marker
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, self.mtx, self.dst)
                    
                    # Draw marker axes
                    cv2.drawFrameAxes(img_viz, self.mtx, self.dst, rvecs[0], tvecs[0], marker_size * 0.5)
            # --- End of new visualization code ---

            ids_list = ids.flatten()
            detected_markers_map = {id_val: corners[i] for i, id_val in enumerate(ids_list)}
            
            docking_ids_present = set(self.docking_marker_ids).issubset(detected_markers_map.keys())
            localization_markers_present = [m for m in ids_list if m in self.localization_marker_ids]
            
            if docking_ids_present:
                count_msg.data = len(self.docking_marker_ids)
                self.ProcessDockingPair(detected_markers_map, img_viz)
                pose_detected = True
            elif len(localization_markers_present) > 0:
                count_msg.data = len(localization_markers_present)
                self.ProcessLocalizationMarkers(detected_markers_map, img_viz)
                pose_detected = True
            else:
                count_msg.data = 0

        self.pub_marker_count.publish(count_msg)
        if not pose_detected: self._reset_windows_and_filters()
        if self.visualize and img_viz is not None:
            if ids is not None: cv2.aruco.drawDetectedMarkers(img_viz, corners, ids)
            cv2.imshow("camera", img_viz); cv2.waitKey(1)

    def ProcessDockingPair(self, detected_markers_map, img_viz):
        # Dynamically determine left and right markers based on their x-coordinate in the image
        pairs = [(id_, detected_markers_map[id_][0]) for id_ in self.docking_marker_ids if id_ in detected_markers_map]
        if len(pairs) != 2: return
        
        # Sort by the mean of the x-coordinates of the corners
        pairs.sort(key=lambda p: p[1].mean(axis=0)[0])
        
        (_, corners_left)  = pairs[0]
        (_, corners_right) = pairs[1]

        image_points = np.vstack([corners_left, corners_right]).astype(np.float32)

        half_marker, dist_center = self.docking_marker_size / 2.0, self.docking_marker_size / 2.0 + self.gap_size / 2.0
        
        objp_left  = np.array([[-dist_center-half_marker, +half_marker, 0], [-dist_center+half_marker, +half_marker, 0], [-dist_center+half_marker, -half_marker, 0], [-dist_center-half_marker, -half_marker, 0]])
        objp_right = np.array([[+dist_center-half_marker, +half_marker, 0], [+dist_center+half_marker, +half_marker, 0], [+dist_center+half_marker, -half_marker, 0], [+dist_center-half_marker, -half_marker, 0]])
        object_points = np.vstack([objp_left, objp_right]).astype(np.float32)

        _, rvecs, tvecs, _ = cv2.solvePnPGeneric(object_points, image_points, self.mtx, self.dst, flags=cv2.SOLVEPNP_IPPE)
        if rvecs is None: return

        best_i = 0 if len(rvecs) == 1 or tvecs[0][2] > tvecs[1][2] else 1
        rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, self.mtx, self.dst, rvecs[best_i], tvecs[best_i])
            
        # T_optical_from_dock_board: 도킹 보드 -> 카메라 (OpenCV 좌표계)
        T_optical_from_dock_board = self.rtvec_to_matrix(rvec, tvec)

        # T_base_from_dock_board: 도킹 보드 기준 로봇 베이스의 상대 포즈
        # T_base_from_dock_board = T_base_from_optical @ T_optical_from_dock_board
        #                        = inv(T_optical_from_base) @ T_optical_from_dock_board
        T_base_from_dock_board = np.linalg.inv(self.T_optical_from_base) @ T_optical_from_dock_board

        # T_map_from_base: 최종 로봇 포즈 (맵 기준)
        # T_map_from_base = T_map_from_dock_board @ inv(T_base_from_dock_board)
        T_map_from_base = self.T_map_from_dock_board @ np.linalg.inv(T_base_from_dock_board)
        
        # broadcast_docking_board_tf를 위한 T_base_from_marker 계산
        T_base_from_marker = np.linalg.inv(self.T_optical_from_base) @ T_optical_from_dock_board
        self.broadcast_docking_board_tf(T_base_from_marker)

        # ROS 표준 좌표계로 변환된 base_link 포즈 추출
        pos = T_map_from_base[:3, 3]
        x_raw, y_raw = pos[0], pos[1]
        yaw_raw = self._extract_yaw_from_rotation_matrix(T_map_from_base[:3, :3])
            
        err = cv2.norm(image_points, cv2.projectPoints(object_points, rvec, tvec, self.mtx, self.dst)[0].squeeze(), cv2.NORM_L2) / len(image_points)
        area = float(cv2.contourArea(corners_left) + cv2.contourArea(corners_right))
            
        self.FilterAndPublishPose(x_raw, y_raw, yaw_raw, err, area)
        if self.visualize and img_viz is not None: cv2.drawFrameAxes(img_viz, self.mtx, self.dst, rvec, tvec, self.docking_marker_size)

    def ProcessLocalizationMarkers(self, detected_markers, img_viz):
        all_object_points, all_image_points = [], []
        s = self.localization_marker_size / 2.0
        local_corners = np.array([[-s,s,0,1], [s,s,0,1], [s,-s,0,1], [-s,-s,0,1]]).T

        # 1. Populate points for solvePnP using all visible localization markers
        for marker_id, corners in detected_markers.items():
            if marker_id not in self.localization_marker_ids: continue
            
            T_map_marker = self.aruco_marker_position.get(marker_id)
            if T_map_marker is None: continue

            map_corners_h = T_map_marker @ local_corners
            all_object_points.extend(map_corners_h[:3, :].T)
            all_image_points.extend(corners[0])
        
        if len(all_object_points) < 4: return

        # 2. Determine which marker to publish for center_info
        # Prioritize the target marker if it's visible. Otherwise, use the largest.
        marker_to_publish_corners = None
        marker_to_publish_id = None
        
        # Try to find the target marker first
        if self.target_marker_id is not None and self.target_marker_id in detected_markers:
            marker_to_publish_corners = detected_markers[self.target_marker_id][0]
            marker_to_publish_id = self.target_marker_id
        else:
            # Fallback: find the largest visible marker
            largest_area = 0
            for marker_id, corners in detected_markers.items():
                if marker_id not in self.localization_marker_ids: continue
                
                area = cv2.contourArea(corners[0])
                if area > largest_area:
                    largest_area = area
                    marker_to_publish_corners = corners[0]
                    marker_to_publish_id = marker_id

        # 3. Publish center_info for the selected marker
        if marker_to_publish_corners is not None:
            image_width = img_viz.shape[1] if img_viz is not None else 640
            image_height = img_viz.shape[0] if img_viz is not None else 480
            
            center_x = np.mean(marker_to_publish_corners[:, 0])
            center_y = np.mean(marker_to_publish_corners[:, 1])
            area = cv2.contourArea(marker_to_publish_corners)

            normalized_cx = (center_x / image_width) - 0.5
            normalized_cy = (center_y / image_height) - 0.5
            normalized_area = area / (image_width * image_height)
            
            center_info_msg = PointStamped()
            center_info_msg.header.stamp = self.get_clock().now().to_msg()
            center_info_msg.header.frame_id = f"aruco_marker_{largest_marker_id}"  # Use frame_id to store marker ID
            center_info_msg.point.x = normalized_cx
            center_info_msg.point.y = normalized_cy
            center_info_msg.point.z = normalized_area
            self.pub_center_info.publish(center_info_msg)

        # 4. Continue with pose estimation using all markers
        obj_pts = np.array(all_object_points, dtype=np.float32)
        img_pts = np.array(all_image_points, dtype=np.float32)
        
        success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.mtx, self.dst)
        if not success: return

        # T_optical_from_map: 맵 좌표계 -> 카메라 광학 좌표계 변환
        T_optical_from_map = self.rtvec_to_matrix(rvec, tvec)
        
        # T_map_from_base (로봇 베이스의 맵 기준 포즈) 계산
        # T_map_from_base = T_map_from_optical @ T_optical_from_base
        #                 = inv(T_optical_from_map) @ T_optical_from_base
        T_map_from_base = np.linalg.inv(T_optical_from_map) @ self.T_optical_from_base
        
        pos = T_map_from_base[:3, 3]
        # yaw 추출 방식은 ROS 표준(zyx)을 따르도록 통일
        yaw, pitch, roll = R.from_matrix(T_map_from_base[:3, :3]).as_euler('zyx')
        x_raw, y_raw, yaw_raw = pos[0], pos[1], yaw
        
        reproj_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.mtx, self.dst)
        err = cv2.norm(img_pts, reproj_pts.squeeze(), cv2.NORM_L2) / len(img_pts)
        area = sum(cv2.contourArea(detected_markers[mid]) for mid in detected_markers if mid in self.localization_marker_ids)

        # --- Visualization of ALL calculated poses (no filtering) ---
        pose = Pose()
        pose.position.x, pose.position.y = x_raw, y_raw
        q = quaternion_from_euler(0.0, 0.0, yaw_raw)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        self.pose_array_for_viz.poses.append(pose)
        self.pose_array_for_viz.header.stamp = self.get_clock().now().to_msg()
        self.pub_pose_samples_viz.publish(self.pose_array_for_viz)

        self.FilterAndPublishPose(x_raw, y_raw, yaw_raw, err, area)
        if self.visualize and img_viz is not None:
            # map 좌표계 축을 그리는 것은 rvec, tvec (T_optical_from_map)을 사용하므로 그대로 둡니다.
            cv2.drawFrameAxes(img_viz, self.mtx, self.dst, rvec, tvec, 0.2) # Draw map frame axes

    def FilterAndPublishPose(self, x_raw, y_raw, yaw_raw, reproj_err, marker_area):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        yaw_unwrap = self._unwrap_angle(yaw_raw)

        self.hx.append(x_raw)
        self.hy.append(y_raw)
        self.hyaw.append(yaw_unwrap)

        if self.filter_enable and len(self.hx) >= self.min_samples:
            x_med = self.mf_x.filt(x_raw)
            y_med = self.mf_y.filt(y_raw)
            yaw_med = self.mf_yaw.filt(yaw_unwrap)

            x_f = float(self.lp_x.filt(x_med, t_now))
            y_f = float(self.lp_y.filt(y_med, t_now))
            yaw_f = self._wrap_pi(float(self.lp_yaw.filt(yaw_med, t_now)))

            self.PublishRobotPose(x_f, y_f, yaw_f)
            self.PublishRobotPoseWithInfo(x_f, y_f, yaw_f, reproj_err, marker_area)
            self.BroadcastDynamicTf((x_f, y_f, 0.0), (0.0, 0.0, yaw_f))
        else:
            self.PublishRobotPose(x_raw, y_raw, yaw_raw)
            self.BroadcastDynamicTf((x_raw, y_raw, 0.0), (0.0, 0.0, yaw_raw))

    def PublishRobotPose(self, x, y, yaw_rad):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x, pose_msg.pose.position.y = x, y
        q = quaternion_from_euler(0.0, 0.0, yaw_rad)
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q
        self.pub_robot_pose.publish(pose_msg)

    def PublishRobotPoseWithInfo(self, x, y, yaw_rad, reproj_err, marker_area):
        pose_msg = PoseWithInfoStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.x, pose_msg.pose.y, pose_msg.pose.theta = float(x), float(y), float(yaw_rad)
        pose_msg.info.append(KeyValue(key="reprojection_error", value=f"{reproj_err:.6f}"))
        pose_msg.info.append(KeyValue(key="marker_area", value=f"{marker_area:.1f}"))
        self.pub_robot_pose_info.publish(pose_msg)

    def BroadcastDynamicTf(self, translation, rotation):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "base_aruco"
        tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = float(translation[0]), float(translation[1]), 0.0
        q = quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(tf_msg)
    
    def BroadcastOdomBaseLinkDynamicTf(self, x, y, yaw_rad):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link_aruco"
        tf_msg.transform.translation.x, tf_msg.transform.translation.y = x, y
        q = quaternion_from_euler(0, 0, yaw_rad)
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(tf_msg)

    def broadcast_static_tfs(self):
        tfs = []
        # Broadcast for localization markers
        for marker_id, T_map_marker in self.aruco_marker_position.items():
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()  # 또는 Time(sec=0)
            tf_msg.header.frame_id = "map"
            tf_msg.child_frame_id = f"aruco_marker_{marker_id}"

            pos = T_map_marker[:3, 3]
            quat = R.from_matrix(T_map_marker[:3, :3]).as_quat()  # [x,y,z,w]

            tf_msg.transform.translation.x = float(pos[0])
            tf_msg.transform.translation.y = float(pos[1])
            tf_msg.transform.translation.z = float(pos[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])

            tfs.append(tf_msg)
            
        # Broadcast for docking board pose
        if hasattr(self, 'T_map_from_dock_board'):
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "map"
            tf_msg.child_frame_id = "docking_board"

            pos = self.T_map_from_dock_board[:3, 3]
            quat = R.from_matrix(self.T_map_from_dock_board[:3, :3]).as_quat()

            tf_msg.transform.translation.x = float(pos[0])
            tf_msg.transform.translation.y = float(pos[1])
            tf_msg.transform.translation.z = float(pos[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            
            tfs.append(tf_msg)

        if tfs:
            self.static_tf_broadcaster.sendTransform(tfs)
            self.get_logger().info(f"Broadcasting {len(tfs)} static TFs on /tf_static")

    def broadcast_docking_board_tf(self, T_board_base):
        """도킹 보드 TF 방송: odom→dock_board_obs"""
        try:
            # odom→base_fused TF 가져오기
            now = self.get_clock().now()
            from rclpy.duration import Duration
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_fused", now.to_msg(), timeout=Duration(seconds=0.1)
            )
            
            # TransformStamped를 4x4 행렬로 변환
            T_odom_base = self.transform_to_matrix(transform)
            
            # T_base_board = inv(T_board_base)
            T_base_board = np.linalg.inv(T_board_base)
            
            # T_odom_board_obs = T_odom_base @ T_base_board
            T_odom_board_obs = T_odom_base @ T_base_board
            
            # TF 메시지 생성 및 방송
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "dock_board_obs"
            
            # 위치와 회전 추출
            pos = T_odom_board_obs[:3, 3]
            quat = R.from_matrix(T_odom_board_obs[:3, :3]).as_quat()
            
            tf_msg.transform.translation.x = float(pos[0])
            tf_msg.transform.translation.y = float(pos[1])
            tf_msg.transform.translation.z = float(pos[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            
            self.tf_broadcaster.sendTransform(tf_msg)
            
        except Exception as e:
            # TF 변환 실패 시 무시 (odom→base_fused가 없을 수 있음)
            pass

    def transform_to_matrix(self, transform):
        """TransformStamped를 4x4 변환 행렬로 변환"""
        T = np.eye(4)
        
        # Translation
        T[0, 3] = transform.transform.translation.x
        T[1, 3] = transform.transform.translation.y
        T[2, 3] = transform.transform.translation.z
        
        # Rotation (quaternion to matrix)
        q = transform.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        T[:3, :3] = R.from_quat(quat).as_matrix()
        
        return T

    def static_tf_as_matrix(self, xyz, rpy, order='xyz'):
        T = np.eye(4)
        T[:3, 3] = np.array(xyz)
        T[:3, :3] = R.from_euler(order, rpy).as_matrix()
        return T
    
    def rtvec_to_matrix(self, rvec, tvec):
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.flatten()
        return T
    
    def _extract_yaw_from_rotation_matrix(self, R_matrix):
        """회전 행렬에서 yaw 각도 추출 (ROS 표준)"""
        yaw = math.atan2(R_matrix[1, 0], R_matrix[0, 0])
        return self._wrap_pi(yaw)

    def _wrap_pi(self, a): return (a + math.pi) % (2*math.pi) - math.pi

    def _unwrap_angle(self, a):
        if self._yaw_unwrap_prev is None: self._yaw_unwrap_prev = a
        else: self._yaw_unwrap_prev += self._wrap_pi(a - self._yaw_unwrap_prev)
        return self._yaw_unwrap_prev

    def _reset_windows_and_filters(self):
        self.hx.clear(); self.hy.clear(); self.hyaw.clear()
        self._yaw_unwrap_prev = None
        self.lp_x.reset(); self.lp_y.reset(); self.lp_yaw.reset()
        self.mf_x = Utils.MedianFilter(window=11)
        self.mf_y = Utils.MedianFilter(window=11)
        self.mf_yaw = Utils.MedianFilter(window=11)

def image_msg_to_mat_step(msg: Image) -> np.ndarray:
    if not all([msg.width, msg.height, msg.step]): raise ValueError("Invalid image dims/step")
    w, h, step = msg.width, msg.height, msg.step
    bpp = step // w
    row_view = np.ndarray(shape=(h, step), dtype=np.uint8, buffer=msg.data)
    if bpp == 1: return row_view[:, :w].copy()
    elif bpp == 3: return row_view[:, :w*3].reshape(h, w, 3).copy()
    else: raise ValueError(f"Unsupported bpp={bpp}")

def main():
    rclpy.init()
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
