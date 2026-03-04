#!/usr/bin/env python3
import math
from enum import Enum

import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist, PointStamped, TwistStamped
from std_msgs.msg import UInt8, String, Int16MultiArray
from diagnostic_msgs.msg import KeyValue

from rcl_interfaces.msg import SetParametersResult

from edie_mobile_action.control_pid import PID

from rclpy.time import Time

import random
from edie_msgs.msg import PoseWithInfoStamped

# ===== Command indices =====
CMD_IDLE                 = 0
CMD_INIT_HOME            = 1
CMD_SPIN_ONCE            = 2
CMD_GO_TO_PRE_ALIGN_POSE = 3 
CMD_ALIGNMENT            = 4
CMD_ROTATE_BEHIND        = 5
CMD_GO_BACK              = 6
CMD_FIND_STATION_INIT    = 7
CMD_FIND_STATION_CHARGE  = 8
CMD_MOVE_RANDOMLY        = 9
CMD_SEARCH_ARUCO         = 10
CMD_APPROACH_STATION     = 11
CMD_GO_TO_MIDDLE         = 12
CMD_APPROACH_MARKER      = 13 # New command
CMD_GO_TO_SIDE_LEFT      = 14 # Go to (-1.0, 0.85)
CMD_GO_TO_SIDE_RIGHT     = 15 # Go to (-1.0, -0.85)
CMD_GO_TO_FRONT          = 16 # Go forward slightly for wheel alignment
CMD_GO_TO_SIDE_INIT      = 17 # Go to side init position (-0.7, 0.3)
CMD_ALIGN_YAW_TO_ZERO    = 18 # Rotate in place to align yaw to 0
CMD_FIND_AND_GO_CLEARING = 19 # Find clearing and move to it
CMD_WANDER_RANDOMLY      = 20 # Wander like a pet

# ===== Utils =====
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def get_yaw_from_quaternion(quat) -> float:
    return euler_from_quaternion(quat)[2]

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def yaw_to_quat(yaw: float):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, normalize_angle(yaw))
    return qx, qy, qz, qw

def make_goal_pose(x: float, y: float, yaw: float, node: Node) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quat(yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg

def euler_from_quaternion(q):
    try:
        x, y, z, w = q.x, q.y, q.z, q.w
    except AttributeError:
        # tuple/list 형태로 들어온 경우
        x, y, z, w = q

    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


# ===== Output kinds & results =====
class StateOutputKind(Enum):
    NONE  = 0
    TWIST = 1
    GOAL  = 2

class StateResult:
    CONTINUE = 0
    COMPLETE = 1


# ===== State base =====
class ControllerState:
    def __init__(self, controller: "MobileActionController"):
        self.controller = controller

    def update(self, current_pose: PoseStamped):
        raise NotImplementedError("update() must be implemented by subclasses")


# ===== Concrete states =====
class IdleState(ControllerState):
    def update(self, current_pose: PoseStamped):
        # IDLE 상태는 즉시 완료(COMPLETE)로 처리하여 'done' 메시지를 보내도록 함
        return (StateOutputKind.NONE, None, StateResult.COMPLETE)


class InitHome(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.start_time = None   # 시작 시간
        self.phase = 0           # 0=전진, 1=회전
        self.forward_time = 2.0

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()

        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9  # 초 단위

        t = Twist()
        if self.phase == 0:
            # 2초 동안 직진
            if elapsed < self.forward_time:
                t.linear.x = 0.3
                t.angular.z = 0.0
                return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
            else:
                # 전진 끝나면 회전 phase로 전환
                self.phase = 1
                self.start_time = now
                return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)

        elif self.phase == 1:
            # 3초 동안 제자리 회전
            if elapsed < 2.0:
                t.linear.x = 0.0
                t.angular.z = 1.57
                return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
            else:
                # 회전 종료
                return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)


class SpinOnce(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.start_time = None
        self.angular_velocity = 1.9  # rad/s
        # 한바퀴(2*pi rad) / 1.5 rad/s = 약 4.19초
        self.spin_duration = 5.0

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()
        
        if self.start_time is None:
            self.start_time = now
            self.controller.get_logger().info(
                f"SpinOnce: Starting 360° rotation at {self.angular_velocity} rad/s for {self.spin_duration:.2f}s"
            )
        
        elapsed = (now - self.start_time).nanoseconds / 1e9  # 초 단위
        
        # COMPLETE condition: 정해진 시간이 지나면 완료
        if elapsed >= self.spin_duration:
            if self.controller.debug_mode:
                self.controller.get_logger().info(f"SpinOnce: Rotation complete after {elapsed:.2f}s")
            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
        
        # CONTINUE rotating
        t = Twist()
        t.angular.z = self.angular_velocity
        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

class GoToPreAlignPose(ControllerState):
    def __init__(self, controller):
        self.y_offset = 0.0
        super().__init__(controller)
        
        # 목표 위치
        self.target_x = -1.35
        self.target_y = 0.0 + self.y_offset
        self.target_yaw = 0.0
        
        # 제어 파라미터
        self.linear_speed = 0.2
        self.angular_speed = 1.2
        self.min_angular_speed = 0.5
        self.success_count = 0
        self.required_success_count = 3
        self.log_counter = 0
        
        # 초기 거리 체크용
        self.initial_check_done = False
        self.skip_threshold = 0.3
        self.final_approach_threshold = 0.15  # 이 거리보다 가까우면 최종 각도만 조정 ← 추가!

    def update(self, current_pose: PoseStamped):
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        # 목표까지의 거리와 방향
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_goal - current_yaw)
        yaw_error = normalize_angle(self.target_yaw - current_yaw)
        
        # 초기 위치 체크
        if not self.initial_check_done:
            self.initial_check_done = True
            if distance < self.skip_threshold:
                self.controller.get_logger().info(
                    f"[GoToPreAlignPose] 시작 위치가 목표에 충분히 가까움 ({distance:.3f}m). 즉시 통과!"
                )
                return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
        
        # 로그
        self.log_counter += 1
        if self.controller.debug_mode and (self.log_counter % 50 == 0 or self.success_count > 0):
            self.controller.get_logger().info(
                f"[GoToPreAlignPose] pos:({current_x:.2f},{current_y:.2f}) "
                f"dist:{distance:.2f}m heading_err:{math.degrees(heading_error):.1f}° "
                f"yaw_err:{math.degrees(yaw_error):.1f}° success:{self.success_count}/{self.required_success_count}"
            )
        
        t = Twist()
        
        # ===== 매우 가까우면 (0.15m 이내) heading 무시하고 최종 yaw만 조정 =====
        if distance < self.final_approach_threshold:
            t.linear.x = 0.0  # 움직이지 않음
            
            # 최종 yaw 조정
            if abs(yaw_error) > self.controller.angle_tolerance:
                angular_magnitude = max(abs(yaw_error * 1.5), self.min_angular_speed)
                t.angular.z = math.copysign(angular_magnitude, yaw_error)
                t.angular.z = clamp(t.angular.z, -self.angular_speed, self.angular_speed)
            else:
                t.angular.z = 0.0
                self.success_count += 1
                if self.success_count >= self.required_success_count:
                    if self.controller.debug_mode:
                        self.controller.get_logger().info("[GoToPreAlignPose] ✓ Complete!")
                    return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
            
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
        
        # ===== 완료 조건 (일반) =====
        if (distance < self.controller.distance_tolerance and
            abs(yaw_error) < self.controller.angle_tolerance):
            self.success_count += 1
            if self.success_count >= self.required_success_count:
                if self.controller.debug_mode:
                    self.controller.get_logger().info("[GoToPreAlignPose] ✓ Complete!")
                return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
            else:
                return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        else:
            self.success_count = 0
        
        # ===== 제어 로직 (거리가 있을 때) =====
        if distance > 0.5:
            if abs(heading_error) > math.radians(45):
                # 제자리 회전
                t.linear.x = 0.0
                t.angular.z = clamp(1.5 * heading_error, -self.angular_speed, self.angular_speed)
            else:
                # 전진 + 조향
                t.linear.x = min(self.linear_speed, distance * 0.5)
                t.angular.z = clamp(heading_error, -self.angular_speed * 0.5, self.angular_speed * 0.5)
        else:
            # 0.15~0.5m: 천천히 접근
            t.linear.x = 0.1
            if abs(heading_error) < math.radians(90):
                t.angular.z = clamp(0.3 * heading_error, -0.4, 0.4)
            else:
                t.linear.x = 0.0
                t.angular.z = clamp(heading_error, -0.6, 0.6)
        
        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)


# class AlignToStation(ControllerState):
#     def __init__(self, controller):
#         super().__init__(controller)
#         self.y_offset = -0.01
        
#         # 목표 위치 (map 기준)
#         self.target_x = -0.63
#         self.target_y = 0.0 + self.y_offset
#         self.target_yaw = 0.0
        
#         # 제어 파라미터
#         self.linear_speed = 0.2      # 전진 속도 (m/s)
#         self.angular_speed = 1.2      # 회전 속도 (rad/s) - 더 빠르게
#         self.min_angular_speed = 0.6  # 최소 회전 속도 (데드존 극복)
#         self.position_kp = 0.7        # 위치 제어 게인
#         self.heading_kp = 1.5         # 방향 제어 게인 - 1.5는 정상 배터리
        
#         self.success_count = 0
#         self.required_success_count = 5  # 5번 연속 성공
#         self.log_counter = 0
#         self.start_time = None
#         self.timeout = 10.0

#     def update(self, current_pose: PoseStamped):
#         self.start_time = self.controller.get_clock().now()
#         # 현재 위치와 목표 위치
#         current_x = current_pose.pose.position.x
#         current_y = current_pose.pose.position.y
#         current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
#         # 목표까지의 거리와 방향
#         dx = self.target_x - current_x
#         dy = self.target_y - current_y
#         distance = math.sqrt(dx**2 + dy**2)
#         angle_to_goal = math.atan2(dy, dx)
        
#         # 목표 yaw와의 차이
#         yaw_error = normalize_angle(self.target_yaw - current_yaw)
#         heading_error = normalize_angle(angle_to_goal - current_yaw)
        
#         # 로그 (100번에 한 번)
#         self.log_counter += 1
#         if self.controller.debug_mode and (self.log_counter % 100 == 0 or self.success_count > 0):
#             self.controller.get_logger().info(
#                 f"[AlignToStation] pos:({current_x:.2f},{current_y:.2f}) "
#                 f"dist:{distance:.2f}m yaw_err:{math.degrees(yaw_error):.1f}° "
#                 f"success:{self.success_count}/{self.required_success_count}"
#             )
        
#         t = Twist()

#         # Timeout check
#         if self.start_time:
#             elapsed_seconds = (self.controller.get_clock().now() - self.start_time).nanoseconds / 1e9
#             # self.controller.get_logger().info(f"[AlignToStation] Elapsed time: {elapsed_seconds} seconds")
#             if elapsed_seconds > self.timeout:
#                 self.controller.get_logger().warn("[AlignToStation] Timed out after 10 seconds. Forcing completion.")
#                 return (StateOutputKind.NONE, None, StateResult.COMPLETE)
#         # 완료 조건
#         if (distance < 0.07 and
#             abs(yaw_error) < self.controller.angle_tolerance):
#             self.success_count += 1
#             if self.success_count >= self.required_success_count:
#                 if self.controller.debug_mode:
#                     self.controller.get_logger().info("[AlignToStation] ✓ Complete!")
#                 return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
#             else:
#                 return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
#         else:
#             self.success_count = 0
        
#         # 제어 로직
#         if distance > self.controller.distance_tolerance:
#             # 목표 방향과 차이가 크면 회전 우선
#             if abs(heading_error) > math.radians(30):
#                 # 제자리 회전
#                 t.linear.x = 0.0
#                 raw_angular = self.heading_kp * heading_error
#                 angular_magnitude = max(abs(raw_angular), self.min_angular_speed)
#                 t.angular.z = math.copysign(angular_magnitude, heading_error)
#                 t.angular.z = clamp(t.angular.z, -self.angular_speed, self.angular_speed)
#             else:
#                 # 전진 + 조향
#                 t.linear.x = min(self.linear_speed, self.position_kp * distance)
#                 t.angular.z = self.heading_kp * heading_error * 0.5
#                 t.angular.z = clamp(t.angular.z, -self.angular_speed * 0.5, self.angular_speed * 0.5)
#         else:
#             # 위치 도달, 최종 각도 조정
#             t.linear.x = 0.0
            
#             if abs(yaw_error) > self.controller.angle_tolerance:
#                 raw_angular = yaw_error * 2.5  # 게인 2.0 → 2.5로 증가
#                 angular_magnitude = max(abs(raw_angular), self.min_angular_speed)
#                 t.angular.z = math.copysign(angular_magnitude, yaw_error)
#                 t.angular.z = clamp(t.angular.z, -self.angular_speed, self.angular_speed)
#             else:
#                 t.angular.z = 0.0
        
#         return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
class AlignToStation(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.goal_sent = False
        self.target_x = -0.63
        self.target_y = -0.02
        self.target_yaw = 0.0
        self.start_time = None
        self.timeout = 7.0

    def update(self, current_pose: PoseStamped):
        if not self.goal_sent:
            self.start_time = self.controller.get_clock().now()
            goal = make_goal_pose(self.target_x, self.target_y, self.target_yaw, self.controller)
            self.goal_sent = True
            if self.controller.debug_mode:
                self.controller.get_logger().info(f"AlignToStation: Going to side init position ({self.target_x}, {self.target_y})")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)
        
        self.controller.get_logger().info(f"AlignToStation: Start time: {self.start_time}")
        
        # Timeout check
        if self.start_time:
            elapsed_seconds = (self.controller.get_clock().now() - self.start_time).nanoseconds / 1e9
            # self.controller.get_logger().info(f"AlignToStation: Elapsed time: {elapsed_seconds} seconds")
            if elapsed_seconds > self.timeout:
                self.controller.get_logger().warn("GoToSidAlignToStationeInit: Timed out after 10 seconds. Forcing completion.")
                return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        
        # 목표 지점 도달 확인
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        distance = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)
        yaw_error = normalize_angle(self.target_yaw - current_yaw)
        
        if distance < 0.05 and abs(yaw_error) < self.controller.angle_tolerance:
            if self.controller.debug_mode:
                self.controller.get_logger().info("AlignToStation: Reached side init position!")
            return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        else:
            return (StateOutputKind.NONE, None, StateResult.CONTINUE)

class ApproachStation(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        # 기본 게인/리밋
        self.k_ang = 1.2
        self.k_lin = 0.5
        self.max_ang = 1.8          # ↑ 1.8로 상향
        self.max_lin = controller.aruco_max_lin_near  # 0.25 하드코딩 대신 파라미터 사용

        # 회전/전진 데드존 보정
        self.min_ang_align = 0.5    # 정렬용 최소 각속도
        self.min_ang_move  = 0.3    # 전진 중 각보정 최소 각속도
        self.turn_activation_margin = 0.02  # deadband 밖에서 최소 여유

        self.min_lin_ff = 0.06      # 최소 전진 속도(정지마찰 극복)

        # 바이어스/파라미터(네 값 그대로)
        self.center_x_bias = controller.aruco_center_x_bias
        self.align_deadband_in  = controller.aruco_center_deadband_in
        self.align_deadband_out = controller.aruco_center_deadband_out
        self.align_ok_needed_frames = controller.aruco_align_ok_frames
        self.target_area = controller.aruco_target_area
        self.min_complete_ratio = controller.aruco_min_complete_ratio
        self.max_safe_area = controller.aruco_max_safe_area
        self.max_ang_adv = controller.aruco_max_ang_adv

        self.phase = "ALIGN"
        self.align_ok_frames = 0
        self.cx_ema = None
        self.cx_alpha = 0.3
        self.debug_counter = 0

    def _clamp(self, x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def _apply_ang_deadzone(self, cx, raw_cmd, deadband, min_mag):
        """
        deadband 밖이면 최소 각속도(min_mag)로 보정해 주고,
        deadband 경계 바로 바깥에서는 튀지 않도록 소량의 여유를 둔다.
        """
        if abs(cx) <= deadband:
            return 0.0
        # deadband를 살짝 벗어난 경우 곧바로 큰 속도가 걸리는 걸 방지
        if abs(cx) <= deadband + self.turn_activation_margin:
            return math.copysign(min_mag, raw_cmd)
        # 일반 구간: 최소크기 보장 + 상한 클램프
        mag = max(min_mag, abs(raw_cmd))
        return math.copysign(self._clamp(mag, 0.0, self.max_ang), raw_cmd)

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()
        t = Twist()

        # 시야 체크
        if (self.controller.aruco_center is None or
            (now.nanoseconds - self.controller.aruco_center_stamp.nanoseconds) / 1e9 > self.controller.aruco_seen_timeout):
            self.phase = "ALIGN"; self.align_ok_frames = 0
            return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)

        cx_raw = float(self.controller.aruco_center.x)
        area   = float(self.controller.aruco_center.z)
        cx = cx_raw - self.center_x_bias

        target_area = self.target_area
        deadband_in = self.align_deadband_in
        deadband_out = self.align_deadband_out

        # 너무 가까우면 즉시 완료
        if area >= self.max_safe_area:
            if self.controller.debug_mode:
                self.controller.get_logger().info("ApproachStation: Area is too close to the aruco marker. Completing the action.")
            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)

        # ===== ALIGN: 회전만 (최소 0.5 rad/s 보장) =====
        if self.phase == "ALIGN":
            raw_ang = -self.k_ang * cx
            t.angular.z = self._apply_ang_deadzone(cx, raw_ang, deadband_in, self.min_ang_align)
            t.linear.x  = 0.0

            if abs(cx) <= deadband_in:
                self.align_ok_frames += 1
                if self.align_ok_frames >= self.align_ok_needed_frames:
                    self.phase = "ADVANCE"
            else:
                self.align_ok_frames = 0

            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

        # ===== ADVANCE: 전진 + 소프트 보정 (최소 0.3 rad/s 보장, 필요시만) =====
        # 정렬 크게 무너지면 ALIGN로
        if abs(cx) > deadband_out:
            self.phase = "ALIGN"; self.align_ok_frames = 0
            raw_ang = -self.k_ang * cx
            t.angular.z = self._apply_ang_deadzone(cx, raw_ang, deadband_in, self.min_ang_align)
            t.linear.x  = 0.0
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

        # 각속도 (소프트 보정하지만, 데드존은 0.3로 보장)
        raw_ang = -0.6 * self.k_ang * cx
        # deadband_in 안쪽이면 굳이 돌리지 않음(0). 그 밖이면 최소 0.3 보장.
        if abs(cx) <= deadband_in:
            t.angular.z = 0.0
        else:
            t.angular.z = self._apply_ang_deadzone(cx, raw_ang, deadband_in, self.min_ang_move)
            # 상한을 ADVANCE용으로 더 낮추고 싶다면:
            t.angular.z = self._clamp(t.angular.z, -self.max_ang_adv, self.max_ang_adv)

        # 선속도 (정규화 + 바닥 0.06 m/s)
        err_ratio = max((target_area - area) / target_area, 0.0)  # 0..1
        v_ff = self.min_lin_ff
        v_cmd = v_ff + (self.max_lin - v_ff) * err_ratio
        t.linear.x = self._clamp(v_cmd, 0.0, self.max_lin)

        # 완료 조건
        horizontal_aligned = abs(cx) <= deadband_in
        distance_ok = (area >= target_area * self.min_complete_ratio) or (area >= self.max_safe_area)
        if horizontal_aligned and distance_ok:
            if self.controller.debug_mode:
                self.controller.get_logger().info("ApproachStation: Horizontal aligned and distance ok. Completing the action.")
                self.controller.get_logger().info(f"ApproachStation: Area: {area}, Target Area: {target_area}, Min Complete Ratio: {self.min_complete_ratio}, Max Safe Area: {self.max_safe_area}")
            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)

        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

class GoToMiddle(ControllerState):
    """
    단순 shimmy 방식:
    EVAL(평가/소회전 서보) -> ROTATE_OUT(±θ) -> DRIVE(s 후진) -> ROTATE_BACK(∓θ) -> 다시 EVAL
    목표: cx가 deadband_in 안으로 N프레임 연속 들어오면 COMPLETE
    """

    def __init__(self, controller):
        super().__init__(controller)

        # ---- 기준/파라미터 ----
        self.center_x_bias   = controller.aruco_center_x_bias
        self.deadband_in     = controller.aruco_center_deadband_in
        self.deadband_out    = controller.aruco_center_deadband_out
        self.ok_frames_need  = controller.aruco_align_ok_frames
        self.seen_timeout    = controller.aruco_seen_timeout

        # "소회전 서보(ALIGN서보)" 파라미터 – 큰 shimmy 없이 정밀하게 중앙으로
        self.k_ang_align     = controller.gtm_k_ang_align
        self.min_ang_align   = controller.gtm_min_ang_align
        self.max_ang_align   = controller.gtm_max_ang_align

        # shimmy 파라미터(시간 기반; 각·거리/속도에서 파생)
        self.theta_min_deg   = controller.gtm_theta_min_deg
        self.theta_max_deg   = controller.gtm_theta_max_deg
        self.step_min_m      = controller.gtm_step_min_m
        self.step_max_m      = controller.gtm_step_max_m
        self.k_theta_scale   = controller.gtm_k_theta_scale
        self.k_step_scale    = controller.gtm_k_step_scale

        self.ang_speed       = controller.gtm_ang_speed
        self.lin_speed       = controller.gtm_lin_speed

        # 너무 가까우면 shimmy 스텝을 줄이기 위한 안전기준(있으면 사용)
        self.max_safe_area   = controller.aruco_max_safe_area

        # ---- 상태 ----
        self.phase = "EVAL"           # EVAL / ROTATE_OUT / DRIVE / ROTATE_BACK
        self.align_ok_frames = 0
        self._shimmy_dir = 0.0        # +1(left)/-1(right)
        self._until_t   = 0.0         # 현재 서브동작 종료 시각(sec)
        self._theta_rad = 0.0         # 현재 사이클 목표 회전각
        self._step_m    = 0.0         # 현재 사이클 목표 전진거리

    # ---------------- 유틸 ----------------
    def _now(self):
        return self.controller.get_clock().now().nanoseconds / 1e9

    def _clamp(self, x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def _apply_ang_deadzone(self, cx, raw_cmd, min_mag, max_mag):
        if abs(cx) <= self.deadband_in:
            return 0.0
        mag = max(min_mag, abs(raw_cmd))
        return math.copysign(self._clamp(mag, 0.0, max_mag), raw_cmd)

    # ---------------- 메인 ----------------
    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()
        t = Twist()

        # 마커 시야 체크
        if (self.controller.aruco_center is None or
            (now.nanoseconds - self.controller.aruco_center_stamp.nanoseconds)/1e9 > self.seen_timeout):
            # 안 보이면 정지 + 초기화
            self.phase = "EVAL"; self.align_ok_frames = 0
            return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)

        cx = float(self.controller.aruco_center.x) - self.center_x_bias
        area = float(self.controller.aruco_center.z)

        # ---- EVAL: 소회전 서보로 중앙 붙여보고, 많이 어긋나면 shimmy 시작 ----
        if self.phase == "EVAL":
            if self.controller.debug_mode:
                self.controller.get_logger().info(f"GoToMiddle: EVAL: cx: {cx}, area: {area}")
            # 1) 중앙 판정
            if abs(cx) <= self.deadband_in:
                self.align_ok_frames += 1
                if self.align_ok_frames >= self.ok_frames_need:
                    return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
            else:
                self.align_ok_frames = 0

            # 2) 조금만 어긋나면(<= deadband_out) 회전 서보로 정렬 시도
            if abs(cx) <= self.deadband_out:
                raw_ang = -self.k_ang_align * cx
                t.angular.z = self._apply_ang_deadzone(cx, raw_ang, self.min_ang_align, self.max_ang_align)
                t.linear.x  = 0.0
                return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

            # 3) 많이 어긋나면 shimmy 사이클 진입
            #    cx>0(마커가 화면 오른쪽)이면 오른쪽으로 "옆이동"해야 하므로 θ는 양수(반시계)로.
            self._shimmy_dir = 1.0 if cx > 0 else -1.0

            # |cx|를 0~1로 노멀라이즈했다고 가정하고 스케일 (실전: 0.15~0.2부근 cap 느낌)
            cx_scale = self._clamp(abs(cx) / 0.15, 0.0, 1.0)

            theta_deg = (self.theta_min_deg +
                         (self.theta_max_deg - self.theta_min_deg) * (self.k_theta_scale * cx_scale))
            step_m    = (self.step_min_m +
                         (self.step_max_m - self.step_min_m) * (self.k_step_scale * cx_scale))

            # 너무 가까우면 스텝 축소(있는 경우)
            if self.max_safe_area is not None and area >= 0.7 * self.max_safe_area:
                step_m *= 0.6   # 살짝만 후진

            self._theta_rad = math.radians(theta_deg)
            self._step_m    = step_m

            # ROTATE_OUT 시작 (시간기반)
            dur = abs(self._theta_rad) / max(self.ang_speed, 1e-6)
            self._until_t = self._now() + dur
            self.phase = "ROTATE_OUT"
            # 바로 해당 동작을 수행
            t.angular.z = self._shimmy_dir * self.ang_speed
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

        # ---- ROTATE_OUT: dir * θ 만큼 회전 ----
        if self.phase == "ROTATE_OUT":
            t.angular.z = self._shimmy_dir * self.ang_speed
            if self._now() >= self._until_t:
                # DRIVE로 전환
                dur = self._step_m / max(self.lin_speed, 1e-6)
                self._until_t = self._now() + dur
                self.phase = "DRIVE"
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

        # ---- DRIVE: 후진 s (각속도 0으로 유지 → 측이동 효과 극대화) ----
        if self.phase == "DRIVE":
            t.linear.x  = -self.lin_speed
            t.angular.z = 0.0
            if self._now() >= self._until_t:
                # ROTATE_BACK으로 전환
                dur = abs(self._theta_rad) / max(self.ang_speed, 1e-6)
                self._until_t = self._now() + dur
                self.phase = "ROTATE_BACK"
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

        # ---- ROTATE_BACK: -dir * θ 만큼 원래 헤딩 복구 ----
        if self.phase == "ROTATE_BACK":
            t.angular.z = -self._shimmy_dir * self.ang_speed
            if self._now() >= self._until_t:
                # 한 사이클 끝 → 다시 평가
                self.phase = "EVAL"
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)

        # 예외
        self.phase = "EVAL"
        return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
# class RotateBehind(ControllerState):
#     def __init__(self, controller):
#         super().__init__(controller)
#         self.target_yaw = None
#         self.angular_speed = 1.0  # rad/s (낮은 값 = 천천히 회전, 기본 0.3)

#     def update(self, current_pose: PoseStamped):
#         # 현재 yaw
#         yaw_now = get_yaw_from_quaternion(current_pose.pose.orientation)

#         if self.target_yaw is None:
#             # 최초에 목표 yaw 계산 (현재 yaw + 175도)
#             self.target_yaw = normalize_angle(yaw_now + math.radians(175))

#         # 목표 yaw와의 차이
#         yaw_error = normalize_angle(self.target_yaw - yaw_now)

#         # COMPLETE 판정: yaw 오차가 angle_tolerance 이하이면 완료
#         if abs(yaw_error) < self.controller.angle_tolerance:
#             return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
        
#         # 회전 방향 결정 (yaw_error의 부호로 결정)
#         t = Twist()
#         t.linear.x = 0.0
#         t.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
        
#         return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
class RotateBehind(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.goal_sent = False
        self.target_x = -1.0
        self.target_y = -0.0
        self.target_yaw = 0.0

    def update(self, current_pose: PoseStamped):
        # 목표 지점 도달 확인
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)

        if not self.goal_sent:
            self.target_x = current_x
            self.target_y = current_y
            # self.target_yaw = current_yaw + 3.14
            self.target_yaw = 3.14
            # self.target_yaw = 2.96706 #노배터리
            goal = make_goal_pose(self.target_x , self.target_y, self.target_yaw, self.controller)
            self.goal_sent = True
            self.controller.get_logger().info(f"current_x: {current_x}, current_y: {current_y}, current_yaw: {current_yaw}!")
            self.controller.get_logger().info(f"target_x: {self.target_x}, target_y: {self.target_y}, target_yaw: {self.target_yaw}!")
            if self.controller.debug_mode:
                self.controller.get_logger().info(f"RotateBehind: Going to ({self.target_x}, {self.target_y})")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)
        
        distance = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)
        yaw_error = normalize_angle(self.target_yaw - current_yaw)
        
        # if distance > self.controller.distance_tolerance:
        #     self.controller.get_logger().info("RotateBehind: Not Reached target position!")
        #     self.controller.get_logger().info(f"distance: {distance}!")
        
        # if abs(yaw_error) > self.controller.angle_tolerance:
        #     self.controller.get_logger().info("RotateBehind: Not Reached target angle!")
        #     self.controller.get_logger().info(f"yaw_error: {yaw_error}!")

        if distance < self.controller.distance_tolerance and abs(yaw_error) < self.controller.angle_tolerance:
            if self.controller.debug_mode:
                self.controller.get_logger().info("RotateBehind: Reached target position!")
            return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        else:
            return (StateOutputKind.NONE, None, StateResult.CONTINUE)


class GoBack(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.start_time = None   # 시작 시간
        self.backward_time = 1.0 # 후진 지속 시간 (초)

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()

        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9  # 초 단위 경과시간

        t = Twist()
        if elapsed < self.backward_time:
            t.linear.x = -0.2
            t.angular.z = 0.0
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
        else:

            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)


class GoToSideLeft(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.goal_sent = False
        self.target_x = -1.0
        self.target_y = 0.85
        self.target_yaw = 0.0

    def update(self, current_pose: PoseStamped):
        if not self.goal_sent:
            goal = make_goal_pose(self.target_x, self.target_y, self.target_yaw, self.controller)
            self.goal_sent = True
            if self.controller.debug_mode:
                self.controller.get_logger().info(f"GoToSideLeft: Going to ({self.target_x}, {self.target_y})")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)
        
        # 목표 지점 도달 확인
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        distance = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)
        yaw_error = normalize_angle(self.target_yaw - current_yaw)

        
        
        if distance < self.controller.distance_tolerance and abs(yaw_error) < self.controller.angle_tolerance:
            if self.controller.debug_mode:
                self.controller.get_logger().info("GoToSideLeft: Reached target position!")
            return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        else:
            return (StateOutputKind.NONE, None, StateResult.CONTINUE)


class GoToSideRight(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.goal_sent = False
        self.target_x = -1.0
        self.target_y = -0.85
        self.target_yaw = 0.0

    def update(self, current_pose: PoseStamped):
        if not self.goal_sent:
            goal = make_goal_pose(self.target_x, self.target_y, self.target_yaw, self.controller)
            self.goal_sent = True
            if self.controller.debug_mode:
                self.controller.get_logger().info(f"GoToSideRight: Going to ({self.target_x}, {self.target_y})")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)
        
        # 목표 지점 도달 확인
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        distance = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)
        yaw_error = normalize_angle(self.target_yaw - current_yaw)
        
        if distance < self.controller.distance_tolerance and abs(yaw_error) < self.controller.angle_tolerance:
            if self.controller.debug_mode:
                self.controller.get_logger().info("GoToSideRight: Reached target position!")
            return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        else:
            return (StateOutputKind.NONE, None, StateResult.CONTINUE)


class FindStationForInit(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.attempt = 1          # 현재 시도 횟수
        self.max_attempts = 3     # 최대 시도 횟수
        self.phase = "ROTATING"   # 현재 단계: "ROTATING" or "MOVING_BACK"
        self.start_time = None
        
        # 각 단계별 지속 시간 (초)
        self.rotation_duration = 14.0 # 360도 회전 (0.45 rad/s * 14s ~= 6.3 rad)
        self.move_back_duration = 1.0 # 10cm 후진 (-0.1 m/s * 1s)

    def update(self, current_pose: PoseStamped):
        # 1. 성공 조건: 동작 중 아루코 마커가 한 번이라도 보였다면 즉시 성공
        if self.controller.aruco_marker_seen_in_find_station:
            # self.controller.get_logger().info("FindStationForInit: Aruco marker detected. Success!")
            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)

        now = self.controller.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            # self.controller.get_logger().info(f"FindStationForInit: Starting attempt {self.attempt}/{self.max_attempts}, phase: {self.phase}")

        elapsed = (now - self.start_time).nanoseconds / 1e9
        t = Twist()

        # 2. 현재 단계(phase)에 따라 동작 수행
        if self.phase == "ROTATING":
            if elapsed < self.rotation_duration:
                t.angular.z = 0.45
                return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
            else:
                # 회전 종료
                self.start_time = None
                if self.attempt >= self.max_attempts:
                    # 모든 시도 실패
                    # self.controller.get_logger().warn("FindStationForInit: All attempts failed.")
                    return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
                else:
                    # 다음 단계 (후진)로 전환
                    self.phase = "MOVING_BACK"
                    return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
        elif self.phase == "MOVING_BACK":
            if elapsed < self.move_back_duration:
                t.linear.x = 0.2
                return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
            else:
                # 후진 종료, 다음 시도(회전) 준비
                self.start_time = None
                self.attempt += 1
                self.phase = "ROTATING"
                return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)


class FindStationForCharge(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.goal_sent = False
        self.target_x = -1.0
        self.target_y = 0.0

    def update(self, current_pose: PoseStamped):
        # 충전소 위치 (0, 0, 0) 로 이동
        if not self.goal_sent:
            goal = make_goal_pose(self.target_x, self.target_y, 0.0, self.controller)
            self.goal_sent = True # goal_sent 플래그가 없으면 매번 goal을 보내게 되어 수정했습니다.
            # self.controller.get_logger().info("FindStationForCharge: Going to charging station at (0, 0, 0)")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)
        
        # 목표 지점 도달 확인
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        distance = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)
        
        if distance < self.controller.distance_tolerance:
            # self.controller.get_logger().info("FindStationForCharge: Reached charging station!")
            return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        else:
            return (StateOutputKind.NONE, None, StateResult.CONTINUE)


class MoveRandomly(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.trial_count = 0
        self.trial_max_count = 20

        # 랜덤 이동 파라미터
        self._RANDOM_STEP_MIN = 0.5
        self._RANDOM_STEP_MAX = 1.5
        self._YAW_JITTER_MAX = math.radians(60.0)

        # 레이저 단발 트리거(마진 없이 임계치 기준)
        self.laser_escape_armed = True  # True일 때만 한 번 반응

    def get_new_random_target(self, cx: float, cy: float, cyaw: float):
        self.trial_count += 1
        step = random.uniform(self._RANDOM_STEP_MIN, self._RANDOM_STEP_MAX)
        yaw_jitter = random.uniform(-self._YAW_JITTER_MAX, self._YAW_JITTER_MAX)
        nyaw = normalize_angle(cyaw + yaw_jitter)
        nx = cx + step * math.cos(nyaw)
        ny = cy + step * math.sin(nyaw)
        return -nx, ny, nyaw 

    def update(self, current_pose: PoseStamped):
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)  # rad

        # 0) 초기 목표 생성
        if self.target_x is None or self.target_y is None or self.target_yaw is None:
            rx, ry, ryaw = self.get_new_random_target(current_x, current_y, current_yaw)
            self.target_x, self.target_y, self.target_yaw = rx, ry, ryaw
            goal = make_goal_pose(rx, ry, ryaw, self.controller)
            # self.controller.get_logger().info( f"First New random goal -> x: {rx:.2f}, y: {ry:.2f}, yaw: {ryaw:.2f}")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)

        # 1) 도착 판정
        dist = math.hypot(current_x - self.target_x, current_y - self.target_y)
        yaw_err = normalize_angle(self.target_yaw - current_yaw)
        reached = (dist < self.controller.distance_tolerance and
                   abs(yaw_err) < self.controller.angle_tolerance)

        # 2) 레이저 상태 (임계치만 사용, 길이 안전)
        lv = getattr(self.controller, "laser_sensor_value", [])
        th = self.controller.laser_threshold
        below = (len(lv) >= 2) and (lv[0] < th or lv[1] < th)
        above = (len(lv) >= 2) and (lv[0] >= th and lv[1] >= th)

        # 임계치 이상으로 회복되면 재무장
        if above:
            self.laser_escape_armed = True

        # 3-a) 도착 → 새 랜덤 목표
        if reached:
            rx, ry, ryaw = self.get_new_random_target(current_x, current_y, current_yaw)
            self.target_x, self.target_y, self.target_yaw = rx, ry, ryaw
            goal = make_goal_pose(rx, ry, ryaw, self.controller)
            # 도착 후에는 레이저 반응 가능하게 재무장(선택)
            self.laser_escape_armed = True
            # self.controller.get_logger().info( f"Reach New random goal -> x: {rx:.2f}, y: {ry:.2f}, yaw: {ryaw:.2f}")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)

        # 3-b) 도착 못했어도: 레이저 임계 이하 & 무장 상태면 → 단 한 번 새 goal 생성
        if below and self.laser_escape_armed:
            self.laser_escape_armed = False  # 단발 처리
            rx, ry, ryaw = self.get_new_random_target(current_x, current_y, current_yaw)
            self.target_x, self.target_y, self.target_yaw = rx, ry, ryaw
            goal = make_goal_pose(rx, ry, ryaw, self.controller)
            # self.controller.get_logger().info( f"laser New random goal -> x: {rx:.2f}, y: {ry:.2f}, yaw: {ryaw:.2f}")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)

        # 4) 종료 조건
        # aruco_seen = bool(getattr(self.controller, "aruco_marker_seen_in_find_station", False))
        if (self.trial_count >= self.trial_max_count) or aruco_seen:
            return (StateOutputKind.NONE, Twist(), StateResult.COMPLETE)

        # 계속 진행
        return (StateOutputKind.NONE, Twist(), StateResult.CONTINUE)


class SearchAruco(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.controller.get_logger().info("SearchAruco: Starting rotational search for stable markers.")
        
        # Rotation tracking
        self.start_yaw = None
        self.last_yaw = None
        self.accumulated_rotation = 0.0
        self.angular_velocity = 1.0 # rad/s

        # Stability check
        self.stable_frames_needed = 10  # Require 10 consecutive frames with >= 2 markers
        self.stable_frame_count = 0

    def update(self, current_pose: PoseStamped):
        t = Twist()

        # 1. Success condition check
        # Check if we see 2 or more markers.
        if self.controller.visible_marker_count >= 2:
            self.stable_frame_count += 1
            self.controller.get_logger().info(f"SearchAruco: >=2 markers detected. Stability count: {self.stable_frame_count}/{self.stable_frames_needed}")
            
            # If we meet the stability criteria, stop and complete.
            if self.stable_frame_count >= self.stable_frames_needed:
                self.controller.get_logger().info(f"SearchAruco: Markers are stable for {self.stable_frames_needed} frames. Task complete.")
                return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
            else:
                # We see markers, but they are not yet stable. Stop rotating to check stability.
                return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
        # 2. Not enough markers are visible. Reset stability and continue searching.
        else:
            if self.stable_frame_count > 0:
                # This log is useful to know if we found a candidate spot but it was unstable.
                self.controller.get_logger().info(f"SearchAruco: Marker visibility lost. Resetting stability count.")
            self.stable_frame_count = 0
        
        # --- Continue rotation logic ---
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        if self.start_yaw is None:
            self.start_yaw = current_yaw
            self.last_yaw = current_yaw
        
        # Calculate accumulated rotation to handle angle wrap-around correctly
        delta_yaw = normalize_angle(current_yaw - self.last_yaw)
        self.accumulated_rotation += abs(delta_yaw)
        self.last_yaw = current_yaw
        
        # 3. Failure condition: Increased to 1 full rotations to allow time for stopping and checking
        if self.accumulated_rotation >= (2 * math.pi):
            self.controller.get_logger().warn("SearchAruco: 1 full rotations complete, stable markers not found. Failing.")
            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
        
        # 4. Keep rotating
        t.angular.z = self.angular_velocity
        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)


class ApproachMarker(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.phase = "SEARCHING"  # SEARCHING -> APPROACHING
        
        # 접근 제어 파라미터
        self.k_lin = 0.4         # 선형 속도 게인
        self.max_lin = 0.25      # 최대 선형 속도
        self.k_ang = 0.8         # 각속도 게인 (조향용)
        self.max_ang = 0.4       # 최대 각속도
        
        # 접근 제어
        self.search_angular_speed = 0.8  # rad/s (기존 0.3에서 상향)
        self.search_start_time = None

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()
        t = Twist()

        self.target_marker_id = self.controller.target_marker_id
        
        # 현재 시야에 타겟 마커가 보이는지 확인
        is_target_visible = False
        if (self.controller.aruco_center is not None and
            (now.nanoseconds - self.controller.aruco_center_stamp.nanoseconds) / 1e9 < self.controller.aruco_seen_timeout):
            if self._is_target_marker_visible():
                is_target_visible = True
        
        # =================================================================
        # Phase 1: SEARCHING - 목표 마커를 찾을 때까지 회전
        # =================================================================
        if self.phase == "SEARCHING":
            if not is_target_visible:
                # 아직 못 찾음, 계속 회전
                if self.search_start_time is None:
                    self.search_start_time = now
                    self.controller.get_logger().info(f"ApproachMarker: Starting search for marker {self.target_marker_id}")
                
                t.angular.z = self.search_angular_speed
                return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
            else:
                # 목표 발견! APPROACHING 단계로 전환하고 바로 아래 로직 실행
                self.controller.get_logger().info(f"ApproachMarker: Target marker {self.target_marker_id} detected! Switching to approach phase.")
                self.phase = "APPROACHING"
        
        # =================================================================
        # Phase 2: APPROACHING - 목표 마커로 접근
        # =================================================================
        if self.phase == "APPROACHING":
            if not is_target_visible:
                # 마커를 놓쳤다! 다시 SEARCHING 단계로 전환하고 즉시 회전 시작
                self.controller.get_logger().warn(f"ApproachMarker: Target marker {self.target_marker_id} lost. Returning to search phase.")
                self.phase = "SEARCHING"
                self.search_start_time = None
                t.angular.z = self.search_angular_speed
                return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
            
            # --- 목표가 보이면 접근 계속 ---
            cx = self.controller.aruco_center.point.x
            
            # 완료 조건: 목표 마커와의 거리가 0.7m 이내
            if self.controller.last_aruco_pose is not None and self.target_marker_id is not None:
                target_id = int(self.target_marker_id)
                if target_id in self.controller.aruco_marker_position:
                    marker_transform = self.controller.aruco_marker_position[target_id]
                    marker_x = marker_transform[0, 3]
                    marker_y = marker_transform[1, 3]
                    
                    aruco_x = self.controller.last_aruco_pose.pose.position.x
                    aruco_y = self.controller.last_aruco_pose.pose.position.y
                    distance = math.sqrt((aruco_x - marker_x)**2 + (aruco_y - marker_y)**2)
                    
                    self.controller.get_logger().info(f"ApproachMarker: distance to target {target_id} = {distance:.3f}m")
                    
                    if distance <= 1.8:
                        self.controller.get_logger().info(f"ApproachMarker: Reached target marker {target_id}. Complete.")
                        return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
            
            # 이동 및 조향 로직: 마커가 중앙에서 멀수록 회전을 우선시
            turn_threshold = 0.25  # 이 값보다 cx가 크면 회전 우선
            if abs(cx) > turn_threshold:
                # 선속도를 줄여 회전에 집중 (곡선 이동 유도)
                t.linear.x = self.max_lin * 0.3
                t.angular.z = clamp(-self.k_ang * cx, -self.max_ang, self.max_ang)
            else:
                # 중앙에 가까우면 전진 위주
                t.linear.x = self.max_lin
                t.angular.z = clamp(-self.k_ang * cx, -self.max_ang, self.max_ang)
            
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
        
        # Fallback in case phase logic fails
        return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
    
    def _is_target_marker_visible(self):
        """Check if the currently visible marker is the target marker"""
        if not self.target_marker_id or not self.controller.aruco_center:
            return True  # If no target specified, any marker is fine
            
        # Extract marker ID from frame_id (format: "aruco_marker_101")
        frame_id = self.controller.aruco_center.header.frame_id
        if frame_id.startswith("aruco_marker_"):
            visible_marker_id = frame_id.replace("aruco_marker_", "")
            is_match = visible_marker_id == str(self.target_marker_id)
            
            # This log is very spammy, let's comment it out.
            # if is_match:
            #     self.controller.get_logger().info(f"ApproachMarker: Target marker {self.target_marker_id} found!")
            # else:
            #     self.controller.get_logger().info(f"ApproachMarker: Found marker {visible_marker_id}, but looking for {self.target_marker_id}")
            
            return is_match
        
        return False

    def _get_visible_marker_id(self):
        """Get the ID of the currently visible marker"""
        if not self.controller.aruco_center:
            return "unknown"
            
        frame_id = self.controller.aruco_center.header.frame_id if hasattr(self.controller.aruco_center, 'header') else ""
        if frame_id.startswith("aruco_marker_"):
            return frame_id.replace("aruco_marker_", "")
        
        return "unknown"

class GoToFront(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.start_time = None   # 시작 시간
        self.forward_time = 0.8  # 전진 지속 시간 (초) - 바퀴 정렬용이므로 짧게
        self.forward_speed = 0.15 # 전진 속도 (m/s) - 천천히

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()

        if self.start_time is None:
            self.start_time = now
            self.controller.get_logger().info(f"GoToFront: Starting forward motion for {self.forward_time}s at {self.forward_speed}m/s")

        elapsed = (now - self.start_time).nanoseconds / 1e9  # 초 단위 경과시간

        t = Twist()
        if elapsed < self.forward_time:
            # 지정된 시간 동안 전진
            t.linear.x = self.forward_speed
            t.angular.z = 0.0
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
        else:
            # 시간 경과 후 → 정지 + COMPLETE
            self.controller.get_logger().info("GoToFront: Forward motion complete")
            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)

class GoToSideInit(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.goal_sent = False
        self.target_x = -0.63
        self.target_y = 0.35
        self.target_yaw = 0.0
        self.start_time = None
        self.timeout = 7.0

    def update(self, current_pose: PoseStamped):
        if not self.goal_sent:
            self.start_time = self.controller.get_clock().now()
            goal = make_goal_pose(self.target_x, self.target_y, self.target_yaw, self.controller)
            self.goal_sent = True
            if self.controller.debug_mode:
                self.controller.get_logger().info(f"GoToSideInit: Going to side init position ({self.target_x}, {self.target_y})")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)
        
        self.controller.get_logger().info(f"GoToSideInit: Start time: {self.start_time}")
        
        # Timeout check
        if self.start_time:
            elapsed_seconds = (self.controller.get_clock().now() - self.start_time).nanoseconds / 1e9
            # self.controller.get_logger().info(f"GoToSideInit: Elapsed time: {elapsed_seconds} seconds")
            if elapsed_seconds > self.timeout:
                self.controller.get_logger().warn("GoToSideInit: Timed out after 10 seconds. Forcing completion.")
                return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        
        # 목표 지점 도달 확인
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        distance = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)
        yaw_error = normalize_angle(self.target_yaw - current_yaw)
        
        if distance < self.controller.distance_tolerance and abs(yaw_error) < self.controller.angle_tolerance:
            if self.controller.debug_mode:
                self.controller.get_logger().info("GoToSideInit: Reached side init position!")
            return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        else:
            return (StateOutputKind.NONE, None, StateResult.CONTINUE)


class AlignYawToZero(ControllerState):
    """제자리에서 yaw를 0도로 정렬"""
    def __init__(self, controller):
        super().__init__(controller)
        self.target_yaw = 0.0  # 목표 yaw는 항상 0도
        self.angular_speed = 0.8  # 회전 속도 (rad/s)
        self.min_angular_speed = 0.5  # 최소 회전 속도 (데드존 극복)
        self.success_count = 0
        self.required_success_count = 3  # 3번 연속 성공

    def update(self, current_pose: PoseStamped):
        # 현재 yaw
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        # 목표 yaw(0도)와의 차이
        yaw_error = normalize_angle(self.target_yaw - current_yaw)

        t = Twist()
        t.linear.x = 0.0  # 제자리 회전

        # 완료 조건
        if abs(yaw_error) < self.controller.angle_tolerance:
            self.success_count += 1
            if self.success_count >= self.required_success_count:
                if self.controller.debug_mode:
                    self.controller.get_logger().info(f"[AlignYawToZero] ✓ Yaw aligned to 0° (error: {math.degrees(yaw_error):.2f}°)")
                return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
            else:
                # 성공 중이지만 연속 확인
                return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        else:
            self.success_count = 0

        # 회전 제어 (최소 속도 보장)
        raw_angular = yaw_error * 1.5  # 게인 적용
        angular_magnitude = max(abs(raw_angular), self.min_angular_speed)
        t.angular.z = math.copysign(angular_magnitude, yaw_error)
        t.angular.z = clamp(t.angular.z, -self.angular_speed, self.angular_speed)

        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)


class GoToFront(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.start_time = None   # 시작 시간
        self.forward_time = 0.8  # 전진 지속 시간 (초) - 바퀴 정렬용이므로 짧게
        self.forward_speed = 0.1 # 전진 속도 (m/s) - 천천히

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now()

        if self.start_time is None:
            self.start_time = now
            self.controller.get_logger().info(f"GoToFront: Starting forward motion for {self.forward_time}s at {self.forward_speed}m/s")

        elapsed = (now - self.start_time).nanoseconds / 1e9  # 초 단위 경과시간

        t = Twist()
        if elapsed < self.forward_time:
            # 지정된 시간 동안 전진
            t.linear.x = self.forward_speed
            t.angular.z = 0.0
            return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
        else:
            # 시간 경과 후 → 정지 + COMPLETE
            self.controller.get_logger().info("GoToFront: Forward motion complete")
            return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)


class GoToSideInit(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.goal_sent = False
        self.target_x = -0.6
        self.target_y = 0.3
        self.target_yaw = 0.0

    def update(self, current_pose: PoseStamped):
        if not self.goal_sent:
            goal = make_goal_pose(self.target_x, self.target_y, self.target_yaw, self.controller)
            self.goal_sent = True
            self.controller.get_logger().info(f"GoToSideInit: Going to side init position ({self.target_x}, {self.target_y})")
            return (StateOutputKind.GOAL, goal, StateResult.CONTINUE)
        
        # 목표 지점 도달 확인
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        distance = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)
        yaw_error = normalize_angle(self.target_yaw - current_yaw)
        
        if distance < self.controller.distance_tolerance and abs(yaw_error) < self.controller.angle_tolerance:
            self.controller.get_logger().info("GoToSideInit: Reached side init position!")
            return (StateOutputKind.NONE, None, StateResult.COMPLETE)
        else:
            return (StateOutputKind.NONE, None, StateResult.CONTINUE)


class FindAndGoClearing(ControllerState):
    """
    공터를 찾아서 이동하는 통합 State (Potential Field 방식)
    
    Phase:
    1. SCANNING: 360도 회전하며 레이저 데이터 수집
    2. ROTATING: 잠재력장 분석 후 최적 방향으로 회전
    3. MOVING: 공터로 전진
    4. CHECKING: 공터 조건 확인 (반경 1m 내 80% 비어있는지)
    """
    def __init__(self, controller):
        super().__init__(controller)
        self.phase = "SCANNING"
        
        # 스캔 관련
        self.scan_start_time = None
        self.angular_velocity = 1.0  # rad/s (~57°/s)
        self.scan_done = False
        self.yaw_start = None
        self.yaw_prev = None
        self.yaw_accum = 0.0
        self.fake_scan = [None] * 360  # 0..359 deg, meters
        self.max_range = 5.0
        self.min_range = 0.12
        self._verify_open = False
        
        # 센서 간격 (좌우 레이저 센서 사이 거리)
        self.sensor_baseline = 0.11  # m (실제 장착 치수)
        
        # 잠재력장 파라미터
        self.R_clear = 1.0    # 공터 반경 기준
        self.R_cap = 3.0      # 자유공간 가중 상한
        self.d_rep = 0.8      # repulsion 활성 거리
        self.k_att = 1.0      # attraction gain
        self.k_rep = 0.25     # repulsion gain
        
        # 로봇/안전 파라미터
        self.r_robot = 0.20
        self.margin = 0.10
        self.open_ratio_thresh = 0.80
        self.open_confirm_frames = 5
        self._open_ok_count = 0
        
        # 최적 방향 결과
        self.best_direction = None
        self.best_distance = None
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        
        # 이동 관련
        self.goal_sent = False
        self.start_x = None
        self.start_y = None
        self.sector_half_deg = 15.0
        self.move_start_time = None
        self.move_timeout_s = 8.0
        
        # 열림 판정 히스테리시스
        self.open_ratio_fall = 0.75  # 내려오면 실패로 간주
        
        # 재시도 제한
        self.rescan_count = 0
        self.max_rescans = 5  # 최대 5번 재스캔
        
        controller.get_logger().info("FindAndGoClearing: Starting clearing search (Potential Field)")
    
    def update(self, current_pose: PoseStamped):
        if self.phase == "SCANNING":
            return self._handle_scanning(current_pose)
        elif self.phase == "ROTATING":
            return self._handle_rotating(current_pose)
        elif self.phase == "MOVING":
            return self._handle_moving(current_pose)
        
        return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
    
    def _handle_scanning(self, current_pose: PoseStamped):
        """360도 스캔하며 데이터 수집"""
        import time
        now = time.time()
        
        # 스핀 시작 초기화
        if self.scan_start_time is None:
            self.scan_start_time = now
            self.yaw_start = get_yaw_from_quaternion(current_pose.pose.orientation)
            self.yaw_prev = None
            self.yaw_accum = 0.0
            self.scan_done = False
            self.fake_scan = [None] * 360
            self.controller.get_logger().info("FindAndGoClearing: Starting 360° scan (PF)")
        
        # 현재 yaw
        yaw_now = get_yaw_from_quaternion(current_pose.pose.orientation)
        
        # 누적 절대 회전량 계산
        if self.yaw_prev is None:
            self.yaw_prev = yaw_now
        else:
            dyaw = normalize_angle(yaw_now - self.yaw_prev)
            self.yaw_accum += abs(dyaw)
            self.yaw_prev = yaw_now
        
        # 0..2π로 래핑한 각도로 버킷 결정
        yaw_wrapped = (yaw_now - self.yaw_start) % (2 * math.pi)
        deg_center = int(round(math.degrees(yaw_wrapped))) % 360
        
        # 레이저 샘플 취득 (좌우 두 센서 모두 활용, 동적 오프셋)
        if len(self.controller.laser_sensor_value) >= 2:
            left = self.controller.laser_sensor_value[0] / 1000.0
            right = self.controller.laser_sensor_value[1] / 1000.0
            
            vals = []
            if self.min_range <= left <= self.max_range:
                vals.append(('L', left))
            if self.min_range <= right <= self.max_range:
                vals.append(('R', right))
            
            # 동적 오프셋 (거리에 따라 각도 오프셋 변화)
            for tag, d in vals:
                delta_deg = math.degrees(math.atan2(max(self.sensor_baseline * 0.5, 1e-3), max(d, 0.2)))
                deg = int(round(deg_center + (+delta_deg if tag == 'L' else -delta_deg))) % 360
                prev = self.fake_scan[deg]
                self.fake_scan[deg] = d if prev is None else 0.5 * prev + 0.5 * d
            
            # 중앙 버킷(보수적): min(left, right)
            if vals:
                d_center = min(v for _, v in vals)
                prev = self.fake_scan[deg_center]
                self.fake_scan[deg_center] = d_center if prev is None else 0.5 * prev + 0.5 * d_center
        
        # 회전 명령
        t = Twist()
        t.angular.z = self.angular_velocity
        t.linear.x = 0.0
        
        # 360° 완료 판단: 누적 절대 회전량
        if self.yaw_accum >= 2 * math.pi - 0.05:
            self.scan_done = True
            self.controller.get_logger().info(f"FindAndGoClearing: Scan complete! yaw_accum={self.yaw_accum:.2f} rad")
        
        # 디버그: 스캔 진행상황 (5초마다 로그)
        if (now - self.scan_start_time) % 5 < 0.1:
            self.controller.get_logger().info(
                f"FindAndGoClearing: Scanning... yaw_accum={self.yaw_accum:.2f}/{2*math.pi:.2f} rad "
                f"({math.degrees(self.yaw_accum):.1f}°/{360}°)"
            )
        
        if self.scan_done:
            # 결측 보간 + 필터
            self._sanitize_fake_scan()
            
            # 스캔 완료 후: 모든 방향이 1m 이상이면 이미 공터이므로 굳이 움직일 필요 없음
            min_distance = min(self.fake_scan)
            if min_distance >= 1.0:  # 1m (1000mm)
                self.controller.get_logger().info(
                    f"FindAndGoClearing: Already in clearing! All directions >= 1m (min={min_distance:.2f}m) → Complete"
                )
                return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
            
            # verify_open 모드면 오픈 비율만 체크하고 완료
            if self._verify_open:
                open_ratio = self._compute_open_ratio()
                self._verify_open = False
                if open_ratio >= self.open_ratio_thresh:
                    self.controller.get_logger().info(
                        f"FindAndGoClearing: Open verified! (open_ratio={open_ratio:.2%})"
                    )
                    return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
                else:
                    # 히스테리시스 체크
                    if open_ratio < self.open_ratio_fall:
                        # 진짜 부족 → 다시 분석해서 이동
                        self.controller.get_logger().info(
                            f"FindAndGoClearing: Not open enough (open_ratio={open_ratio:.2%}), retry"
                        )
                    else:
                        # 0.75~0.80 경계선 → 한 번 더 재스캔
                        self.controller.get_logger().info(
                            f"FindAndGoClearing: Borderline (open_ratio={open_ratio:.2%}), rescan once more"
                        )
                        self._verify_open = True
                        self.scan_start_time = None
                        self.scan_done = False
                        return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
            
            # 잠재력장 분석
            self._analyze_scan_data_potential_field(current_pose)
            self.phase = "ROTATING"
            open_ratio = self._compute_open_ratio()
            self.controller.get_logger().info(
                f"FindAndGoClearing: Scan done! best_dir={math.degrees(self.best_direction):.1f}°, "
                f"best_dist≈{self.best_distance:.2f}m, open_ratio={open_ratio:.2f}"
            )
            return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
    
    def _sanitize_fake_scan(self):
        """결측값 보간 + 가장 간단한 평활화"""
        arr = self.fake_scan
        # 1) 결측을 양옆 유효값으로 선형 보간
        # 앞쪽 유효 찾기
        last = None
        for i in range(360):
            if arr[i] is not None:
                last = arr[i]
                break
        if last is None:
            # 한 샘플도 없으면 전부 최대거리
            self.fake_scan = [self.max_range] * 360
            return
        
        # 순환 보간
        fill = arr[:]
        prev_val = last
        for i in range(360 * 2):  # 두 바퀴 돌아도 안전
            idx = i % 360
            if fill[idx] is None:
                # 앞으로 유효값 탐색
                j = 1
                while fill[(idx + j) % 360] is None and j < 360:
                    j += 1
                next_val = fill[(idx + j) % 360] if j < 360 else self.max_range
                # 선형 보간
                fill[idx] = prev_val + (next_val - prev_val) * (1.0 / (j + 1))
            prev_val = fill[idx]
        
        # 2) 범위 클리핑
        self.fake_scan = [max(self.min_range, min(self.max_range, x)) for x in fill]
    
    def _compute_open_ratio(self):
        """반경 R_clear 내에서 비어있는 비율 계산"""
        need = self.R_clear + self.r_robot + self.margin
        ok = sum(1 for d in self.fake_scan if d >= need)
        return ok / float(len(self.fake_scan))
    
    def _analyze_scan_data_potential_field(self, current_pose: PoseStamped):
        """잠재력장 방식으로 스캔 데이터 분석"""
        fs = self.fake_scan
        if not fs or all(d is None for d in fs):
            # Fallback: 현재 방향으로
            self.best_direction = get_yaw_from_quaternion(current_pose.pose.orientation)
            self.best_distance = 1.0
            return
        
        v_x = 0.0
        v_y = 0.0
        v_rep_x = 0.0
        v_rep_y = 0.0
        
        # Attraction: 자유공간 쪽으로 벡터합
        for deg in range(360):
            d = fs[deg]
            if d is None:
                continue
            theta = math.radians(deg)  # 스핀 기준 좌표계
            
            # 자유공간 가중 (R_clear 너머가 클수록 큼, R_cap로 상한)
            w_free = max(0.0, min(self.R_cap, d) - self.R_clear) / self.R_cap
            v_x += w_free * math.cos(theta)
            v_y += w_free * math.sin(theta)
            
            # Repulsion: 가까우면 강하게 밀어냄
            if d < self.d_rep:
                w_rep = self.k_rep * max(0.0, (1.0 / d - 1.0 / self.d_rep)) / (d * d)
                v_rep_x += w_rep * math.cos(theta)
                v_rep_y += w_rep * math.sin(theta)
        
        # 합력 (장애물 반발은 반대로 작용)
        v_x = self.k_att * v_x - v_rep_x
        v_y = self.k_att * v_y - v_rep_y
        
        # 방향/크기
        if abs(v_x) < 1e-6 and abs(v_y) < 1e-6:
            # 모든 방향이 막힘 → 현재 yaw 유지
            self.best_direction = get_yaw_from_quaternion(current_pose.pose.orientation)
        else:
            self.best_direction = math.atan2(v_y, v_x)
        
        # 해당 방향의 예상 여유거리(섹터 최소)로 step 설정
        sector_half = math.radians(self.sector_half_deg)
        dmin = self._sector_min_range(self.best_direction, sector_half)
        safety = self.r_robot + self.margin
        self.best_distance = max(0.0, dmin - safety)
        
        # 목표 위치 계산 (1 m cap)
        move_distance = min(1.0, max(0.0, self.best_distance - 0.2))
        cur_x = current_pose.pose.position.x
        cur_y = current_pose.pose.position.y
        goal_x_raw = cur_x + move_distance * math.cos(self.best_direction)
        goal_y_raw = cur_y + move_distance * math.sin(self.best_direction)
        
        # 작업영역 내로 제한
        self.goal_x, self.goal_y = self._keep_in_bounds(goal_x_raw, goal_y_raw)
        self.goal_yaw = self.best_direction
    
    def _sector_min_range(self, heading, half):
        """fake_scan에서 [heading-half, heading+half] 최소거리"""
        def rad2deg(a):
            d = int(round(math.degrees(a))) % 360
            return d
        
        dmin = self.max_range
        start = rad2deg(heading - half)
        end = rad2deg(heading + half)
        
        # 원형 구간 순회
        idx = start
        while True:
            d = self.fake_scan[idx]
            if d is not None:
                dmin = min(dmin, d)
            if idx == end:
                break
            idx = (idx + 1) % 360
        return dmin
    
    def _keep_in_bounds(self, x, y):
        """작업영역 폴리곤 내로 목표 위치 제한"""
        # 작업 영역 예시: x∈[-2, +1], y∈[-3, +3]
        # 실제 환경에 맞게 조정하세요
        x = max(-2.0, min(1.0, x))
        y = max(-3.0, min(3.0, y))
        return x, y
    
    def _handle_rotating(self, current_pose: PoseStamped):
        """최적 방향으로 회전"""
        current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
        angle_diff = normalize_angle(self.best_direction - current_yaw)
        
        # 회전 완료 확인
        if abs(angle_diff) < 0.1:  # ~6도

            self.phase = "MOVING"
            self.start_x = current_pose.pose.position.x
            self.start_y = current_pose.pose.position.y
            self.move_start_time = time.time()  # 타임아웃 체크용
            self.controller.get_logger().info("FindAndGoClearing: Rotation complete, moving forward")
            return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
        # 회전
        t = Twist()
        t.angular.z = 0.8 if angle_diff > 0 else -0.8
        t.linear.x = 0.0
        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
    
    def _handle_moving(self, current_pose: PoseStamped):
        # 스턱 타임아웃 체크
        if self.move_start_time and (time.time() - self.move_start_time) > self.move_timeout_s:
            self.controller.get_logger().warn("FindAndGoClearing: Move timeout → rescan")
            self.phase = "SCANNING"
            self.scan_start_time = None
            self.scan_done = False
            return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
        # 전방 장애물 체크 (레이저 센서 직접 사용, 300mm 기준)
        if len(self.controller.laser_sensor_value) >= 2:
            left_mm = self.controller.laser_sensor_value[0]
            right_mm = self.controller.laser_sensor_value[1]
            min_mm = min(left_mm, right_mm)
            
            # 300mm(0.3m) 이내 장애물 감지 → 정지 후 재스캔
            if min_mm < 200:
                self.rescan_count += 1
                self.controller.get_logger().warn(
                    f"FindAndGoClearing: Obstacle at {min_mm}mm, rescanning ({self.rescan_count}/{self.max_rescans})"
                )
                
                # 최대 재시도 횟수 초과 시 포기
                if self.rescan_count >= self.max_rescans:
                    self.controller.get_logger().warn(
                        f"FindAndGoClearing: Max rescans reached ({self.max_rescans}). Giving up."
                    )
                    return (StateOutputKind.TWIST, Twist(), StateResult.COMPLETE)
                
                self.phase = "SCANNING"
                self.scan_start_time = None
                self.scan_done = False
                return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
        # 이동 거리 확인 (목표 도달 판정)
        if self.start_x is not None and self.start_y is not None:
            current_x = current_pose.pose.position.x
            current_y = current_pose.pose.position.y
            moved_distance = math.sqrt(
                (current_x - self.start_x)**2 + (current_y - self.start_y)**2
            )
            
            # 목표 거리 계산
            target_distance = math.sqrt(
                (self.goal_x - self.start_x)**2 + (self.goal_y - self.start_y)**2
            )
            
            # 목표의 90% 이상 이동했거나, 1m 이상 이동했으면 도착으로 간주
            if moved_distance >= target_distance * 0.9 or moved_distance >= 1.0:
                # 도착 시 현재 위치에서 재스캔하여 오픈 비율 검증
                self.controller.get_logger().info(
                    f"FindAndGoClearing: Moved {moved_distance:.2f}m, verifying openness"
                )
                self.rescan_count = 0  # 성공적으로 이동했으므로 재시도 카운터 리셋
                self.phase = "SCANNING"
                self._verify_open = True
                self.scan_start_time = None
                self.scan_done = False
                return (StateOutputKind.TWIST, Twist(), StateResult.CONTINUE)
        
        # 직진 명령 (Twist 직접 제어)
        t = Twist()
        t.linear.x = 0.2  # 0.2 m/s 전진
        t.angular.z = 0.0
        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)
    


class WanderRandomly(ControllerState):
    def __init__(self, controller):
        super().__init__(controller)
        self.controller.get_logger().info("WanderRandomly: Starting smooth, snake-like wandering mode.")
        
        # Bounding box for wandering area
        self.x_min, self.x_max = -0.7, 1.5
        self.y_min, self.y_max = -1.0, 1.0
        
        # Movement parameters for smoother motion
        self.linear_speed = 0.2  # Reduced for calmer movement
        self.angular_speed_max = 1.5 # Reduced for gentler turns
        self.wander_base_angular_vel = 0.0
        self.angular_vel_change_rate = 0.6 # How quickly the turn rate can change

        # Smoothing parameters (acceleration limits)
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.linear_accel_rate = 0.6 # m/s^2
        self.angular_accel_rate = 2.5 # rad/s^2

        # Obstacle Avoidance state
        self.avoiding = False
        self.laser_threshold = 500.0  # mm

        # Stuck Detection
        self.last_pose = None
        self.stuck_timer = 0.0
        self.stuck_thresh_dist = 0.02 # in meters, over the stuck_time duration
        self.stuck_time = 3.0 # seconds
        self.is_stuck_recovering = False
        self.stuck_recovery_end_time = 0.0
        self.stuck_recovery_twist = Twist()

    def soft_wall_turn_gain(self, x, y):
        dx = min(x - self.x_min, self.x_max - x)
        dy = min(y - self.y_min, self.y_max - y)
        w = self.x_max - self.x_min
        h = self.y_max - self.y_min
        if w <= 0 or h <= 0: return 0.0
        # margin is 0 at boundary, 0.5 at center
        margin = min(dx / w, dy / h) if min(dx, dy) > 0 else 0
        # gain is 1 at boundary, 0 at center
        return clamp(1.0 - 2.0 * margin, 0.0, 1.0)

    def update(self, current_pose: PoseStamped):
        now = self.controller.get_clock().now().nanoseconds / 1e9
        
        # --- 1. Handle Stuck Recovery ---
        if self.is_stuck_recovering:
            if now < self.stuck_recovery_end_time:
                # Continue recovery maneuver
                return (StateOutputKind.TWIST, self.stuck_recovery_twist, StateResult.CONTINUE)
            else:
                # Recovery finished
                self.is_stuck_recovering = False
                self.stuck_timer = 0.0 # Reset timer

        # --- 2. Check for Stuck Condition ---
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        if self.last_pose and abs(self.current_linear_vel) > 0.05: # Only check if supposed to be moving
            moved = math.hypot(current_x - self.last_pose[0], current_y - self.last_pose[1])
            if moved < self.stuck_thresh_dist:
                self.stuck_timer += self.controller.dt
                if self.stuck_timer > self.stuck_time:
                    self.controller.get_logger().warn("WanderRandomly: Stuck! Starting recovery maneuver.")
                    self.is_stuck_recovering = True
                    self.stuck_recovery_end_time = now + 1.5 # Recover for 1.5 seconds
                    self.stuck_recovery_twist.linear.x = -0.1 # Back up
                    self.stuck_recovery_twist.angular.z = self.angular_speed_max * random.choice([-1, 1]) # while turning
                    self.last_pose = (current_x, current_y)
                    self.current_linear_vel = -0.1
                    self.current_angular_vel = self.stuck_recovery_twist.angular.z
                    return (StateOutputKind.TWIST, self.stuck_recovery_twist, StateResult.CONTINUE)
            else:
                self.stuck_timer = 0.0
        self.last_pose = (current_x, current_y)

        # --- 3. Normal Wander and Avoidance Logic ---
        target_linear_x = self.linear_speed
        
        # Check for obstacles
        laser_fresh = self.controller.is_laser_fresh(timeout=0.5)
        laser_values = self.controller.laser_sensor_value
        obstacle_is_close = False
        if laser_values and laser_fresh and min(laser_values) < self.laser_threshold:
            obstacle_is_close = True

        # --- Determine Target Velocities ---
        if obstacle_is_close:
            if not self.avoiding:
                self.avoiding = True
                self.controller.get_logger().warn("WanderRandomly: Obstacle detected. Avoiding.")
            
            # Determine turn direction based on lasers
            if laser_values[0] < laser_values[1]:
                # Obstacle closer on the left, turn right
                target_angular_z = -self.angular_speed_max
            else:
                # Obstacle closer on the right (or equal), turn left
                target_angular_z = self.angular_speed_max
        else: # Wander freely
            if self.avoiding:
                self.avoiding = False
                # Smoothly return to wandering by reducing the sharp turn
                self.wander_base_angular_vel = self.current_angular_vel * 0.5

            # Snake-like motion: slowly change angular velocity
            change = random.uniform(-self.angular_vel_change_rate, self.angular_vel_change_rate) * self.controller.dt
            self.wander_base_angular_vel = clamp(self.wander_base_angular_vel + change, -self.angular_speed_max, self.angular_speed_max)
            target_angular_z = self.wander_base_angular_vel
            
            # Soft wall repulsion (only when not avoiding obstacles)
            wall_gain = self.soft_wall_turn_gain(current_x, current_y)
            if wall_gain > 0:
                center_x = (self.x_min + self.x_max) / 2
                center_y = (self.y_min + self.y_max) / 2
                
                angle_to_center = math.atan2(center_y - current_y, center_x - current_x)
                current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
                heading_error = normalize_angle(angle_to_center - current_yaw)
                
                # Add a component that steers towards the center
                target_angular_z += wall_gain * heading_error * 2.0

        # Couple linear speed to turning rate (the more it turns, the slower it goes)
        turn_ratio = abs(target_angular_z) / self.angular_speed_max
        target_linear_x *= (1.0 - 0.8 * turn_ratio)

        # --- 4. Apply Smoothing (Acceleration Limiting) ---
        dt = self.controller.dt
        t = Twist()

        # Smooth linear velocity
        max_linear_change = self.linear_accel_rate * dt
        linear_error = target_linear_x - self.current_linear_vel
        self.current_linear_vel += clamp(linear_error, -max_linear_change, max_linear_change)
        
        # Smooth angular velocity
        max_angular_change = self.angular_accel_rate * dt
        angular_error = target_angular_z - self.current_angular_vel
        self.current_angular_vel += clamp(angular_error, -max_angular_change, max_angular_change)
        
        t.linear.x = self.current_linear_vel
        t.angular.z = self.current_angular_vel
        
        return (StateOutputKind.TWIST, t, StateResult.CONTINUE)


# ===== Controller node =====
class MobileActionController(Node):
    def __init__(self):
        super().__init__('mobile_action_controller')

        # Parameters
        self.declare_parameter('angle_tolerance', 0.15)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('laser_threshold', 700)
        self.declare_parameter('aruco_center_deadband', 0.05)
        self.declare_parameter('aruco_seen_timeout', 2.0) # seconds
        self.declare_parameter('debug_mode', True)  # 디버그 모드 파라미터 추가

        # ArUco 관련 파라미터들 (ApproachStation용)
        # Declare individual marker parameters
        self.declare_parameter("aruco_marker_0", [-0.01, 0.0, 0.5, 1.5708, 0.0, -1.5708])
        self.declare_parameter("aruco_marker_101", [-2.0, 2.49, 0.5, 1.5708, 0.0, 0.0])
        self.declare_parameter("aruco_marker_201", [-3.99, 0.0, 0.5, 1.5708, 0.0, 1.5708])
        self.declare_parameter("aruco_marker_301", [-2.0, -2.49, 0.5, 1.5708, 0.0, 3.14])

        self.declare_parameter('aruco_target_area', 0.030)
        self.declare_parameter('aruco_center_x_bias', 0.08)
        self.declare_parameter('aruco_center_deadband_in', 0.04)
        self.declare_parameter('aruco_center_deadband_out', 0.06)
        self.declare_parameter('aruco_align_ok_frames', 1)
        self.declare_parameter('aruco_min_complete_ratio', 0.85)
        self.declare_parameter('aruco_max_safe_area', 0.030)
        self.declare_parameter('aruco_max_ang_adv', 0.55)
        self.declare_parameter('aruco_min_lin_near', 0.06)
        self.declare_parameter('aruco_max_lin_near', 0.10)

        # GoToMiddle (gtm) parameters
        self.declare_parameter('gtm_k_ang_align', 1.2)
        self.declare_parameter('gtm_min_ang_align', 0.5)
        self.declare_parameter('gtm_max_ang_align', 1.2)
        self.declare_parameter('gtm_theta_min_deg', 12.0)
        self.declare_parameter('gtm_theta_max_deg', 35.0)
        self.declare_parameter('gtm_step_min_m', 0.18)
        self.declare_parameter('gtm_step_max_m', 0.38)
        self.declare_parameter('gtm_k_theta_scale', 0.8)
        self.declare_parameter('gtm_k_step_scale', 0.8)
        self.declare_parameter('gtm_ang_speed', 0.8)
        self.declare_parameter('gtm_lin_speed', 0.18)

        # Angular PID params
        self.declare_parameter('angular_P', 0.5)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.001)
        self.declare_parameter('angular_max_state', 0.6)
        self.declare_parameter('angular_min_state', -0.6)

        # Linear PID params
        self.declare_parameter('linear_P', 1.0)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        self.declare_parameter('linear_max_state', 0.6)
        self.declare_parameter('linear_min_state', -0.6)

        # Get initial parameter values
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.laser_threshold = self.get_parameter('laser_threshold').value
        self.aruco_target_area = self.get_parameter('aruco_target_area').value
        self.aruco_center_deadband = self.get_parameter('aruco_center_deadband').value
        self.aruco_seen_timeout = self.get_parameter('aruco_seen_timeout').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # ArUco 관련 파라미터 값들
        self.aruco_center_x_bias = self.get_parameter('aruco_center_x_bias').value
        self.aruco_center_deadband_in = self.get_parameter('aruco_center_deadband_in').value
        self.aruco_center_deadband_out = self.get_parameter('aruco_center_deadband_out').value
        self.aruco_align_ok_frames = self.get_parameter('aruco_align_ok_frames').value
        self.aruco_min_complete_ratio = self.get_parameter('aruco_min_complete_ratio').value
        self.aruco_max_safe_area = self.get_parameter('aruco_max_safe_area').value
        self.aruco_max_ang_adv = self.get_parameter('aruco_max_ang_adv').value
        self.aruco_min_lin_near = self.get_parameter('aruco_min_lin_near').value
        self.aruco_max_lin_near = self.get_parameter('aruco_max_lin_near').value

        # Load ArUco marker positions
        self.aruco_marker_position = {}  # Initialize the dictionary first
        
        marker_configs = {
            "0": self.get_parameter("aruco_marker_0").value,
            "101": self.get_parameter("aruco_marker_101").value,
            "201": self.get_parameter("aruco_marker_201").value,
            "301": self.get_parameter("aruco_marker_301").value
        }

        for marker_id, pose_list in marker_configs.items():
            if len(pose_list) == 6:
                xyz = pose_list[:3]
                rpy = pose_list[3:]

                T = np.eye(4)
                T[:3, 3] = np.array(xyz)
                T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()

                self.aruco_marker_position[int(marker_id)] = T
                if self.debug_mode:
                    self.get_logger().info(f"Loaded marker {marker_id} pose: {xyz}")

        # GoToMiddle (gtm) parameters
        self.gtm_k_ang_align = self.get_parameter('gtm_k_ang_align').value
        self.gtm_min_ang_align = self.get_parameter('gtm_min_ang_align').value
        self.gtm_max_ang_align = self.get_parameter('gtm_max_ang_align').value
        self.gtm_theta_min_deg = self.get_parameter('gtm_theta_min_deg').value
        self.gtm_theta_max_deg = self.get_parameter('gtm_theta_max_deg').value
        self.gtm_step_min_m = self.get_parameter('gtm_step_min_m').value
        self.gtm_step_max_m = self.get_parameter('gtm_step_max_m').value
        self.gtm_k_theta_scale = self.get_parameter('gtm_k_theta_scale').value
        self.gtm_k_step_scale = self.get_parameter('gtm_k_step_scale').value
        self.gtm_ang_speed = self.get_parameter('gtm_ang_speed').value
        self.gtm_lin_speed = self.get_parameter('gtm_lin_speed').value

        # Init PIDs (필요 시 사용)
        self.angular_pid = PID()
        self.angular_pid.P = self.get_parameter('angular_P').value
        self.angular_pid.I = self.get_parameter('angular_I').value
        self.angular_pid.D = self.get_parameter('angular_D').value
        self.angular_pid.max_state = self.get_parameter('angular_max_state').value
        self.angular_pid.min_state = self.get_parameter('angular_min_state').value

        self.linear_pid = PID()
        self.linear_pid.P = self.get_parameter('linear_P').value
        self.linear_pid.I = self.get_parameter('linear_I').value
        self.linear_pid.D = self.get_parameter('linear_D').value
        self.linear_pid.max_state = self.get_parameter('linear_max_state').value
        self.linear_pid.min_state = self.get_parameter('linear_min_state').value

        # Publishers / Subscribers
        self.pub_charging_state = self.create_publisher(String, '/edie8/charging/state', 1)
        self.pub_mobile_action_state = self.create_publisher(KeyValue, '/edie8/mobile/action/state', 10)
        self.pub_twist = self.create_publisher(TwistStamped, '/edie8/navigation/direct_vel', 10)
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/edie8/navigation/goal_pose', 10)

        self.sub_mobile_action_command = self.create_subscription(
            UInt8, '/edie8/mobile/action/command', self.mobile_action_command_callback, 10)
        
        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_robot_pose = self.create_subscription(
            PoseStamped, '/edie8/localization/robot_pose', self.robot_pose_callback, qos_profile_best_effort)
        
        self.sub_aruco_center = self.create_subscription(
            PointStamped,
            '/edie8/localization/aruco_marker/center_info',
            self.aruco_center_callback,
            qos_profile_best_effort)
        
        self.sub_aruco_pose = self.create_subscription(
            PoseStamped, '/edie8/localization/aruco_marker/robot_pose', self.aruco_pose_callback, qos_profile_best_effort)

        self.sub_front_laser = self.create_subscription(
            Int16MultiArray, '/edie8/sensor/front/laser', self.laser_sensor_callback, qos_profile_best_effort)

        self.sub_scan_result = self.create_subscription(
            PoseStamped, '/edie8/localization/scan_result', self.scan_result_callback, qos_profile_best_effort)

        self.sub_target_marker_id = self.create_subscription(
            String, '/edie8/behavior/target_marker_id', self.target_marker_id_callback, qos_profile_best_effort)

        self.sub_aruco_eval = self.create_subscription(
            PoseWithInfoStamped, '/edie8/localization/eval_aruco', self.aruco_eval_callback, qos_profile_best_effort)
        
        self.sub_visible_marker_count = self.create_subscription(
            UInt8, 
            '/edie8/localization/aruco_marker/visible_marker_count', 
            self.visible_marker_count_callback, 
            qos_profile_best_effort
        )
        
        self.control_timer = self.create_timer(0.02, self.control_timer_callback)

        # State
        self.current_pose = PoseStamped()
        self.state_instance: ControllerState | None = None
        self.received_command = CMD_IDLE
        self.aruco_marker_seen_in_find_station = False # FindStationForInit와 MoveRandomly 내부에서 사용할 플래그
        self.laser_sensor_value = [2000, 2000]
        self.last_laser_stamp = Time()  # ← 여기에 추가!
        self.last_aruco_pose = None # SearchAruco 상태에서 사용할 변수
        self.aruco_center = None
        self.aruco_center_stamp = Time()
        self.last_scan_result = None  
        self.target_marker_id = None  # Add this
        self.visible_marker_count = 0

        # ArUco 평가 결과 저장
        self.aruco_eval_result = None
        self.aruco_eval_stamp = Time()
        self.dt = 0.02  # 제어 루프 주기 (타이머와 동일하게 설정)

        # Command → State mapping
        self.cmd_to_state = {
            CMD_IDLE:      IdleState,
            CMD_INIT_HOME: InitHome,
            CMD_SPIN_ONCE: SpinOnce,
            CMD_GO_TO_PRE_ALIGN_POSE: GoToPreAlignPose,
            CMD_ALIGNMENT: AlignToStation,
            CMD_ROTATE_BEHIND: RotateBehind,
            CMD_GO_BACK:   GoBack,
            CMD_FIND_STATION_INIT: FindStationForInit,
            CMD_FIND_STATION_CHARGE: FindStationForCharge,
            CMD_MOVE_RANDOMLY: MoveRandomly,
            CMD_SEARCH_ARUCO: SearchAruco,
            CMD_APPROACH_STATION: ApproachStation,
            CMD_GO_TO_MIDDLE: GoToMiddle,
            CMD_APPROACH_MARKER: ApproachMarker,
            CMD_GO_TO_SIDE_LEFT: GoToSideLeft,
            CMD_GO_TO_SIDE_RIGHT: GoToSideRight,
            CMD_GO_TO_FRONT: GoToFront,
            CMD_GO_TO_SIDE_INIT: GoToSideInit,
            CMD_ALIGN_YAW_TO_ZERO: AlignYawToZero,
            CMD_FIND_AND_GO_CLEARING: FindAndGoClearing,
            CMD_WANDER_RANDOMLY: WanderRandomly
        }

        # Param callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("MobileActionController initialized")

    # === Callbacks ===
    def robot_pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def aruco_center_callback(self, msg: PointStamped):
        self.aruco_center = msg  # Store the entire PointStamped message
        self.aruco_center_stamp = Time.from_msg(msg.header.stamp)

    def aruco_pose_callback(self, msg: PoseStamped):
        # 항상 last_aruco_pose 업데이트
        self.last_aruco_pose = msg
        
        # FindStationForInit, MoveRandomly, SearchAruco 상태일 때마다 플래그를 True로 설정
        if isinstance(self.state_instance, (FindStationForInit, MoveRandomly, SearchAruco)):
            self.aruco_marker_seen_in_find_station = True

    def aruco_eval_callback(self, msg: PoseWithInfoStamped):
        """ArUco 평가 결과 콜백"""
        self.aruco_eval_result = msg
        self.aruco_eval_stamp = Time.from_msg(msg.header.stamp)
        

    def visible_marker_count_callback(self, msg: UInt8):
        self.visible_marker_count = msg.data

    def mobile_action_command_callback(self, msg: UInt8):
        self.get_logger().info(f"Received mobile action command: {msg.data}")
        self.received_command = msg.data
        state_cls = self.cmd_to_state.get(self.received_command)
        if state_cls is None:
            # self.get_logger().warn(f"Unknown command: {self.received_command}")
            return
        self.set_state(state_cls)

    def laser_sensor_callback(self, msg: Int16MultiArray):
        self.laser_sensor_value = msg.data[:]
        self.last_laser_stamp = self.get_clock().now()

    def scan_result_callback(self, msg: PoseStamped):
        self.last_scan_result = msg
        if self.debug_mode:
            self.get_logger().info(f"Scan result received: position=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})")

    def target_marker_id_callback(self, msg: String):
        self.target_marker_id = msg.data
        if self.debug_mode:
            self.get_logger().info(f"Target marker ID received: {self.target_marker_id}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'angle_tolerance':
                self.angle_tolerance = param.value
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = param.value
            elif param.name == 'laser_threshold':
                self.laser_threshold = param.value
            elif param.name == 'aruco_target_area':
                self.aruco_target_area = param.value
            elif param.name == 'aruco_center_deadband':
                self.aruco_center_deadband = param.value
            elif param.name == 'aruco_seen_timeout':
                self.aruco_seen_timeout = param.value
            elif param.name == 'aruco_center_x_bias':
                self.aruco_center_x_bias = param.value
            elif param.name == 'aruco_center_deadband_in':
                self.aruco_center_deadband_in = param.value
            elif param.name == 'aruco_center_deadband_out':
                self.aruco_center_deadband_out = param.value
            elif param.name == 'aruco_align_ok_frames':
                self.aruco_align_ok_frames = param.value
            elif param.name == 'aruco_min_complete_ratio':
                self.aruco_min_complete_ratio = param.value
            elif param.name == 'aruco_max_safe_area':
                self.aruco_max_safe_area = param.value
            elif param.name == 'aruco_max_ang_adv':
                self.aruco_max_ang_adv = param.value
            elif param.name == 'aruco_min_lin_near':
                self.aruco_min_lin_near = param.value
            elif param.name == 'aruco_max_lin_near':
                self.aruco_max_lin_near = param.value
            elif param.name == 'gtm_k_ang_align':
                self.gtm_k_ang_align = param.value
            elif param.name == 'gtm_min_ang_align':
                self.gtm_min_ang_align = param.value
            elif param.name == 'gtm_max_ang_align':
                self.gtm_max_ang_align = param.value
            elif param.name == 'gtm_theta_min_deg':
                self.gtm_theta_min_deg = param.value
            elif param.name == 'gtm_theta_max_deg':
                self.gtm_theta_max_deg = param.value
            elif param.name == 'gtm_step_min_m':
                self.gtm_step_min_m = param.value
            elif param.name == 'gtm_step_max_m':
                self.gtm_step_max_m = param.value
            elif param.name == 'gtm_k_theta_scale':
                self.gtm_k_theta_scale = param.value
            elif param.name == 'gtm_k_step_scale':
                self.gtm_k_step_scale = param.value
            elif param.name == 'gtm_ang_speed':
                self.gtm_ang_speed = param.value
            elif param.name == 'gtm_lin_speed':
                self.gtm_lin_speed = param.value
            elif param.name == 'angular_P':
                self.angular_pid.P = param.value
            elif param.name == 'angular_I':
                self.angular_pid.I = param.value
            elif param.name == 'angular_D':
                self.angular_pid.D = param.value
            elif param.name == 'angular_max_state':
                self.angular_pid.max_state = param.value
            elif param.name == 'angular_min_state':
                self.angular_pid.min_state = param.value
            elif param.name == 'linear_P':
                self.linear_pid.P = param.value
            elif param.name == 'linear_I':
                self.linear_pid.I = param.value
            elif param.name == 'linear_D':
                self.linear_pid.D = param.value
            elif param.name == 'linear_max_state':
                self.linear_pid.max_state = param.value
            elif param.name == 'linear_min_state':
                self.linear_pid.min_state = param.value
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
        return SetParametersResult(successful=True)

    # === Helpers ===
    def cancel_current_goal_if_any(self):
        """현재 진행 중인 goal 취소 (더미 함수)"""
        try:
            # Nav2 action 사용 시:
            # self.nav2_client.cancel_all_goals()
            # 현재는 goal_pose 토픽만 사용하므로 별도 취소 불필요
            pass
        except Exception as e:
            self.get_logger().warn(f"cancel_current_goal_if_any failed: {e}")
    
    def set_state(self, state_cls):
        # FindStationForInit, MoveRandomly, SearchAruco 상태가 시작될 때마다 마커 감지 플래그를 리셋
        if state_cls in [FindStationForInit, MoveRandomly, SearchAruco]:
            self.aruco_marker_seen_in_find_station = False

        self.state_instance = state_cls(self)
        if self.debug_mode:
            self.get_logger().info(f"State -> {state_cls.__name__}")

    def publish_robot_state(self, state: str, result: int):
        '''
        value가 0이면 done.
        value가 1이면 running.
        '''
        kv = KeyValue()
        kv.key = state
        kv.value = str(result)
        self.pub_mobile_action_state.publish(kv)

    # === Control loop ===
    def control_timer_callback(self):
        if self.state_instance is None:
            return

        kind, out_msg, status = self.state_instance.update(self.current_pose)

        if status == StateResult.COMPLETE:
            self.publish_robot_state(self.state_instance.__class__.__name__, 0)  # done=0
        elif status == StateResult.CONTINUE:
            self.publish_robot_state(self.state_instance.__class__.__name__, 1)  # running=1

        # 메시지 타입에 맞게 퍼블리시
        if kind == StateOutputKind.TWIST:
            ts_msg = TwistStamped()
            ts_msg.header.stamp = self.get_clock().now().to_msg()
            ts_msg.header.frame_id = "base_link"
            ts_msg.twist = out_msg
            self.pub_twist.publish(ts_msg)
        elif kind == StateOutputKind.GOAL:
            self.pub_goal_pose.publish(out_msg)

    def is_laser_fresh(self, timeout=0.5):
        now = self.get_clock().now().nanoseconds / 1e9
        laser_time = self.last_laser_stamp.nanoseconds / 1e9
        return (now - laser_time) < timeout

def main(args=None):
    rclpy.init(args=args)
    node = MobileActionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()
