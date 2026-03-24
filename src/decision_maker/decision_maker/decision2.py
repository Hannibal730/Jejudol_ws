#!/usr/bin/env python3

import os
import select
import sys
import termios
import threading
import time
import tty
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, String

# 실험용 변수 기본값(한 곳에서 관리)
DEFAULTS = {
    'publish_rate_hz': 30.0,
    'log_rate_hz': 1.0,

    'emergency_deceleration_sec': 0.2,
    'emergency_deceleration_target_throttle': 0.0,
    'emergency_recovery_delay_sec': 0.0,

    'decelerate_to_gps_sec': 1.5,
    'decelerate_to_yolotl_sec': 1.5,
    'decelerate_to_rrt_sec': 1.5,

    # 안전 장치
    'auto_steer_angle_abs_max': 23.0,
    'auto_throttle_max': 1.0,
    'throttle_curve_k': 0.0,  # k가 0.0이면 선형 감속. 그리고 k가 커질수록 같은 steer에서 throttle이 더 빨리 min 쪽으로 내려간다.

    # 조향각: yolotl
    'auto_throttle_yolotl_min': 0.4,
    'auto_throttle_yolotl_max': 1.0,

    # 조향각: '/home/hannibal/Jejudol_ws/src/rrt_planning/src/rrt_purepursuit.py'
    'auto_throttle_rrt_min': 0.3,
    'auto_throttle_rrt_max': 0.6,

    # 조향각: '/home/hannibal/Jejudol_ws/src/gps_planning/scripts/gps_purepursuit.py'
    'auto_throttle_gps': 0.3,

    'manual_stop_use_spacebar': True,
    'mannual_deceleration_sec': 2.5,
}

# mission state 이름
MISSION_EMERGENCY = 'emergency'
MISSION_LANE = 'lane'
MISSION_GPS = 'gps'
MISSION_RRT = 'rrt'


# 동작 단계 요약 (위에서 아래로 우선순위 적용)
# 1) num_lidar_cone == 0 인 경우
#    - emergency=True -> mission_state=emergency
#    - 아니면 lane_detect=True -> mission_state=lane
#    - 모두 아니면 -> mission_state=gps
#
# 2) num_lidar_cone > 0 인 경우
#    - emergency=True -> mission_state=emergency
#    - 아니면 lane_detect=True -> mission_state=lane
#    - 모두 아니면 -> mission_state=rrt


# state별 제어
# - emergency : steer=yolotl, throttle은 현재값에서 emergency_deceleration_sec 동안
#               emergency_deceleration_target_throttle로 선형 감속
# - lane      : steer=yolotl, throttle=[yolotl_min, yolotl_max] 역비례 매핑
#               (lane로 전환 시 현재 throttle이 더 크면 decelerate_to_yolotl_sec 동안 선형 감속)
# - gps       : steer=gps, throttle=auto_throttle_gps(고정)
#               (gps로 전환 시 현재 throttle이 더 크면 decelerate_to_gps_sec 동안 선형 감속)
# - rrt       : steer=rrt, throttle=[rrt_min, rrt_max] 역비례 매핑
#               (rrt로 전환 시 현재 throttle이 더 크면 decelerate_to_rrt_sec 동안 선형 감속)
# 공통 안전장치: |auto_steer_angle| < auto_steer_angle_abs_max, 0<=auto_throttle<=auto_throttle_max


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # 실험값을 파라미터로 전부 노출
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)
        self.emergency_deceleration_sec = float(self.get_parameter('emergency_deceleration_sec').value)
        self.emergency_deceleration_target_throttle = float(
            self.get_parameter('emergency_deceleration_target_throttle').value
        )
        self.emergency_recovery_delay_sec = float(self.get_parameter('emergency_recovery_delay_sec').value)
        self.decelerate_to_gps_sec = float(self.get_parameter('decelerate_to_gps_sec').value)
        self.decelerate_to_yolotl_sec = float(self.get_parameter('decelerate_to_yolotl_sec').value)
        self.decelerate_to_rrt_sec = float(self.get_parameter('decelerate_to_rrt_sec').value)
        self.auto_steer_angle_abs_max = float(self.get_parameter('auto_steer_angle_abs_max').value)
        self.auto_throttle_max = float(self.get_parameter('auto_throttle_max').value)
        self.auto_throttle_yolotl_max = float(self.get_parameter('auto_throttle_yolotl_max').value)
        self.auto_throttle_yolotl_min = float(self.get_parameter('auto_throttle_yolotl_min').value)
        self.auto_throttle_rrt_max = float(self.get_parameter('auto_throttle_rrt_max').value)
        self.auto_throttle_rrt_min = float(self.get_parameter('auto_throttle_rrt_min').value)
        self.auto_throttle_gps = float(self.get_parameter('auto_throttle_gps').value)
        self.throttle_curve_k = float(self.get_parameter('throttle_curve_k').value)
        self.mannual_deceleration_sec = float(self.get_parameter('mannual_deceleration_sec').value)
        self.manual_stop_use_spacebar = bool(self.get_parameter('manual_stop_use_spacebar').value)

        # 파라미터 안전 보정
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = DEFAULTS['publish_rate_hz']
        if self.emergency_deceleration_sec < 0.0:
            self.emergency_deceleration_sec = DEFAULTS['emergency_deceleration_sec']
        if self.emergency_recovery_delay_sec < 0.0:
            self.emergency_recovery_delay_sec = DEFAULTS['emergency_recovery_delay_sec']
        if self.decelerate_to_gps_sec <= 0.0:
            self.decelerate_to_gps_sec = DEFAULTS['decelerate_to_gps_sec']
        if self.decelerate_to_yolotl_sec <= 0.0:
            self.decelerate_to_yolotl_sec = DEFAULTS['decelerate_to_yolotl_sec']
        if self.decelerate_to_rrt_sec <= 0.0:
            self.decelerate_to_rrt_sec = DEFAULTS['decelerate_to_rrt_sec']
        if self.log_rate_hz < 0.0:
            self.log_rate_hz = DEFAULTS['log_rate_hz']
        if self.auto_steer_angle_abs_max <= 0.0:
            self.auto_steer_angle_abs_max = DEFAULTS['auto_steer_angle_abs_max']
        if self.auto_throttle_max <= 0.0:
            self.auto_throttle_max = DEFAULTS['auto_throttle_max']
        self.emergency_deceleration_target_throttle = self._clamp(
            self.emergency_deceleration_target_throttle,
            0.0,
            self.auto_throttle_max,
        )
        if self.mannual_deceleration_sec <= 0.0:
            self.mannual_deceleration_sec = DEFAULTS['mannual_deceleration_sec']
        self.steer_limit = max(0.001, self.auto_steer_angle_abs_max - 1e-3)

        # 입력 상태
        self.lane_detect = False
        self.num_lidar_cone = 0
        self.emergency = False
        self.emergency_exit_time = -1.0e9
        self.auto_steer_angle_rrt = 0.0
        self.auto_steer_angle_yolotl = 0.0
        self.auto_steer_angle_gps = 0.0
        self.current_mission_state = MISSION_GPS

        # 긴급 감속 상태
        self.current_auto_throttle = 0.0
        self.decel_active = False
        self.decel_start_time = 0.0
        self.decel_start_throttle = 0.0
        self.transition_decel_active = False
        self.transition_decel_target_state = ''
        self.transition_decel_start_time = 0.0
        self.transition_decel_duration_sec = 0.0
        self.transition_decel_start_throttle = 0.0
        self.transition_decel_target_throttle = 0.0
        self._next_log_time = 0.0
        self.spacebar_stop_active = False
        self.spacebar_stop_start_time = 0.0
        self.spacebar_stop_start_throttle = 0.0
        self._spacebar_stop_done_logged = False

        # 키보드(space) 입력 상태
        self._spacebar_pending = False
        self._spacebar_pending_lock = threading.Lock()
        self._kbd_stop = False
        self._kbd_thread = None
        self._stdin_fd = None
        self._stdin_attr_backup = None

        # 입력 토픽 구독
        self.create_subscription(Bool, '/lane_detect', self._lane_detection_cb, 10)
        self.create_subscription(Int8, '/vn/num_lidar_cone', self._num_lidar_cone_cb, 10)
        self.create_subscription(Bool, '/emergency', self._emergency_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_rrt', self._steer_rrt_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_yolotl', self._steer_yolotl_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_gps', self._steer_gps_cb, 10)

        # 출력 토픽 상시 발행
        self.auto_steer_pub = self.create_publisher(Float32, '/auto_steer_angle', 10)
        self.auto_throttle_pub = self.create_publisher(Float32, '/auto_throttle', 10)
        self.mission_state_pub = self.create_publisher(String, '/mission_state', 10)

        self._start_manual_stop_listener()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            'decision2_node started: publish_rate=%.1fHz emergency_deceleration_sec=%.2fs '
            'emergency_target_throttle=%.3f '
            'decelerate_to_gps_sec=%.2fs decelerate_to_yolotl_sec=%.2fs decelerate_to_rrt_sec=%.2fs '
            'mannual_deceleration_sec=%.2fs auto_steer_angle_abs_max=%.2f auto_throttle_max=%.2f'
            % (
                self.publish_rate_hz,
                self.emergency_deceleration_sec,
                self.emergency_deceleration_target_throttle,
                self.decelerate_to_gps_sec,
                self.decelerate_to_yolotl_sec,
                self.decelerate_to_rrt_sec,
                self.mannual_deceleration_sec,
                self.auto_steer_angle_abs_max,
                self.auto_throttle_max,
            )
        )

    def _lane_detection_cb(self, msg: Bool):
        self.lane_detect = bool(msg.data)

    def _num_lidar_cone_cb(self, msg: Int8):
        self.num_lidar_cone = int(msg.data)

    def _emergency_cb(self, msg: Bool):
        self.emergency = bool(msg.data)

    def _steer_rrt_cb(self, msg: Float32):
        self.auto_steer_angle_rrt = float(msg.data)

    def _steer_yolotl_cb(self, msg: Float32):
        self.auto_steer_angle_yolotl = float(msg.data)

    def _steer_gps_cb(self, msg: Float32):
        self.auto_steer_angle_gps = float(msg.data)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _now_sec() -> float:
        return time.monotonic()

    def _clamp_steer(self, steer_value: float) -> float:
        # "< max" 조건을 만족시키기 위해 아주 작은 마진을 둠
        return self._clamp(steer_value, -self.steer_limit, self.steer_limit)

    def _compute_emergency_throttle(self, now_sec: float) -> float:
        # emergency=True:
        # 현재 throttle에서 target throttle까지 emergency_deceleration_sec 동안 선형 감속
        if not self.decel_active:
            self.decel_active = True
            self.decel_start_time = now_sec
            self.decel_start_throttle = self.current_auto_throttle

        target_throttle = self._clamp(
            self.emergency_deceleration_target_throttle,
            0.0,
            self.decel_start_throttle,  # emergency에서는 감속만 허용(가속 방지)
        )
        # emergency_deceleration_sec==0.0이면 즉시 target_throttle 적용
        if self.emergency_deceleration_sec <= 0.0:
            return target_throttle

        elapsed = now_sec - self.decel_start_time
        progress = self._clamp(elapsed / self.emergency_deceleration_sec, 0.0, 1.0)
        return self.decel_start_throttle + (
            (target_throttle - self.decel_start_throttle) * progress
        )

    def _cancel_transition_deceleration(self):
        self.transition_decel_active = False
        self.transition_decel_target_state = ''

    def _start_transition_deceleration(
        self,
        now_sec: float,
        target_state: str,
        target_throttle: float,
        duration_sec: float,
    ):
        self.transition_decel_active = True
        self.transition_decel_target_state = target_state
        self.transition_decel_start_time = now_sec
        self.transition_decel_duration_sec = max(duration_sec, 1e-6)
        self.transition_decel_start_throttle = max(0.0, self.current_auto_throttle)
        self.transition_decel_target_throttle = max(0.0, target_throttle)
        self.get_logger().info(
            '[TRANSITION-DECEL] %s: /auto_throttle %.3f -> %.3f in %.2fs'
            % (
                target_state,
                self.transition_decel_start_throttle,
                self.transition_decel_target_throttle,
                self.transition_decel_duration_sec,
            )
        )

    def _compute_transition_deceleration_throttle(self, now_sec: float) -> float:
        elapsed = now_sec - self.transition_decel_start_time
        progress = self._clamp(elapsed / self.transition_decel_duration_sec, 0.0, 1.0)
        throttle = self.transition_decel_start_throttle + (
            (self.transition_decel_target_throttle - self.transition_decel_start_throttle) * progress
        )
        if progress >= 1.0:
            self._cancel_transition_deceleration()
        return throttle

    def _consume_spacebar_request(self) -> bool:
        with self._spacebar_pending_lock:
            if not self._spacebar_pending:
                return False
            self._spacebar_pending = False
            return True

    def _activate_spacebar_stop(self, now_sec: float):
        self.spacebar_stop_active = True
        self.spacebar_stop_start_time = now_sec
        self.spacebar_stop_start_throttle = max(0.0, self.current_auto_throttle)
        self._spacebar_stop_done_logged = False
        self.get_logger().warn(
            '[MANUAL-STOP] stop requested: /auto_throttle %.3f -> 0.0 in %.1fs'
            % (
                self.spacebar_stop_start_throttle,
                self.mannual_deceleration_sec,
            )
        )

    def _deactivate_spacebar_stop(self):
        if not self.spacebar_stop_active:
            return
        self.spacebar_stop_active = False
        self._spacebar_stop_done_logged = False
        self.get_logger().warn(
            '[MANUAL-STOP] stop profile canceled: returning to mission throttle control'
        )

    def _toggle_spacebar_stop(self, now_sec: float):
        if self.spacebar_stop_active:
            self._deactivate_spacebar_stop()
        else:
            self._activate_spacebar_stop(now_sec)

    def _compute_spacebar_stop_throttle(self, now_sec: float) -> float:
        elapsed = now_sec - self.spacebar_stop_start_time
        progress = self._clamp(elapsed / self.mannual_deceleration_sec, 0.0, 1.0)
        throttle = self.spacebar_stop_start_throttle * (1.0 - progress)
        if (progress >= 1.0) and (not self._spacebar_stop_done_logged):
            self._spacebar_stop_done_logged = True
            self.get_logger().warn('[MANUAL-STOP] stop profile completed: /auto_throttle=0.000')
        return throttle

    def _start_keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().warn(
                '[KEYBOARD] listener disabled (stdin is not a TTY).'
            )
            return

        try:
            self._stdin_fd = sys.stdin.fileno()
            self._stdin_attr_backup = termios.tcgetattr(self._stdin_fd)
        except Exception as exc:
            self.get_logger().warn(
                f'[KEYBOARD] listener init failed: {exc}'
            )
            self._stdin_fd = None
            self._stdin_attr_backup = None
            return

        self._kbd_stop = False
        self._kbd_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kbd_thread.start()
        self.get_logger().info(
            '[KEYBOARD] press SPACE in this terminal to linearly decelerate /auto_throttle to 0.0'
        )

    def _start_manual_stop_listener(self):
        if self.manual_stop_use_spacebar:
            self._start_keyboard_listener()
            return
        if not self.manual_stop_use_spacebar:
            self.get_logger().warn(
                '[MANUAL-STOP] keyboard listener is disabled.'
            )

    def _restore_terminal(self):
        if (self._stdin_fd is None) or (self._stdin_attr_backup is None):
            return
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_attr_backup)
        except Exception:
            pass

    def _keyboard_loop(self):
        if self._stdin_fd is None:
            return

        try:
            tty.setcbreak(self._stdin_fd)
            while not self._kbd_stop:
                readable, _, _ = select.select([self._stdin_fd], [], [], 0.1)
                if not readable:
                    continue
                data = os.read(self._stdin_fd, 1)
                if data == b' ':
                    with self._spacebar_pending_lock:
                        self._spacebar_pending = True
        except Exception:
            pass
        finally:
            self._restore_terminal()

    def _map_throttle_inverse_by_steer(self, steer_cmd: float, throttle_max: float, throttle_min: float) -> float:
        # |steer|=0 에 가까울수록 throttle_max
        # |steer|=auto_steer_angle_abs_max 에 가까울수록 throttle_min
        if throttle_max < throttle_min:
            throttle_max, throttle_min = throttle_min, throttle_max

        ratio = self._clamp(
            abs(steer_cmd) / max(self.auto_steer_angle_abs_max, 1e-6),
            0.0,
            1.0,
        )
        k = max(0.0, self.throttle_curve_k)
        if k < 1e-6:
            shaped = ratio
        else:
            # k가 커질수록 같은 steer에서 throttle이 더 빨리 min 쪽으로 내려간다.
            # (0~1 정규화 유지: ratio=0 -> 0, ratio=1 -> 1)
            shaped = math.expm1(-k * ratio) / math.expm1(-k)

        return throttle_max - shaped * (throttle_max - throttle_min)

    def _get_mission_state(self) -> str:
        lane_detect = self.lane_detect
        num_lidar_cone = self.num_lidar_cone

        # 콘이 0개일 때: emergency > lane > gps
        if num_lidar_cone == 0:
            if self.emergency:
                return MISSION_EMERGENCY
            if lane_detect:
                return MISSION_LANE
            return MISSION_GPS

        # 콘이 1개 이상일 때: emergency > lane > rrt
        if self.emergency:
            return MISSION_EMERGENCY
        if lane_detect:
            return MISSION_LANE
        return MISSION_RRT

    def _maybe_log_status(self, steer_cmd: float, throttle_cmd: float):
        # log_rate_hz == 0.0 이면 매 tick마다 출력
        now = self._now_sec()
        if self.log_rate_hz > 0.0:
            if now < self._next_log_time:
                return
            self._next_log_time = now + (1.0 / self.log_rate_hz)

        self.get_logger().info(
            f'mission_state={self.current_mission_state}, '
            f'emergency={self.emergency}, '
            f'/auto_steer_angle={steer_cmd:.3f}, '
            f'/auto_throttle={throttle_cmd:.3f}'
        )

    def _on_timer(self):
        now_sec = self._now_sec()
        if self._consume_spacebar_request():
            self._toggle_spacebar_stop(now_sec)

        state = self._get_mission_state()
        prev_state = self.current_mission_state

        # emergency 상태에서 다른 상태로 방금 전환되었는지 확인하여 이탈 시간 기록
        if prev_state == MISSION_EMERGENCY and state != MISSION_EMERGENCY:
            self.emergency_exit_time = now_sec

        self.current_mission_state = state

        if self.transition_decel_active and state != self.transition_decel_target_state:
            self._cancel_transition_deceleration()

        if state == MISSION_EMERGENCY:
            self._cancel_transition_deceleration()
            steer_cmd = self._clamp_steer(self.auto_steer_angle_yolotl)
            throttle_cmd = self._compute_emergency_throttle(now_sec)
        elif state == MISSION_LANE:
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_yolotl)
            lane_target_throttle = self._map_throttle_inverse_by_steer(
                steer_cmd,
                self.auto_throttle_yolotl_max,
                self.auto_throttle_yolotl_min,
            )
            if prev_state != MISSION_LANE:
                if self.current_auto_throttle > lane_target_throttle:
                    self._start_transition_deceleration(
                        now_sec=now_sec,
                        target_state=MISSION_LANE,
                        target_throttle=lane_target_throttle,
                        duration_sec=self.decelerate_to_yolotl_sec,
                    )
                else:
                    self._cancel_transition_deceleration()
            if self.transition_decel_active and self.transition_decel_target_state == MISSION_LANE:
                throttle_cmd = self._compute_transition_deceleration_throttle(now_sec)
            else:
                throttle_cmd = lane_target_throttle
        elif state == MISSION_GPS:
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_gps)
            if prev_state != MISSION_GPS:
                if self.current_auto_throttle > self.auto_throttle_gps:
                    self._start_transition_deceleration(
                        now_sec=now_sec,
                        target_state=MISSION_GPS,
                        target_throttle=self.auto_throttle_gps,
                        duration_sec=self.decelerate_to_gps_sec,
                    )
                else:
                    self._cancel_transition_deceleration()
            if self.transition_decel_active and self.transition_decel_target_state == MISSION_GPS:
                throttle_cmd = self._compute_transition_deceleration_throttle(now_sec)
            else:
                throttle_cmd = self.auto_throttle_gps
        else:
            # MISSION_RRT
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_rrt)
            rrt_target_throttle = self._map_throttle_inverse_by_steer(
                steer_cmd,
                self.auto_throttle_rrt_max,
                self.auto_throttle_rrt_min,
            )
            if prev_state != MISSION_RRT:
                if self.current_auto_throttle > rrt_target_throttle:
                    self._start_transition_deceleration(
                        now_sec=now_sec,
                        target_state=MISSION_RRT,
                        target_throttle=rrt_target_throttle,
                        duration_sec=self.decelerate_to_rrt_sec,
                    )
                else:
                    self._cancel_transition_deceleration()
            if self.transition_decel_active and self.transition_decel_target_state == MISSION_RRT:
                throttle_cmd = self._compute_transition_deceleration_throttle(now_sec)
            else:
                throttle_cmd = rrt_target_throttle

        # 모든 state에서 publish 직전 최종 steer 안전장치 재적용
        steer_cmd = self._clamp_steer(steer_cmd)

        # emergency 종료 후 지정된 시간(초) 동안은 출발하지 않고 대기(Throttle 0.0 유지)
        if (now_sec - self.emergency_exit_time) < self.emergency_recovery_delay_sec:
            throttle_cmd = 0.0

        # 스페이스바 감속 오버라이드(미션 throttle보다 우선)
        if self.spacebar_stop_active:
            throttle_cmd = self._compute_spacebar_stop_throttle(now_sec)

        # throttle 공통 안전장치
        throttle_cmd = self._clamp(throttle_cmd, 0.0, self.auto_throttle_max)
        self.current_auto_throttle = throttle_cmd

        steer_msg = Float32()
        steer_msg.data = float(steer_cmd)
        self.auto_steer_pub.publish(steer_msg)

        throttle_msg = Float32()
        throttle_msg.data = float(throttle_cmd)
        self.auto_throttle_pub.publish(throttle_msg)

        mission_msg = String()
        mission_msg.data = self.current_mission_state
        self.mission_state_pub.publish(mission_msg)

        self._maybe_log_status(steer_cmd, throttle_cmd)

    def destroy_node(self):
        self._kbd_stop = True
        if self._kbd_thread is not None:
            self._kbd_thread.join(timeout=0.3)

        self._restore_terminal()
        super().destroy_node()


def main(args=None):
    # 모든 로그 메시지 맨 앞에 시간을 붙여 출력
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{time}] {message}'
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
