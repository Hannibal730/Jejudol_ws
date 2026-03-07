#!/usr/bin/env python3

import os
import re
import select
import subprocess
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8

# 실험용 변수 기본값(한 곳에서 관리)
DEFAULTS = {
    'publish_rate_hz': 20.0,
    'log_rate_hz': 1.0,
    'deceleration_sec': 1.5,
    'emergency_off_delay_sec': 0.8,
    'auto_steer_angle_abs_max': 23.0,
    'auto_throttle_max': 0.7,
    'auto_throttle_moon_course': 0.2,
    'auto_throttle_yolotl_max': 0.4,
    'auto_throttle_yolotl_min': 0.1,
    'auto_throttle_rrt_max': 0.7,
    'auto_throttle_rrt_min': 0.4,
    'auto_throttle_gps': 0.2,
    'auto_throttle_static_obstacle': 0.4,
    'num_static_obstacle_threshold': 4,
    'spacebar_deceleration_sec': 5.0,
    'manual_stop_use_spacebar': True,
    'manual_stop_use_mouse': True,
    'mouse_button_code': 8,
    'mouse_trigger_on_release': False,
    'mouse_device_id': -1,
    'mouse_device_name': 'Logitech USB Receiver Mouse',
}

# mission state 이름(요청 반영)
MISSION_EMERGENCY = 'emergency'
MISSION_MOON_COURSE = 'moon_course'
MISSION_LANE = 'lane'
MISSION_GPS = 'gps'
MISSION_STATIC_OBSTACLE = 'static_obstacle'
MISSION_RRT = 'rrt'


# 동작 단계 요약
# 1) lane_detection_status=True and num_lidar_cone!=0 -> 2-a, 아니면 2-b
# 2-a) emergency_active=True  -> mission_state=emergency
#      emergency_active=False -> mission_state=moon_course
#      (emergency_active는 /emergency raw 신호에 off-delay 필터를 적용한 값.)
#      (false가 잠깐 들어와도, 마지막 true 시점으로부터 emergency_off_delay_sec가 지나기 전까지는 emergency_active를 계속 true로 유지)
#      (감속 계획이 리셋되는 경우는 emergency 상태를 벗어날 때(decel_active=False로 바뀔 때))
# 2-b) lane_detection_status=True -> mission_state=lane
# 3)   num_lidar_cone==0 -> mission_state=gps
# 4)   num_lidar_cone>=threshold -> mission_state=static_obstacle
#      num_lidar_cone<threshold  -> mission_state=rrt
# state별 제어
# - emergency       : steer=yolotl, throttle은 현재값에서 deceleration_sec 동안 0.0으로 선형 감속
# - moon_course     : steer=rrt_caution, throttle=auto_throttle_moon_course(고정)
# - lane            : steer=yolotl, throttle=[yolotl_min, yolotl_max] 역비례 매핑
# - gps             : steer=gps, throttle=auto_throttle_gps(고정)
# - static_obstacle : steer=rrt_caution, throttle=auto_throttle_static_obstacle(고정)
# - rrt             : steer=rrt, throttle=[rrt_min, rrt_max] 역비례 매핑
# 5)   throttle 매핑 경로(2-b의 yolotl, 4단계의 rrt)에서는 매핑 전에 steer에 auto_steer_angle_abs_max 안전장치를 먼저 적용
# 공통 안전장치: |auto_steer_angle| < auto_steer_angle_abs_max, 0<=auto_throttle<=auto_throttle_max
class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # 실험값을 파라미터로 전부 노출
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)
        self.deceleration_sec = float(self.get_parameter('deceleration_sec').value)
        self.emergency_off_delay_sec = float(self.get_parameter('emergency_off_delay_sec').value)
        self.auto_steer_angle_abs_max = float(self.get_parameter('auto_steer_angle_abs_max').value)
        self.auto_throttle_max = float(self.get_parameter('auto_throttle_max').value)
        self.auto_throttle_moon_course = float(self.get_parameter('auto_throttle_moon_course').value)
        self.auto_throttle_yolotl_max = float(self.get_parameter('auto_throttle_yolotl_max').value)
        self.auto_throttle_yolotl_min = float(self.get_parameter('auto_throttle_yolotl_min').value)
        self.auto_throttle_rrt_max = float(self.get_parameter('auto_throttle_rrt_max').value)
        self.auto_throttle_rrt_min = float(self.get_parameter('auto_throttle_rrt_min').value)
        self.auto_throttle_gps = float(self.get_parameter('auto_throttle_gps').value)
        self.auto_throttle_static_obstacle = float(self.get_parameter('auto_throttle_static_obstacle').value)
        self.num_static_obstacle_threshold = int(self.get_parameter('num_static_obstacle_threshold').value)
        self.spacebar_deceleration_sec = float(self.get_parameter('spacebar_deceleration_sec').value)
        self.manual_stop_use_spacebar = bool(self.get_parameter('manual_stop_use_spacebar').value)
        self.manual_stop_use_mouse = bool(self.get_parameter('manual_stop_use_mouse').value)
        self.mouse_button_code = int(self.get_parameter('mouse_button_code').value)
        self.mouse_trigger_on_release = bool(self.get_parameter('mouse_trigger_on_release').value)
        self.mouse_device_id = int(self.get_parameter('mouse_device_id').value)
        self.mouse_device_name = str(self.get_parameter('mouse_device_name').value)

        # 파라미터 안전 보정
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = DEFAULTS['publish_rate_hz']
        if self.deceleration_sec <= 0.0:
            self.deceleration_sec = DEFAULTS['deceleration_sec']
        if self.emergency_off_delay_sec < 0.0:
            self.emergency_off_delay_sec = DEFAULTS['emergency_off_delay_sec']
        if self.log_rate_hz < 0.0:
            self.log_rate_hz = DEFAULTS['log_rate_hz']
        if self.auto_steer_angle_abs_max <= 0.0:
            self.auto_steer_angle_abs_max = DEFAULTS['auto_steer_angle_abs_max']
        if self.auto_throttle_max <= 0.0:
            self.auto_throttle_max = DEFAULTS['auto_throttle_max']
        if self.num_static_obstacle_threshold < 0:
            self.num_static_obstacle_threshold = DEFAULTS['num_static_obstacle_threshold']
        if self.spacebar_deceleration_sec <= 0.0:
            self.spacebar_deceleration_sec = DEFAULTS['spacebar_deceleration_sec']
        if self.mouse_button_code <= 0:
            self.mouse_button_code = DEFAULTS['mouse_button_code']
        self.steer_limit = max(0.001, self.auto_steer_angle_abs_max - 1e-3)

        # 입력 상태
        self.lane_detection_status = False
        self.num_lidar_cone = 0
        self.emergency_raw = False
        self.emergency_active = False
        self.last_emergency_true_time = -1.0e9
        self.auto_steer_angle_rrt = 0.0
        self.auto_steer_angle_rrt_caution = 0.0
        self.auto_steer_angle_yolotl = 0.0
        self.auto_steer_angle_gps = 0.0
        self.current_mission_state = MISSION_GPS

        # 긴급 감속 상태
        self.current_auto_throttle = 0.0
        self.decel_active = False
        self.decel_start_time = 0.0
        self.decel_start_throttle = 0.0
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
        self._mouse_stop = False
        self._mouse_thread = None
        self._mouse_proc = None
        self._resolved_mouse_device_id = None

        # 입력 토픽 구독
        self.create_subscription(Bool, '/lane_detection_status', self._lane_detection_cb, 10)
        self.create_subscription(Int8, '/vn/num_lidar_cone', self._num_lidar_cone_cb, 10)
        self.create_subscription(Bool, '/emergency', self._emergency_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_rrt', self._steer_rrt_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_rrt_caution', self._steer_rrt_caution_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_yolotl', self._steer_yolotl_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_gps', self._steer_gps_cb, 10)

        # 출력 토픽 상시 발행
        self.auto_steer_pub = self.create_publisher(Float32, '/auto_steer_angle', 10)
        self.auto_throttle_pub = self.create_publisher(Float32, '/auto_throttle', 10)

        self._start_manual_stop_listener()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            'decision_node started: publish_rate=%.1fHz deceleration_sec=%.2fs '
            'spacebar_deceleration_sec=%.2fs auto_steer_angle_abs_max=%.2f auto_throttle_max=%.2f'
            % (
                self.publish_rate_hz,
                self.deceleration_sec,
                self.spacebar_deceleration_sec,
                self.auto_steer_angle_abs_max,
                self.auto_throttle_max,
            )
        )

    def _lane_detection_cb(self, msg: Bool):
        self.lane_detection_status = bool(msg.data)

    def _num_lidar_cone_cb(self, msg: Int8):
        self.num_lidar_cone = int(msg.data)

    def _emergency_cb(self, msg: Bool):
        self.emergency_raw = bool(msg.data)
        if self.emergency_raw:
            self.last_emergency_true_time = self._now_sec()

    def _steer_rrt_cb(self, msg: Float32):
        self.auto_steer_angle_rrt = float(msg.data)

    def _steer_rrt_caution_cb(self, msg: Float32):
        self.auto_steer_angle_rrt_caution = float(msg.data)

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

    def _compute_emergency_active(self, now_sec: float) -> bool:
        if self.emergency_raw:
            return True
        return (now_sec - self.last_emergency_true_time) <= self.emergency_off_delay_sec

    def _compute_emergency_throttle(self, now_sec: float) -> float:
        # 2-a + emergency=True:
        # 현재 throttle에서 0.0까지 deceleration_sec 동안 선형 감속
        if not self.decel_active:
            self.decel_active = True
            self.decel_start_time = now_sec
            self.decel_start_throttle = self.current_auto_throttle

        elapsed = now_sec - self.decel_start_time
        progress = self._clamp(elapsed / self.deceleration_sec, 0.0, 1.0)
        return self.decel_start_throttle * (1.0 - progress)

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
                self.spacebar_deceleration_sec,
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
        progress = self._clamp(elapsed / self.spacebar_deceleration_sec, 0.0, 1.0)
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

    def _resolve_mouse_device_id(self):
        if self.mouse_device_id >= 0:
            return self.mouse_device_id

        try:
            output = subprocess.check_output(['xinput', 'list', '--short'], text=True)
        except Exception as exc:
            self.get_logger().warn(
                f'[MOUSE] failed to run xinput list --short: {exc}'
            )
            return None

        for raw_line in output.splitlines():
            line = raw_line.strip()
            if (self.mouse_device_name in line) and ('[slave  pointer' in line):
                match = re.search(r'id=(\d+)', line)
                if match is not None:
                    return int(match.group(1))
        return None

    def _start_mouse_listener(self):
        device_id = self._resolve_mouse_device_id()
        if device_id is None:
            self.get_logger().warn(
                '[MOUSE] listener disabled: target pointer device not found. '
                'set mouse_device_id manually or check mouse_device_name.'
            )
            return

        cmd = ['xinput', 'test', str(device_id)]
        try:
            self._mouse_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            self.get_logger().warn(
                f'[MOUSE] listener init failed: {exc}'
            )
            self._mouse_proc = None
            return

        if self._mouse_proc.stdout is None:
            self.get_logger().warn('[MOUSE] listener init failed: no stdout pipe.')
            return

        self._resolved_mouse_device_id = device_id
        self._mouse_stop = False
        self._mouse_thread = threading.Thread(target=self._mouse_loop, daemon=True)
        self._mouse_thread.start()
        trigger_event = 'release' if self.mouse_trigger_on_release else 'press'
        self.get_logger().info(
            '[MOUSE] listening xinput id=%d, trigger=button %s %d'
            % (
                self._resolved_mouse_device_id,
                trigger_event,
                self.mouse_button_code,
            )
        )

    def _start_manual_stop_listener(self):
        enabled = False
        if self.manual_stop_use_spacebar:
            self._start_keyboard_listener()
            enabled = True
        if self.manual_stop_use_mouse:
            self._start_mouse_listener()
            enabled = True
        if not enabled:
            self.get_logger().warn(
                '[MANUAL-STOP] all manual stop listeners are disabled.'
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

    def _mouse_loop(self):
        if (self._mouse_proc is None) or (self._mouse_proc.stdout is None):
            return

        target_event = 'release' if self.mouse_trigger_on_release else 'press'
        try:
            while not self._mouse_stop:
                line = self._mouse_proc.stdout.readline()
                if line == '':
                    if self._mouse_proc.poll() is not None:
                        break
                    continue
                normalized = line.strip().lower()
                matched = re.match(r'^button\s+(press|release)\s+(\d+)$', normalized)
                if matched is None:
                    continue
                event_name = matched.group(1)
                button_code = int(matched.group(2))
                if (event_name == target_event) and (button_code == self.mouse_button_code):
                    with self._spacebar_pending_lock:
                        self._spacebar_pending = True
        except Exception:
            pass

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
        return throttle_max - ratio * (throttle_max - throttle_min)

    def _get_mission_state(self) -> str:
        # 처리 속도와 가독성을 위해 상태를 한 번만 판정
        lane_on = self.lane_detection_status
        cone_count = self.num_lidar_cone

        if lane_on and (cone_count != 0):
            if self.emergency_active:
                return MISSION_EMERGENCY
            return MISSION_MOON_COURSE

        if lane_on:
            return MISSION_LANE

        if cone_count == 0:
            return MISSION_GPS

        if cone_count >= self.num_static_obstacle_threshold:
            return MISSION_STATIC_OBSTACLE

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
            f'emergency_raw={self.emergency_raw}, emergency_active={self.emergency_active}, '
            f'/auto_steer_angle={steer_cmd:.3f}, '
            f'/auto_throttle={throttle_cmd:.3f}'
        )

    def _on_timer(self):
        now_sec = self._now_sec()
        if self._consume_spacebar_request():
            self._toggle_spacebar_stop(now_sec)

        self.emergency_active = self._compute_emergency_active(now_sec)
        state = self._get_mission_state()
        self.current_mission_state = state

        if state == MISSION_EMERGENCY:
            steer_cmd = self._clamp_steer(self.auto_steer_angle_yolotl)
            throttle_cmd = self._compute_emergency_throttle(now_sec)
        elif state == MISSION_MOON_COURSE:
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_rrt_caution)
            throttle_cmd = self.auto_throttle_moon_course
        elif state == MISSION_LANE:
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_yolotl)
            throttle_cmd = self._map_throttle_inverse_by_steer(
                steer_cmd,
                self.auto_throttle_yolotl_max,
                self.auto_throttle_yolotl_min,
            )
        elif state == MISSION_GPS:
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_gps)
            throttle_cmd = self.auto_throttle_gps
        elif state == MISSION_STATIC_OBSTACLE:
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_rrt_caution)
            throttle_cmd = self.auto_throttle_static_obstacle
        else:
            # MISSION_RRT
            self.decel_active = False
            steer_cmd = self._clamp_steer(self.auto_steer_angle_rrt)
            throttle_cmd = self._map_throttle_inverse_by_steer(
                steer_cmd,
                self.auto_throttle_rrt_max,
                self.auto_throttle_rrt_min,
            )

        # 모든 state에서 publish 직전 최종 steer 안전장치 재적용
        steer_cmd = self._clamp_steer(steer_cmd)

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
        self._maybe_log_status(steer_cmd, throttle_cmd)

    def destroy_node(self):
        self._kbd_stop = True
        if self._kbd_thread is not None:
            self._kbd_thread.join(timeout=0.3)

        self._mouse_stop = True
        if self._mouse_proc is not None:
            try:
                self._mouse_proc.terminate()
            except Exception:
                pass
        if self._mouse_thread is not None:
            self._mouse_thread.join(timeout=0.5)
        if self._mouse_proc is not None and self._mouse_proc.poll() is None:
            try:
                self._mouse_proc.kill()
            except Exception:
                pass

        self._restore_terminal()
        super().destroy_node()


def main(args=None):
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
