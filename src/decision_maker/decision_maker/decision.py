#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8

# 실험용 변수 기본값(한 곳에서 관리)
DEFAULTS = {
    'publish_rate_hz': 20.0,
    'log_rate_hz': 1.0,
    'deceleration_sec': 1.5,
    'auto_steer_angle_abs_max': 23.0,
    'auto_throttle_max': 0.7,
    'auto_throttle_moon_course': 0.2,
    'auto_throttle_yolotl_max': 0.7,
    'auto_throttle_yolotl_min': 0.4,
    'auto_throttle_rrt_max': 0.7,
    'auto_throttle_rrt_min': 0.4,
    'auto_throttle_gps': 0.3,
    'auto_throttle_static_obstacle': 0.4,
    'num_static_obstacle_threshold': 4,
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
# 2-a) emergency=True  -> mission_state=emergency
#      emergency=False -> mission_state=moon_course
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

        # 파라미터 안전 보정
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = DEFAULTS['publish_rate_hz']
        if self.deceleration_sec <= 0.0:
            self.deceleration_sec = DEFAULTS['deceleration_sec']
        if self.log_rate_hz < 0.0:
            self.log_rate_hz = DEFAULTS['log_rate_hz']
        if self.auto_steer_angle_abs_max <= 0.0:
            self.auto_steer_angle_abs_max = DEFAULTS['auto_steer_angle_abs_max']
        if self.auto_throttle_max <= 0.0:
            self.auto_throttle_max = DEFAULTS['auto_throttle_max']
        if self.num_static_obstacle_threshold < 0:
            self.num_static_obstacle_threshold = DEFAULTS['num_static_obstacle_threshold']
        self.steer_limit = max(0.001, self.auto_steer_angle_abs_max - 1e-3)

        # 입력 상태
        self.lane_detection_status = False
        self.num_lidar_cone = 0
        self.emergency = False
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

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            'decision_node started: publish_rate=%.1fHz deceleration_sec=%.2fs '
            'auto_steer_angle_abs_max=%.2f auto_throttle_max=%.2f'
            % (
                self.publish_rate_hz,
                self.deceleration_sec,
                self.auto_steer_angle_abs_max,
                self.auto_throttle_max,
            )
        )

    def _lane_detection_cb(self, msg: Bool):
        self.lane_detection_status = bool(msg.data)

    def _num_lidar_cone_cb(self, msg: Int8):
        self.num_lidar_cone = int(msg.data)

    def _emergency_cb(self, msg: Bool):
        self.emergency = bool(msg.data)

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

    def _clamp_steer(self, steer_value: float) -> float:
        # "< max" 조건을 만족시키기 위해 아주 작은 마진을 둠
        return self._clamp(steer_value, -self.steer_limit, self.steer_limit)

    def _compute_emergency_throttle(self) -> float:
        # 2-a + emergency=True:
        # 현재 throttle에서 0.0까지 deceleration_sec 동안 선형 감속
        now = time.time()
        if not self.decel_active:
            self.decel_active = True
            self.decel_start_time = now
            self.decel_start_throttle = self.current_auto_throttle

        elapsed = now - self.decel_start_time
        progress = self._clamp(elapsed / self.deceleration_sec, 0.0, 1.0)
        return self.decel_start_throttle * (1.0 - progress)

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
            if self.emergency:
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
        now = time.time()
        if self.log_rate_hz > 0.0:
            if now < self._next_log_time:
                return
            self._next_log_time = now + (1.0 / self.log_rate_hz)

        self.get_logger().info(
            f'mission_state={self.current_mission_state}, '
            f'/auto_steer_angle={steer_cmd:.3f}, '
            f'/auto_throttle={throttle_cmd:.3f}'
        )

    def _on_timer(self):
        state = self._get_mission_state()
        self.current_mission_state = state

        if state == MISSION_EMERGENCY:
            steer_cmd = self._clamp_steer(self.auto_steer_angle_yolotl)
            throttle_cmd = self._compute_emergency_throttle()
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
