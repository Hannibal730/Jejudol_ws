#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8


# 전체 동작 순서 요약
# 1) /lane_detection_status 와 /vn/num_lidar_cone 로 2-a/2-b 분기
#    - 2-a: lane_detection_status=True and num_lidar_cone!=0
#    - 2-b: 위 조건 미충족
# 2) 2-a 단계
#    - steer: /auto_steer_angle_rrt
#    - emergency=True  -> 현재 throttle에서 deceleration_sec 동안 0.0으로 선형 감속
#    - emergency=False -> throttle=moon_course_thottle(기본 0.2) 고정
# 3) 2-b 단계
#    - lane_detection_status=True  -> steer=/auto_steer_angle_yolotl
#    - lane_detection_status=False -> 3단계로 이동
# 4) 3단계 (lane_detection_status=False 인 경우)
#    - num_lidar_cone!=0 -> steer=/auto_steer_angle_rrt
#    - num_lidar_cone==0 -> steer=/auto_steer_angle_gps
# 5) 안전장치
#    - |/auto_steer_angle| < auto_steer_abs_max
#    - 0.0 <= /auto_throttle <= auto_throttle_max
# 6) throttle 매핑 규칙
#    - 2-a 비긴급 고정값/2-a 긴급 감속을 제외한 경우:
#      |steer|가 0에 가까울수록 0.0, auto_steer_abs_max에 가까울수록 auto_throttle_max
# 7) /auto_steer_angle, /auto_throttle을 주기적으로 상시 발행
class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # 핵심 파라미터
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('deceleration_sec', 1.3)
        self.declare_parameter('auto_steer_abs_max', 17.0)
        self.declare_parameter('auto_throttle_max', 0.7)
        self.declare_parameter('moon_course_thottle', 0.2)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.deceleration_sec = float(self.get_parameter('deceleration_sec').value)
        self.auto_steer_abs_max = float(self.get_parameter('auto_steer_abs_max').value)
        self.auto_throttle_max = float(self.get_parameter('auto_throttle_max').value)
        self.moon_course_thottle = float(self.get_parameter('moon_course_thottle').value)

        # 잘못된 파라미터 입력에 대한 안전 기본값
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 20.0
        if self.deceleration_sec <= 0.0:
            self.deceleration_sec = 2.0
        if self.auto_steer_abs_max <= 0.0:
            self.auto_steer_abs_max = 17.0
        if self.auto_throttle_max <= 0.0:
            self.auto_throttle_max = 0.7
        if self.moon_course_thottle < 0.0:
            self.moon_course_thottle = 0.2

        # 입력 상태
        self.lane_detection_status = False
        self.num_lidar_cone = 0
        self.emergency = False
        self.auto_steer_angle_rrt = 0.0
        self.auto_steer_angle_yolotl = 0.0
        self.auto_steer_angle_gps = 0.0

        # 긴급 감속 상태
        self.current_auto_throttle = 0.0
        self.decel_active = False
        self.decel_start_time = 0.0
        self.decel_start_throttle = 0.0

        # 입력 토픽 구독
        self.create_subscription(Bool, '/lane_detection_status', self._lane_detection_cb, 10)
        self.create_subscription(Int8, '/vn/num_lidar_cone', self._num_lidar_cone_cb, 10)
        self.create_subscription(Bool, '/emergency', self._emergency_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_rrt', self._steer_rrt_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_yolotl', self._steer_yolotl_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle_gps', self._steer_gps_cb, 10)

        # 출력 토픽 상시 발행
        self.auto_steer_pub = self.create_publisher(Float32, '/auto_steer_angle', 10)
        self.auto_throttle_pub = self.create_publisher(Float32, '/auto_throttle', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            'decision_node started: publish_rate=%.1fHz deceleration_sec=%.2fs '
            'auto_steer_abs_max=%.2f auto_throttle_max=%.2f moon_course_thottle=%.2f'
            % (
                self.publish_rate_hz,
                self.deceleration_sec,
                self.auto_steer_abs_max,
                self.auto_throttle_max,
                self.moon_course_thottle,
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

    def _steer_yolotl_cb(self, msg: Float32):
        self.auto_steer_angle_yolotl = float(msg.data)

    def _steer_gps_cb(self, msg: Float32):
        self.auto_steer_angle_gps = float(msg.data)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _select_steer_and_mode(self):
        # 1단계: 2-a 조건 체크
        stage_2a = self.lane_detection_status and (self.num_lidar_cone != 0)
        if stage_2a:
            return '2a', self.auto_steer_angle_rrt

        # 2-b: 2-a 실패 시 진입
        if self.lane_detection_status:
            # 2-b: lane_detection_status=True -> yolotl 사용
            return '2b_lane_true', self.auto_steer_angle_yolotl

        # 3단계: 2-b에서 lane_detection_status=False 인 경우
        if self.num_lidar_cone != 0:
            return '3_rrt', self.auto_steer_angle_rrt
        return '3_gps', self.auto_steer_angle_gps

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

    def _compute_mapped_throttle(self, steer_cmd: float) -> float:
        # steer 절댓값 비례 매핑:
        # |steer|=0 -> 0.0, |steer|=auto_steer_abs_max 근접 -> auto_throttle_max
        ratio = self._clamp(
            abs(steer_cmd) / max(self.auto_steer_abs_max, 1e-6),
            0.0,
            1.0,
        )
        return self.auto_throttle_max * ratio

    def _on_timer(self):
        mode, steer_cmd = self._select_steer_and_mode()

        # 안전장치: |auto_steer_angle| < auto_steer_abs_max
        steer_limit = max(0.001, self.auto_steer_abs_max - 1e-3)
        steer_cmd = self._clamp(steer_cmd, -steer_limit, steer_limit)

        if mode == '2a':
            if self.emergency:
                throttle_cmd = self._compute_emergency_throttle()
            else:
                # 2-a + emergency=False: 고정 throttle
                self.decel_active = False
                throttle_cmd = self.moon_course_thottle
        else:
            # 2-b 및 3단계: steer 기반 throttle 매핑
            self.decel_active = False
            throttle_cmd = self._compute_mapped_throttle(steer_cmd)

        # 안전장치: 0.0 <= auto_throttle <= auto_throttle_max
        throttle_cmd = self._clamp(throttle_cmd, 0.0, self.auto_throttle_max)
        self.current_auto_throttle = throttle_cmd

        steer_msg = Float32()
        steer_msg.data = float(steer_cmd)
        self.auto_steer_pub.publish(steer_msg)

        throttle_msg = Float32()
        throttle_msg.data = float(throttle_cmd)
        self.auto_throttle_pub.publish(throttle_msg)


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
