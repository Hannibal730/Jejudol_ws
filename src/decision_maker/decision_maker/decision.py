#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8


# 전체 동작 순서 요약
# 1) /lane_detection_status 와 /vn/num_lidar_cone 로 2-a/2-b 단계 분기
#    - 2-a: lane_detection_status=True 이고 num_lidar_cone!=0
#    - 2-b: 그 외 모든 경우
# 2) 조향 선택
#    - 2-a: /auto_steer_angle_rrt 사용
#    - 2-b: num_lidar_cone==0 이면 /auto_steer_angle_yolotl, 아니면 /auto_steer_angle_rrt
# 3) 쓰로틀 계산
#    - 2-a에서 /emergency=True 면 현재 쓰로틀에서 deceleration_sec 동안 0.0으로 선형 감속
#    - 2-a에서 /emergency=False 면 moon_course_thottle 고정값 사용
#    - 2-b에서는 |steer| 비율로 0.0~auto_throttle_max 선형 매핑
# 4) 안전장치 적용
#    - |/auto_steer_angle| < auto_steer_abs_max
#    - 0.0 <= /auto_throttle <= auto_throttle_max
# 5) /auto_steer_angle, /auto_throttle 을 주기적으로 상시 발행
class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # ===== 사용자 요구사항 대응 파라미터 =====
        # deceleration_sec: 2-a 단계 emergency 시 감속 완료까지 걸리는 시간(초)
        # auto_steer_abs_max: /auto_steer_angle 절댓값 안전 상한
        # auto_throttle_max: /auto_throttle 안전 상한
        # moon_course_thottle: 2-a (비긴급) 고정 쓰로틀 값
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('deceleration_sec', 2.0)
        self.declare_parameter('auto_steer_abs_max', 17.0)
        self.declare_parameter('auto_throttle_max', 0.7)
        self.declare_parameter('moon_course_thottle', 0.2)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.deceleration_sec = float(self.get_parameter('deceleration_sec').value)
        self.auto_steer_abs_max = float(self.get_parameter('auto_steer_abs_max').value)
        self.auto_throttle_max = float(self.get_parameter('auto_throttle_max').value)
        self.moon_course_thottle = float(self.get_parameter('moon_course_thottle').value)

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

        self.lane_detection_status = False
        self.num_lidar_cone = 0
        self.emergency = False
        self.auto_steer_angle_rrt = 0.0
        self.auto_steer_angle_yolotl = 0.0

        self.current_auto_throttle = 0.0
        self.decel_active = False
        self.decel_start_time = 0.0
        self.decel_start_throttle = 0.0

        # ===== 입력 토픽(사용자 정의 1단계/2단계 판단 근거) =====
        self.create_subscription(
            Bool, '/lane_detection_status', self._lane_detection_cb, 10
        )
        self.create_subscription(
            Int8, '/vn/num_lidar_cone', self._num_lidar_cone_cb, 10
        )
        self.create_subscription(
            Bool, '/emergency', self._emergency_cb, 10
        )
        self.create_subscription(
            Float32, '/auto_steer_angle_rrt', self._steer_rrt_cb, 10
        )
        self.create_subscription(
            Float32, '/auto_steer_angle_yolotl', self._steer_yolotl_cb, 10
        )

        # ===== 출력 토픽(항상 발행) =====
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

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _choose_steer(self) -> float:
        # 1단계
        # lane_detection_status == True 이고 num_lidar_cone != 0 이면 2-a 단계
        # 아니면 2-b 단계
        stage_2a = self.lane_detection_status and (self.num_lidar_cone != 0)
        if stage_2a:
            # 2-a 단계(조향)
            # emergency 여부와 무관하게 조향은 RRT 값을 사용
            return self.auto_steer_angle_rrt

        # 2-b 단계(조향)
        # num_lidar_cone == 0 이면 yolotl, 아니면 rrt 사용
        cone_count_2b = self.num_lidar_cone
        if cone_count_2b == 0:
            return self.auto_steer_angle_yolotl
        return self.auto_steer_angle_rrt

    def _compute_emergency_throttle(self) -> float:
        # 2-a 단계에서 emergency=True 일 때:
        # 현재 throttle 값에서 0.0까지 deceleration_sec 동안 선형 감속
        now = time.time()
        if not self.decel_active:
            self.decel_active = True
            self.decel_start_time = now
            self.decel_start_throttle = self.current_auto_throttle

        elapsed = now - self.decel_start_time
        progress = self._clamp(elapsed / self.deceleration_sec, 0.0, 1.0)
        return self.decel_start_throttle * (1.0 - progress)

    def _on_timer(self):
        # 1단계 판별 결과를 timer 루프에서도 재사용
        stage_2a = self.lane_detection_status and (self.num_lidar_cone != 0)
        steer_cmd = self._choose_steer()

        # 안전장치: |auto_steer_angle| < auto_steer_abs_max
        steer_limit = max(0.001, self.auto_steer_abs_max - 1e-3)
        steer_cmd = self._clamp(steer_cmd, -steer_limit, steer_limit)

        if stage_2a:
            if self.emergency:
                # 2-a + emergency=True: 지정 시간에 걸쳐 0.0으로 감속
                throttle_cmd = self._compute_emergency_throttle()
            else:
                # 2-a + emergency=False: moon_course_thottle 고정 사용
                self.decel_active = False
                throttle_cmd = self.moon_course_thottle
        else:
            # 2-b: 기존 로직 유지(조향 절댓값 기반 쓰로틀 매핑)
            self.decel_active = False
            ratio = self._clamp(abs(steer_cmd) / max(self.auto_steer_abs_max, 1e-6), 0.0, 1.0)
            throttle_cmd = self.auto_throttle_max * ratio

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
