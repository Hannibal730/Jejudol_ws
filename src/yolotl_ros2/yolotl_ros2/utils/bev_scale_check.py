#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
import argparse
import os


class BEVScaleClickNode(Node):
    def __init__(self, args):
        super().__init__('bev_scale_click_node')
        self.args = args

        # -----------------------------
        # BEV 파라미터 로드
        # -----------------------------
        try:
            params = np.load(self.args.bev_npz)
            self.src_points = params['src_points'].astype(np.float32)
            self.dst_points = params['dst_points'].astype(np.float32)
            self.warp_w = int(params['warp_w'])
            self.warp_h = int(params['warp_h'])
        except Exception as e:
            self.get_logger().error(f"Failed to load BEV npz: {e}")
            raise

        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)

        # -----------------------------
        # 클릭 관련 상태
        # -----------------------------
        self.mode = 'x'   # 'x' or 'y'
        self.x_points = []
        self.y_points = []

        self.m_per_pixel_x = None
        self.m_per_pixel_y = None

        self.latest_raw = None
        self.latest_bev = None
        self.hover_pt = None

        # -----------------------------
        # ROS image type 결정
        # -----------------------------
        if 'compressed' in self.args.topic:
            self.msg_type = CompressedImage
        else:
            self.msg_type = Image

        self.subscription = self.create_subscription(
            self.msg_type,
            self.args.topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f"Subscribing to: {self.args.topic}")
        self.get_logger().info(f"Loaded BEV params from: {self.args.bev_npz}")

        # -----------------------------
        # OpenCV window
        # -----------------------------
        self.window_name = "BEV Scale Click Tool"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        print("\n[사용 방법 - BEV 스케일 계산 툴]")
        print("1. raw 이미지를 받아 자동으로 BEV 화면으로 변환합니다.")
        print("2. x 키를 눌러 X축 점 클릭 모드로 전환")
        print("3. y 키를 눌러 Y축 점 클릭 모드로 전환")
        print("4. 실제 0.5m 간격 점들을 같은 축 방향으로 여러 개 클릭")
        print("   - X축 점: 같은 y선상에 있는 점들")
        print("   - Y축 점: 같은 x선상에 있는 점들")
        print("5. c 키를 눌러 m_per_pixel_x, m_per_pixel_y 계산")
        print("6. s 키를 눌러 결과 저장")
        print("7. u: 현재 모드 마지막 점 취소 / r: 전체 리셋 / q: 종료")
        print("-" * 60)
        print(f"현재 interval = {self.args.interval_m:.3f} m")
        print("추천: 각 축마다 4~6개 점 찍기\n")

    # =========================================================
    # 이미지 디코딩
    # =========================================================
    def decode_image(self, msg):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)

        if self.msg_type == Image:
            if msg.encoding == 'yuyv':
                frame = cv2.cvtColor(
                    np_arr.reshape((msg.height, msg.width, 2)),
                    cv2.COLOR_YUV2BGR_YUYV
                )
            elif msg.encoding == 'rgb8':
                frame = cv2.cvtColor(
                    np_arr.reshape((msg.height, msg.width, 3)),
                    cv2.COLOR_RGB2BGR
                )
            elif msg.encoding == 'bgr8':
                frame = np_arr.reshape((msg.height, msg.width, 3))
            else:
                self.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
                return None
        else:
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        return frame

    # =========================================================
    # BEV 변환
    # =========================================================
    def do_bev_transform(self, image):
        return cv2.warpPerspective(
            image,
            self.M,
            (self.warp_w, self.warp_h),
            flags=cv2.INTER_LINEAR
        )

    # =========================================================
    # 마우스 콜백
    # =========================================================
    def mouse_callback(self, event, x, y, flags, param):
        self.hover_pt = (x, y)

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mode == 'x':
                self.x_points.append((x, y))
                print(f"[X] point {len(self.x_points)} added: {(x, y)}")
            elif self.mode == 'y':
                self.y_points.append((x, y))
                print(f"[Y] point {len(self.y_points)} added: {(x, y)}")

    # =========================================================
    # 계산
    # =========================================================
    def compute_scale(self):
        if len(self.x_points) < 2:
            print("[ERROR] X축 점이 최소 2개 필요합니다.")
            return

        if len(self.y_points) < 2:
            print("[ERROR] Y축 점이 최소 2개 필요합니다.")
            return

        # X축: 첫 점과 끝 점의 x 차이 사용
        x_first = self.x_points[0]
        x_last = self.x_points[-1]
        x_pixel_dist = abs(x_last[0] - x_first[0])

        # Y축: 첫 점과 끝 점의 y 차이 사용
        y_first = self.y_points[0]
        y_last = self.y_points[-1]
        y_pixel_dist = abs(y_last[1] - y_first[1])

        real_x_dist = self.args.interval_m * (len(self.x_points) - 1)
        real_y_dist = self.args.interval_m * (len(self.y_points) - 1)

        if x_pixel_dist <= 0:
            print("[ERROR] X축 픽셀 거리 계산값이 0입니다.")
            return

        if y_pixel_dist <= 0:
            print("[ERROR] Y축 픽셀 거리 계산값이 0입니다.")
            return

        self.m_per_pixel_x = real_x_dist / x_pixel_dist
        self.m_per_pixel_y = real_y_dist / y_pixel_dist

        print("\n" + "=" * 60)
        print("[계산 결과]")
        print(f"X points count : {len(self.x_points)}")
        print(f"Y points count : {len(self.y_points)}")
        print(f"Real X dist    : {real_x_dist:.6f} m")
        print(f"Real Y dist    : {real_y_dist:.6f} m")
        print(f"Pixel X dist   : {x_pixel_dist:.6f} px")
        print(f"Pixel Y dist   : {y_pixel_dist:.6f} px")
        print(f"m_per_pixel_x  : {self.m_per_pixel_x:.9f}")
        print(f"m_per_pixel_y  : {self.m_per_pixel_y:.9f}")
        print("=" * 60 + "\n")

    # =========================================================
    # 저장
    # =========================================================
    def save_result(self):
        if self.m_per_pixel_x is None or self.m_per_pixel_y is None:
            print("[ERROR] 먼저 c 키로 계산부터 하세요.")
            return

        try:
            np.savez(
                self.args.out_npz,
                bev_npz=self.args.bev_npz,
                x_points=np.array(self.x_points, dtype=np.int32),
                y_points=np.array(self.y_points, dtype=np.int32),
                interval_m=float(self.args.interval_m),
                m_per_pixel_x=float(self.m_per_pixel_x),
                m_per_pixel_y=float(self.m_per_pixel_y),
                warp_w=int(self.warp_w),
                warp_h=int(self.warp_h)
            )

            with open(self.args.out_txt, 'w') as f:
                f.write("# BEV Scale Calibration Result\n")
                f.write(f"bev_npz: {os.path.abspath(self.args.bev_npz)}\n")
                f.write(f"interval_m: {self.args.interval_m}\n")
                f.write(f"warp_w: {self.warp_w}\n")
                f.write(f"warp_h: {self.warp_h}\n")
                f.write(f"m_per_pixel_x: {self.m_per_pixel_x:.9f}\n")
                f.write(f"m_per_pixel_y: {self.m_per_pixel_y:.9f}\n\n")

                f.write("# X-axis points\n")
                for i, (x, y) in enumerate(self.x_points):
                    f.write(f"X{i+1}: {x}, {y}\n")

                f.write("\n# Y-axis points\n")
                for i, (x, y) in enumerate(self.y_points):
                    f.write(f"Y{i+1}: {x}, {y}\n")

            print("[INFO] 저장 완료:")
            print(f"       NPZ -> {os.path.abspath(self.args.out_npz)}")
            print(f"       TXT -> {os.path.abspath(self.args.out_txt)}")

        except Exception as e:
            print(f"[ERROR] 저장 중 오류 발생: {e}")

    # =========================================================
    # 시각화
    # =========================================================
    def draw_overlay(self, img):
        out = img.copy()

        # Hover
        if self.hover_pt is not None:
            x, y = self.hover_pt
            cv2.circle(out, (x, y), 4, (0, 255, 255), -1)
            cv2.putText(out, f"({x},{y})", (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # X points (red)
        for i, (x, y) in enumerate(self.x_points):
            cv2.circle(out, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(out, f"X{i+1}", (x + 8, y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Y points (blue)
        for i, (x, y) in enumerate(self.y_points):
            cv2.circle(out, (x, y), 6, (255, 0, 0), -1)
            cv2.putText(out, f"Y{i+1}", (x + 8, y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # X / Y line
        if len(self.x_points) >= 2:
            cv2.line(out, self.x_points[0], self.x_points[-1], (0, 0, 255), 2)

        if len(self.y_points) >= 2:
            cv2.line(out, self.y_points[0], self.y_points[-1], (255, 0, 0), 2)

        infos = [
            f"Mode: {'X-axis' if self.mode == 'x' else 'Y-axis'}",
            f"Interval: {self.args.interval_m:.3f} m",
            f"X points: {len(self.x_points)}",
            f"Y points: {len(self.y_points)}",
            f"m/px X: {self.m_per_pixel_x:.9f}" if self.m_per_pixel_x is not None else "m/px X: N/A",
            f"m/px Y: {self.m_per_pixel_y:.9f}" if self.m_per_pixel_y is not None else "m/px Y: N/A",
            "x: X-mode, y: Y-mode, c: compute, u: undo, r: reset, s: save, q: quit"
        ]

        for i, text in enumerate(infos):
            cv2.putText(out, text, (10, 25 + i * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        return out

    # =========================================================
    # 키 입력 처리
    # =========================================================
    def process_key(self, key):
        if key == ord('q'):
            rclpy.shutdown()

        elif key == ord('x'):
            self.mode = 'x'
            print("[MODE] X-axis click mode")

        elif key == ord('y'):
            self.mode = 'y'
            print("[MODE] Y-axis click mode")

        elif key == ord('u'):
            if self.mode == 'x' and self.x_points:
                removed = self.x_points.pop()
                print(f"[UNDO X] removed {removed}")
                self.m_per_pixel_x = None
            elif self.mode == 'y' and self.y_points:
                removed = self.y_points.pop()
                print(f"[UNDO Y] removed {removed}")
                self.m_per_pixel_y = None

        elif key == ord('r'):
            self.x_points.clear()
            self.y_points.clear()
            self.m_per_pixel_x = None
            self.m_per_pixel_y = None
            print("[RESET] all points cleared")

        elif key == ord('c'):
            self.compute_scale()

        elif key == ord('s'):
            self.save_result()

    # =========================================================
    # ROS image callback
    # =========================================================
    def image_callback(self, msg):
        try:
            frame = self.decode_image(msg)
            if frame is None:
                return

            self.latest_raw = frame
            bev = self.do_bev_transform(frame)
            self.latest_bev = bev

            view = self.draw_overlay(bev)
            cv2.imshow(self.window_name, view)

            key = cv2.waitKey(1) & 0xFF
            self.process_key(key)

        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', type=str, default='/image_raw/compressed',
                        help='구독할 ROS2 이미지 토픽')
    parser.add_argument('--bev-npz', type=str, required=True,
                        help='BEV 파라미터 npz 파일')
    parser.add_argument('--interval-m', type=float, default=0.5,
                        help='점 간 실제 거리(m), 기본 0.5')
    parser.add_argument('--out-npz', type=str, default='bev_scale_result.npz',
                        help='계산 결과 저장 npz')
    parser.add_argument('--out-txt', type=str, default='bev_scale_result.txt',
                        help='계산 결과 저장 txt')
    parsed_args = parser.parse_args()

    node = BEVScaleClickNode(parsed_args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()