

ros2 run yolotl_ros2 lane_follower


백파일 input
ros2 bag play ~/Downloads/test13 -l

카메라 input은 도율이가 작업 중



토픽 list
j:~$ ros2 topic list
/auto_steer_angle_lane
/auto_throttle
/drivable_area
/events/read_split
/image_raw/compressed
/lane_detection_status
/lane_path
/lookahead_distance
/parameter_events
/rosout


plothuggler 실행
source /opt/ros/humble/setup.bash
ros2 run plotjuggler plotjuggler