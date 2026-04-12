- '제5회 국제 대학생 EV 자율주행 경진대회' 1/5사이즈 부문을 위한 workspace
- 대회에 대한 자세한 정보는 다음 pdf 파일에 정리되어있으니까 내용을 직접 확인해본 뒤에 README.md에 추가하면 좋을 내용을 선별하여 추가해줘. C:\Users\Hannibal\Desktop\Jejudol_ws\제5회 국제 대학생 EV 자율주행 경진대회 1_5사이즈 부문.pdf 
- 팀원 이름은 다음과 같아. CHOI DAE SEUNG, YOO SEUNG HOON, KIM BUM SU, HAN SEON JU, KIM DO YUL
- 위 정보와 아래 정보를 바탕으로 각 명령문 (ros2 node)에 대한 자세한 설명을 제공하는 깃허브 readme.md를 작성해줘. 그리고 작성 과정을 담은 readme_process.md도 작성해줘. 너가 참고하면 좋을 래퍼런스는 C:\Users\Hannibal\Desktop\Jejudol_ws\README_reference.md 이거니까 참고해.
- 각 노드 설명에 대한 실제 각 노드 파일의 경로를 아래에 제공해줄게. 각 경로에서 파일의 내부 코드를 직접 전부 확인하고, 코드 내용을 바탕으로 각 노드마다의 설명을 추가해줘.

C:\Users\Hannibal\Desktop\Jejudol_ws\src\RTK_GPS_NTRIP\ublox_gps\launch\ublox_f9p_launch.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\RTK_GPS_NTRIP\ublox_gps\launch\ublox_f9r_launch.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\RTK_GPS_NTRIP\ntrip_client\launch\ntrip_client_launch.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\RTK_GPS_NTRIP\fix2nmea\src\fix2nmea.cpp
C:\Users\Hannibal\Desktop\Jejudol_ws\data\f9r_to_csv.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\utm_tf\config\tf_gps_csv.yaml
C:\Users\Hannibal\Desktop\Jejudol_ws\src\usb_cam\launch\camera.launch.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\yolotl_ros2\yolotl_ros2\main6.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\dynamic_cone\dynamic_cone\dynamic_cone.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\voxelnext_ros2\scripts\lidar_cone_detect.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\utm_tf\launch\tf_gps_csv.launch.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\gps_planning\src\f9r_roi_path.cpp
C:\Users\Hannibal\Desktop\Jejudol_ws\src\gps_planning\scripts\gps_purepursuit.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\rrt_planning\src\main.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\rrt_planning\src\rrt_purepursuit.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\rrt_planning\src\rrt_caution_purepursuit.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\decision_maker\decision_maker\decision.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\decision_maker\decision_maker\visualizer.py
C:\Users\Hannibal\Desktop\Jejudol_ws\src\serial_bridge\serial_bridge\serial_bridge.py




1. RTK 마운트 포인트 수정
src/RTK_GPS_NTRIP/ntrip_client/launch/ntrip_client_launch.py 에서 DeclareLaunchArgument('mountpoint', default_value='SUWN-RTCM31')

2.GNSS 센서와 RTK 모듈 스타팅
ros2 launch ublox_gps ublox_f9p_launch.py
ros2 launch ublox_gps ublox_f9r_launch.py
ros2 run fix2nmea fix2nmea
ros2 launch ntrip_client ntrip_client_launch.py

3. csv맵으로 변환 후 사용
\Jejudol_ws\data\f9r_to_csv.py 에서 백파일 경로 지정하고 실행하여 csv 맵을 data폴더에 저장
src/utm_tf/config/tf_gps_csv.yaml 에서 새로 저장한 csv맵으로 지정.

4. USB 카메라 센서 및 관련 모듈 스타팅
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video2" -p pixel_format:="mjpeg2rgb"
ros2 run yolotl_ros2 main6 (차선 검출 후 카메라 기반 주행 계획 경로 생성 후 pure-pursuit 알고리즘으로 조향각 산출)
ros2 run dynamic_cone dynamic_cone (카메라 기반 주행 계획 경로를 트래픽 콘이 가로막는지 판단)


5. 3D-Lidar 센서 및 관련 모듈 스타팅
rv (velodyne vlp-16 3D lidar sensor start)
ros2 run voxelnext_ros2 lidar_cone_detect.py (voxelnext 모델을 ros2 실시간 디텍 모듈로써 활용)

6. TF 모듈 스타팅 (csv, velodyne, f9r)
ros2 launch utm_tf tf_gps_csv.launch.py

7. csv맵 위에서 f9r 기반 roi 경로 생성
ros2 launch gps_planning f9r_roi_path.launch.py

8. GNSS 기반 pure-pursuit 알고리즘으로 조향각 산출
ros2 run gps_planning gps_purepursuit.py

8. 3D-Lidar & GNSS sensor fusion 기반 RRT 경로생성 모듈 및 pure-pursuit 알고리즘으로 조향각 산출
ros2 run rrt_planning main.py
ros2 run rrt_planning rrt_purepursuit.py
ros2 run rrt_planning rrt_caution_purepursuit

9. 조향각에 따른 차량 쓰로틀 제어를 위한 판단 로직과 대회 미션 코스를 위한 판단 로직을 담은 판단 노드 스타팅
ros2 run decision_maker decision

10. 대회 연습주행 시 판단 로직의 문제점 파악을 위한 실시간 로직 현황 비주얼라이저
ros2 run decision_maker visualizer

11. 아두이노의 시리얼 통신을 위한 ros2 topic의 시리얼 변환 브릿지 노드
ros2 run serial_bridge serial_bridge --ros-args -p port:=/dev/arduino_bridge