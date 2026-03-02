cmake_minimum_required(VERSION 3.8)
project(rrt_planning)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  PROGRAMS
    src/main.py
    src/MaRRTPathPlanNode.py
    src/ma_rrt.py
    src/rrt_purepursuit.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
