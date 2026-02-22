cmake_minimum_required(VERSION 3.5)
project(voxelnext_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(pcdet)

install(
    PROGRAMS
    scripts/3D_object_detect.py
    scripts/center_object_detect.py
    scripts/model.py

    DESTINATION lib/${PROJECT_NAME}
)

install(
    FILES
    DESTINATION lib/${PROJECT_NAME}
)

# install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )

ament_package()