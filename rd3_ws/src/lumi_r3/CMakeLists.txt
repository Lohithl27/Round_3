cmake_minimum_required(VERSION 3.8)
project(lumi_r3)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  lumi_r3/apriltag_detector.py
  lumi_r3/tile_detector.py
  lumi_r3/grid_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY worlds config launch rviz materials
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
