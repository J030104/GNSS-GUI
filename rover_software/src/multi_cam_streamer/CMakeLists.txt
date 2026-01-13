cmake_minimum_required(VERSION 3.8)
project(multi_cam_streamer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartCameraStream.srv"
  "srv/StopCameraStream.srv"
  "srv/GetCameraStreamStatus.srv"
)

# Install Python scripts
install(PROGRAMS
  scripts/mock_rover_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
