cmake_minimum_required(VERSION 3.8)
project(evolo_gimbal_remote_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(evolo_msgs REQUIRED)
find_package(z1_pro_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(gimbal_camera_remote_node
  src/main.cpp
  src/json_subscriber.cpp
)

target_include_directories(gimbal_camera_remote_node PRIVATE
  include
)

ament_target_dependencies(gimbal_camera_remote_node
  rclcpp
  std_msgs
  z1_pro_msgs
)

target_link_libraries(gimbal_camera_remote_node
  nlohmann_json::nlohmann_json
)

install(TARGETS
  gimbal_camera_remote_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
