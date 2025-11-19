cmake_minimum_required(VERSION 3.8)
project(mvs_ros2_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(/opt/MVS/include)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  link_directories(/opt/MVS/lib/aarch64/)
  message(STATUS "Running on aarch64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  link_directories(/opt/MVS/lib/64/)
  message(STATUS "Running on x86_64")
endif()

add_executable(mvs_camera_node src/grab_trigger.cpp)

target_include_directories(mvs_camera_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

ament_target_dependencies(mvs_camera_node
    cv_bridge
    image_transport
)

target_link_libraries(mvs_camera_node
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  MvCameraControl
)

install(TARGETS mvs_camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()