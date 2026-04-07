cmake_minimum_required(VERSION 3.16)
project(ros_omron_agv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(libaria CONFIG REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Omron.msg"
  "srv/DockRequest.srv"
)

add_executable(omron_laser_node src/laser_node.cpp)
ament_target_dependencies(omron_laser_node
  rclcpp
  sensor_msgs
)
target_link_libraries(omron_laser_node
  libaria::ArNetworking
)

add_executable(robot_status_node src/robot_status.cpp)
ament_target_dependencies(robot_status_node
  action_msgs
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(robot_status_node
  libaria::ArNetworking
)

add_executable(map_node src/map_server.cpp)
ament_target_dependencies(map_node
  nav_msgs
  rclcpp
)
target_link_libraries(map_node
  libaria::ArNetworking
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(robot_status_node ${cpp_typesupport_target})

install(TARGETS omron_laser_node robot_status_node map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
