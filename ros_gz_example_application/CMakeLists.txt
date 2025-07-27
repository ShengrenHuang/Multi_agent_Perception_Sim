cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# # Gazebo
# find_package(gz-msgs9 REQUIRED)
# find_package(gz-transport12 REQUIRED)

add_executable(talker src/publisher_lambda_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_lambda_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(image_processor_node src/cascade_classifier.cpp)
ament_target_dependencies(image_processor_node
  rclcpp
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
)

add_executable(UAV_pid_control_node src/UAV_navigation.cpp)
ament_target_dependencies(UAV_pid_control_node
  rclcpp
  nav_msgs
  # ament_index_cpp
  # gz-msgs9::gz-msgs9
  # gz-transport12::gz-transport12
)

add_executable(Robot_pid_control_node src/Robot_navigation.cpp)
ament_target_dependencies(Robot_pid_control_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)

add_executable(odometry_publisher_node
  src/odometry.cpp
  src/odometry_publish.cpp
)


ament_target_dependencies(odometry_publisher_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/jazzy/include/cv_bridge
)

target_link_libraries(image_processor_node
  ${OpenCV_LIBS}
)




install(TARGETS 
    talker
    listener
    image_processor_node
    UAV_pid_control_node
    Robot_pid_control_node
    odometry_publisher_node
    DESTINATION lib/${PROJECT_NAME})


ament_package()
