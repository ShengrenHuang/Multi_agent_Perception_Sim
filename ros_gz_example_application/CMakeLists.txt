cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)


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


include_directories(
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
    DESTINATION lib/${PROJECT_NAME})


ament_package()
