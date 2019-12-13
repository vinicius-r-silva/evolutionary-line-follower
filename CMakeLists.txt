cmake_minimum_required(VERSION 2.8.3)
project(line_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)


add_message_files(
  FILES
  Num.msg
)

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  LIBRARIES line_follower
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
	IMAGE_PROCESSING  
	src/headers/image_processing.h
	src/image_processing.cpp
)

add_library(
	ROBOT_CONTROL  
	src/headers/robot_control.h
	src/robot_control.cpp
)

add_library(
	AG  
	src/headers/ag.h
	src/ag.cpp
)

add_executable(main 
  src/main.cpp
  src/image_processing.cpp
  src/robot_control.cpp
	src/ag.cpp
)
target_link_libraries(main 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
  IMAGE_PROCESSING
  ROBOT_CONTROL
  AG
)
add_dependencies(main line_follower_generate_messages_cpp)

add_executable(consts_test src/consts_test.cpp)
target_link_libraries(consts_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(consts_test line_follower_generate_messages_cpp)

















