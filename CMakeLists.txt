cmake_minimum_required(VERSION 3.0.2)
project(acc_controller)

find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  message_generation
  pcl_ros
  cv_bridge
  waypoint_maker
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   jhs_msg.msg
   jhs_child.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs 
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES CM_LC
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs visualization_msgs pcl_ros std_msgs
  DEPENDS opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/acc_controller.cpp) 

target_link_libraries(${PROJECT_NAME}_node 
${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

