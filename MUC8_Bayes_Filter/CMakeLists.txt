cmake_minimum_required(VERSION 3.0.2)
project(MUC8_Bayes_Filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  message_filters
  tf
  tf2
)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mus2_pr5
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)
add_executable(${PROJECT_NAME}_task src/task.cpp)
add_executable(${PROJECT_NAME}_laser src/laser.cpp)
add_executable(${PROJECT_NAME}_evaluation src/evaluation.cpp)
add_executable(${PROJECT_NAME}_parameters src/parameters.cpp)
add_executable(${PROJECT_NAME}_amclPose src/amclPose.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
  target_link_libraries(${PROJECT_NAME}_task
   ${catkin_LIBRARIES}
 )
  target_link_libraries(${PROJECT_NAME}_laser
   ${catkin_LIBRARIES}
 )
  target_link_libraries(${PROJECT_NAME}_evaluation
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

  target_link_libraries(${PROJECT_NAME}_parameters
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

 target_link_libraries(${PROJECT_NAME}_amclPose
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
