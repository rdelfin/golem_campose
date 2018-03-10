cmake_minimum_required(VERSION 2.8.3)
project(golem_campose)

add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")
message("CMAKE CURRENT LIST DIR:  ${CMAKE_CURRENT_LIST_DIR}")
message("CMAKE MODULE PATH:       ${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  campose_msgs
  cv_bridge
)

# Exports OpenPose_INCLUDE_DIRS, OpenPose_LIBS, OpenPose_FOUND, et. al.
find_package(OpenCV REQUIRED)
find_package(OpenPose REQUIRED)
find_package(gflags REQUIRED)
find_package(FlyCapture2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy campose_msgs
)


include_directories(
  include
  ${OpenPose_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${FlyCapture2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(openpose_collector src/openpose_collector_node.cpp
                                  src/RostopicProducer.cpp)
add_dependencies(openpose_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(openpose_collector
  ${OpenPose_LIBS}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  gflags
)

add_executable(person_finder src/person_finder.cpp)
add_dependencies(person_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(person_finder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


add_executable(flycap_publisher src/flycap_publisher_node.cpp)
add_dependencies(flycap_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flycap_publisher
  ${catkin_LIBRARIES}
  ${FlyCapture2_LIBRARIES}
)
