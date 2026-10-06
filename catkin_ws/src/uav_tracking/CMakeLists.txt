cmake_minimum_required(VERSION 3.0.2)
project(uav_tracking)

## 编译类型默认 Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  message_generation
)

## ------------------------------------------------------------------
## 自定义消息（新增 .msg 时把文件名追加进 FILES）
## ------------------------------------------------------------------
add_message_files(FILES
  CentroidArray.msg
  DetectedTarget.msg
  Detections.msg
  VisualTrack.msg
  VisualTrackArray.msg
  Track.msg
  TrackArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs mavros_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

## ------------------------------------------------------------------
## 安装资源目录（文件就位后取消注释）
## ------------------------------------------------------------------
# install(DIRECTORY launch config rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
