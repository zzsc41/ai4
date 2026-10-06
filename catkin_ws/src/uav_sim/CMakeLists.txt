cmake_minimum_required(VERSION 3.0.2)
project(uav_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  xacro
)

catkin_package()

## ------------------------------------------------------------------
## 安装仿真资源（文件就位后取消注释）
## ------------------------------------------------------------------
# install(DIRECTORY worlds launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
