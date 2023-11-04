cmake_minimum_required(VERSION 2.8.3)
project(sxl_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp

  tf2
  tf
  tf2_ros
  tf2_geometry_msgs
  message_filters
  std_msgs
  std_srvs
  geometry_msgs
#   nav_msgs
#   sensor_msgs
#   mavros_msgs
#   gmapping
#   map_server
#   amcl
  #   robot_localization
#   geographic_msgs
# robotnik_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    robot_localization
    roscpp
    tf
    tf2
    tf2_ros
    message_filters
    std_msgs
    std_srvs
    geometry_msgs
    nav_msgs
    sensor_msgs
    robotnik_msgs
    mavros_msgs
)

###########
## Build ##
###########

install(
  DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
