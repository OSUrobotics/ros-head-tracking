# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(head_pose_estimation)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
  sensor_msgs
  cv_bridge
  pcl_ros
  image_transport
  angles
  message_filters
  dynamic_reconfigure
  people_msgs
  face_detector
)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(orocos_kdl REQUIRED) 

set( 	HEAD_DEMO
		src/CRForestEstimator.cpp
		src/CRTree.cpp
		src/main.cpp 
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
                   rospy
                   geometry_msgs
                   tf
                   sensor_msgs
                   cv_bridge
                   pcl_ros
                   image_transport
                   angles
                   message_filters
                   dynamic_reconfigure
                   people_msgs
                   face_detector
) 

add_executable( head_pose_estimation ${HEAD_DEMO})
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${orocos_kdl_LIBRARIES})


install(TARGETS
  head_pose_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY trees
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})