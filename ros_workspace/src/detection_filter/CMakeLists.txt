cmake_minimum_required(VERSION 2.8.3)
project(detection_filter)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS tf rospy
  roscpp
  cmake_modules
  sensor_msgs
  cv_bridge
  dynamic_reconfigure
  image_geometry
  samplereturn_msgs
  samplereturn)

find_package(OpenCV)
find_package(Eigen REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

generate_dynamic_reconfigure_options(cfg/kalman_filter_params.cfg
    cfg/ManipulatorKalmanFilter.cfg)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp roslib
  DEPENDS opencv2
  )

###########
## Build ##
###########

## Specify additional locations of header files
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include_directories(include
   ${catkin_INCLUDE_DIRS}
   ${Eigen_INCLUDE_DIRS}
)

add_definitions( -std=gnu++0x )

## Declare a cpp library
# add_library(sample_detection
#   src/${PROJECT_NAME}/sample_detection.cpp
# )

## Declare a cpp executable
 add_executable(dslr_kalman_filter_node
     nodes/dslr_kalman_filter_node.cpp
     src/probability.cpp)
 add_executable(manipulator_kalman_filter_node
     nodes/manipulator_kalman_filter_node.cpp
     src/probability.cpp)

 add_dependencies(dslr_kalman_filter_node ${PROJECT_NAME}_gencfg)
 add_dependencies(manipulator_kalman_filter_node ${PROJECT_NAME}_gencfg)

## Add dependencies to the executable
 add_dependencies(dslr_kalman_filter_node samplereturn_msgs_gencpp)
 add_dependencies(manipulator_kalman_filter_node samplereturn_msgs_gencpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(dslr_kalman_filter_node
   ${catkin_LIBRARIES}
   ${eigen_LIBRARIES}
 )
 target_link_libraries(manipulator_kalman_filter_node
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
#install(DIRECTORY launch config
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#		USE_SOURCE_PERMISSIONS
#)

## Mark executables and/or libraries for installation
install(TARGETS dslr_kalman_filter_node
  manipulator_kalman_filter_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sample_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
