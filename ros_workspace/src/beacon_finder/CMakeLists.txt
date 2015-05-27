cmake_minimum_required(VERSION 2.8.3)
project(beacon_finder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
     rospy
     roscpp
     tf
     
     sensor_msgs
     geometry_msgs
     std_msgs
     message_generation
     
     cv_bridge
     image_transport
     image_geometry
     cmake_modules
     )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    AprilTagDetection.msg
    AprilTagDetectionArray.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions 
#add_action_files(
#	DIRECTORY action
#	FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sample_detection_filter
    CATKIN_DEPENDS
        rospy
		cv
		cv2
        std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_subdirectory(src/apriltags)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${apriltags_INCLUDE_DIRS})

## Declare a cpp library
# add_library(sample_detection_filter
#   src/${PROJECT_NAME}/sample_detection_filter.cpp
# )

## Declare a cpp executable
# add_executable(sample_deteciton_filter_node src/sample_detection_filter_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sample_detection_filter_node sample_detection_filter_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sample_detection_filter_node
#   ${catkin_LIBRARIES}
# )

add_executable(beacon_april_node src/beacon_april_node.cpp)

add_dependencies(beacon_april_node beacon_finder_generate_messages_cpp)

target_link_libraries(beacon_april_node
      ${catkin_LIBRARIES}
      apriltags
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(
    PROGRAMS
        nodes/beacon_finder.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		USE_SOURCE_PERMISSIONS
)

## Mark executables and/or libraries for installation
# install(TARGETS sample_detection_filter sample_detection_filter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sample_detection_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
