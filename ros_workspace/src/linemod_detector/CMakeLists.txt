cmake_minimum_required(VERSION 2.8.3)
project(linemod_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        roslib
        sensor_msgs
        stereo_msgs
        std_msgs
        samplereturn_msgs
        image_transport
        cv_bridge
        message_generation
        image_geometry
        saliency_detector
        tf
        cmake_modules
        )

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   NamedPoint.msg
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
#   std_msgs  # Or other packages containing msgs
#   sensor_msgs
#   geometry_msgs
# )
#
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   LIBRARIES ${PROJECT_NAME}
   INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
   CATKIN_DEPENDS roscpp roslib
   DEPENDS opencv2 Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${Eigen_DEFINITIONS})

## Declare a cpp library
 add_library(new_modalities
   src/new_modalities.cpp
 )

## Declare a cpp executable
 add_executable(linemod_trainer src/linemod_trainer.cpp)
 add_executable(linemod_detector src/linemod_detector.cpp)
 add_executable(linemod_detector_2d src/linemod_detector_2d.cpp)
 add_executable(stereo_linemod_trainer src/stereo_linemod_trainer.cpp)

add_dependencies(linemod_detector ${PROJECT_NAME}_gencpp)
add_dependencies(linemod_detector_2d ${PROJECT_NAME}_gencpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cv_test_node cv_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#add_library(linemod_color_naming
#  src/color_naming.cpp
#)

 target_link_libraries(linemod_trainer
   new_modalities
   ${catkin_LIBRARIES}
 )
 target_link_libraries(stereo_linemod_trainer
   new_modalities
   ${catkin_LIBRARIES}
 )
 target_link_libraries(linemod_detector
   ${catkin_LIBRARIES}
 )
 target_link_libraries(linemod_detector_2d
   new_modalities
   #color_naming
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS new_modalities linemod_trainer linemod_detector linemod_detector_2d
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )
install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cv_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
