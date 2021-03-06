cmake_minimum_required(VERSION 2.8.3)
project(platform_motion)

# use c++0x
add_definitions(-std=c++0x)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        roslib
        rospy
        rosconsole
        message_generation
        actionlib
        tf
        tf_conversions
        eigen_conversions
        dynamic_reconfigure
        canopen
	    smach
        smach_ros
        platform_motion_msgs
	    manipulator_msgs
        cmake_modules
        )

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
# qt4_wrap_cpp(MOC_FILES
#   include/visualization/path_display.h
# )
#

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/PlatformParameters.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roslib
        roscpp
        rospy
        canopen
)

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${BFL_INCLUDE_DIRS}
    )

#common commands for building c++ executables and libraries
# add_library(${PROJECT_NAME} src/motion.cpp src/wheelpod.cpp)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(motion_node
    src/motion.cpp
    src/wheelpod.cpp
    src/motion_node.cpp)
target_link_libraries(motion_node
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies(motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(lls_odometry
    src/odometry_node.cpp
    src/LLS_odometry.cpp
    )
target_link_libraries(lls_odometry
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(least_squares_odometry
    src/odometry_node.cpp
    src/least_squares_odometry_node.cpp
    src/lmmin.c
    src/least_squares_odometry_lmmin.cpp)
target_link_libraries(least_squares_odometry
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(EKF_odometry
    src/odometry_node.cpp
    src/EKF_odometry.cpp
    src/EKF_odometry_node.cpp
    src/wheelpodmeasurementpdf.cpp
    src/odometrywithcurvature.cpp
    )
target_link_libraries(EKF_odometry
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(test_EKF_odometry
    src/odometry_node.cpp
    src/test_EKF_odometry.cpp
    src/wheelpodmeasurementpdf.cpp
    )
target_link_libraries(test_EKF_odometry
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(test_path
    src/scarytestpath.cpp
    )
target_link_libraries(test_path
    ${catkin_LIBRARIES}
    )
add_dependencies(test_path platform_motion_msgs_generate_messages)

# add_library(path_viz
#   src/path_display.cpp
#   src/path_visual.cpp
#   ${MOC_FILES}
#   )
# target_link_libraries(path_viz ${QT_LIBRARIES} ${catkin_LIBRARIES})
#
# add_dependencies(path_viz ${PROJECT_NAME}_gencfg)

catkin_add_gtest(test_acceleration_limit test/test_acceleration_limit.cpp
    src/acceleration_limit.cpp)

install(
    TARGETS
      motion_node
      least_squares_odometry
      EKF_odometry
      lls_odometry
      test_path
      #path_viz

    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    PROGRAMS
        nodes/carousel_indexer.py
        nodes/manual_control_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
