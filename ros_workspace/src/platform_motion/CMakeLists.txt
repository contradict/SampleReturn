cmake_minimum_required(VERSION 2.8.3)
project(platform_motion)

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
	platform_motion_msgs
	visual_servo_msgs
	manipulator_msgs
        )

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

# use c++0x
set( CMAKE_CXX_FLAGS -std=c++0x)

include_directories(${BFL_INCLUDE_DIRS})

message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})


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
    )

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/motion.cpp src/wheelpod.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(motion_node src/motion_node.cpp)
target_link_libraries(motion_node
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies(teleop ${${PROJECT_NAME}_EXPORTED_TARGETS})

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


catkin_add_gtest(test_acceleration_limit test/test_acceleration_limit.cpp
    src/acceleration_limit.cpp)

install(
    TARGETS
      ${PROJECT_NAME}
      motion_node
      teleop
      least_squares_odometry
      EKF_odometry
      test_path

    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    PROGRAMS
        nodes/carousel_indexer.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
