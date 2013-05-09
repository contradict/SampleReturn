cmake_minimum_required(VERSION 2.8.3)
project(platform_motion)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        roslib
        rospy
        rosconsole
        message_generation
        actionlib_msgs
        actionlib
        geometry_msgs
        std_msgs
        tf
        dynamic_reconfigure
        canopen
        bfl
        )

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)

# messages
add_message_files(
    DIRECTORY msg
    FILES
        GPIO.msg
        ServoStatus.msg
)

# services
add_service_files(
    DIRECTORY srv
    FILES
        Enable.srv
        SelectCommandSource.srv
)

# actions
add_action_files(
    DIRECTORY action
    FILES
        Home.action
        SelectCarouselBin.action
)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        std_msgs
        geometry_msgs
)

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
        message_generation
)

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/motion.cpp src/wheelpod.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_executable(motion_node src/motion_node.cpp)
target_link_libraries(motion_node
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop
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
    )
target_link_libraries(EKF_odometry
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(test_EKF_odometry
    src/odometry_node.cpp
    src/test_EKF_odometry.cpp
    src/wheelpodmeasurementpdf.cpp
    )
target_link_libraries(test_EKF_odometry
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(velocity_limiter
    src/velocity_limiter.cpp
    src/acceleration_limit.cpp
    )
target_link_libraries(velocity_limiter
    ${Boost_LIBRARIES}
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
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    PROGRAMS
        nodes/carousel_indexer.py
        nodes/sinusoid_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
