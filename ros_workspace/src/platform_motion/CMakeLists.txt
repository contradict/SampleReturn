cmake_minimum_required(VERSION 2.8.3)
project(platform_motion)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        roslib
        rosconsole
        message_generation
        actionlib_msgs
        actionlib
        geometry_msgs
        std_msgs
        tf
        dynamic_reconfigure
        canopen
        )

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)

# messages
add_message_files(
    DIRECTORY msg
    FILES
        GPIO.msg
)

# services
add_service_files(
    DIRECTORY srv
    FILES Enable
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
        message_runtime
        actionlib_msgs
        actionlib
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

add_executable(odometry src/odometry.cpp src/lmmin.c)
target_link_libraries(odometry
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(
    TARGETS
      motion_node
      teleop
      odometry
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
