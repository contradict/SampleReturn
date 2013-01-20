cmake_minimum_required(VERSION 2.8.3)
project(robot_setup_tf)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        tf
        geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        tf
        geometry_msgs
)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})
