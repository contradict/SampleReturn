cmake_minimum_required(VERSION 2.8.3)
project(manipulator)

find_package(catkin REQUIRED
    COMPONENTS
        platform_motion
        rospy
        message_generation
        message_runtime
        actionlib
        dynamixel_controllers
        smach
        smach_ros
	manipulator_msgs
        )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
        message_runtime
        actionlib
)

install(
    PROGRAMS
        nodes/manipulator_node.py
        nodes/manipulator_test_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
foreach( d
        launch
        config
       )
     install(DIRECTORY ${d}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
        )
endforeach()
