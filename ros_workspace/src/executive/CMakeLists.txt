cmake_minimum_required(VERSION 2.8.3)
project(executive)

find_package(catkin REQUIRED
        COMPONENTS
            rospy
            actionlib_msgs
            actionlib
            smach
            smach_ros
        )

catkin_python_setup()
    
catkin_package(
    CATKIN_DEPENDS
        rospy
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

install(
    PROGRAMS
       nodes/executive_master_node.py
       nodes/level_one_node.py
       nodes/level_two_node.py
       nodes/pursue_sample_node.py
       nodes/costmap_checker_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
