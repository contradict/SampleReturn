cmake_minimum_required(VERSION 2.8.3)
project(executive)

find_package(catkin REQUIRED
        COMPONENTS
            rospy
        )

catkin_package(
    CATKIN_DEPENDS
        rospy
)

foreach( d
        launch
        )
    install(DIRECTORY ${d}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
        )
endforeach()

install(
    PROGRAMS
       nodes/master_executive.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
