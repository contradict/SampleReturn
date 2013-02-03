cmake_minimum_required(VERSION 2.8.3)
project(samplereturn)

find_package(catkin REQUIRED
        COMPONENTS
            rospy
        )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
)

foreach( d
        launch
        config
        calibration
        Media
        urdf)
    install(DIRECTORY ${d}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
        )
endforeach()
