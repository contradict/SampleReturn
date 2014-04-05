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

catkin_add_env_hooks(
    00phidgets
    SHELLS sh
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

foreach( d
        launch
        config
        calibration
        Media
        urdf
        maps)
    install(DIRECTORY ${d}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
        )
endforeach()
install(
    PROGRAMS
       nodes/camlogger.py
       nodes/image_desync.py
       nodes/lights.py
       nodes/announce_voltage.py
       nodes/pause_switch.py
       nodes/announcer.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
