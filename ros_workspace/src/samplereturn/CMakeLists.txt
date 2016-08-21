cmake_minimum_required(VERSION 2.8.3)
project(samplereturn)

add_definitions("-std=c++11" "-Wall")

find_package(catkin REQUIRED
        COMPONENTS
        samplereturn_msgs
            rospy
        )

find_package(OpenCV)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
    LIBRARIES samplereturn
     CATKIN_DEPENDS
        rospy
)

catkin_add_env_hooks(
    00phidgets
    SHELLS sh
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(samplereturn
    src/utils/colormodel.cpp
    src/utils/mask_utils.cpp
)
target_link_libraries(samplereturn
    ${OpenCV_LIBRARIES}
    )
add_dependencies(samplereturn samplereturn_msgs_gencpp)

install(TARGETS samplereturn
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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

install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

install(
    PROGRAMS
       nodes/camlogger.py
       nodes/image_desync.py
       nodes/lights.py
       nodes/announce_voltage.py
       nodes/pause_switch.py
       nodes/announcer.py
       nodes/triggered_camera.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
