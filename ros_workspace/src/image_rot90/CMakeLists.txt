cmake_minimum_required(VERSION 2.8.3)
project(image_rot90)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        roslib
        dynamic_reconfigure
        sensor_msgs
        image_transport
        nodelet
        pluginlib
        cv_bridge
        )

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DynamicParameters.cfg
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roslib
        roscpp
    DEPENDS opencv2
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_library(nodelet_image_rot90 src/image_rot90_nodelet.cpp)
target_link_libraries(nodelet_image_rot90
    ${catkin_LIBRARIES}
    )

install(TARGETS nodelet_image_rot90
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES nodelet_image_rot90.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

