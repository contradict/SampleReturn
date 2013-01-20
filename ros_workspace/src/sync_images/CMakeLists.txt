cmake_minimum_required(VERSION 2.8.3)
project(sync_images)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        image_transport
        sensor_msgs
        message_filters
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        image_transport
        sensor_msgs
        message_filters
)

set (BINNAME sync_images)

# Set the source files to use with the executable.
set (SRCS ${SRCS} src/sync_images_core.cpp)
set (SRCS ${SRCS} src/sync_images.cpp)

# Set the directories where include files can be found.
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# Build the executable that will be used to run this node.
add_executable (${BINNAME} ${SRCS})
target_link_libraries(${BINNAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
