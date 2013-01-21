cmake_minimum_required (VERSION 2.8)
project (canopen)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options )

set( CMAKE_CXX_FLAGS -std=c++0x)

set(CMAKE_BUILD_TYPE Debug)

set(canlib_INCLUDE_DIR /usr/local/src/linuxcan/include)
set(canlib_LINK_DIR /usr/local/src/linuxcan/canlib)

catkin_package(
    INCLUDE_DIRS
        include
        ${canlib_INCLUDE_DIR}
    LIBRARIES
        ${PROJECT_NAME}
        canlib
    DEPENDS Boost
)

include_directories(
    include
    ${canlib_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

link_directories(
    ${canlib_LINK_DIR}
    )

add_subdirectory(src)

add_subdirectory(test)

