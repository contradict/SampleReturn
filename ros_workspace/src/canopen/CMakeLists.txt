cmake_minimum_required (VERSION 2.8)
project (canopen)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options )

set( CMAKE_CXX_FLAGS -std=c++0x)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
        canlib
    DEPENDS Boost
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

add_subdirectory(src)

add_subdirectory(test)

install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    USE_SOURCE_PERMISSIONS
)
