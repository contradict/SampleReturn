cmake_minimum_required (VERSION 2.8)
set(CMAKE_COLOR_MAKEFILE on)

project (CANOPENLib)
set( CMAKE_CXX_FLAGS -std=c++0x)

set(CMAKE_BUILD_TYPE Debug)

find_package( Boost 1.42 COMPONENTS program_options )
include_directories( ${Boost_INCLUDE_DIRS} )
link_directories( ${Boost_LIBRARY_DIRS} )

include_directories (/usr/local/src/linuxcan_v4/include)
link_directories (/usr/local/src/linuxcan_v4/canlib)
include_directories (${CANOPENLib_SOURCE_DIR}/include)
link_directories (${CANOPENLib_BINARY_DIR}/canopen)

add_subdirectory(src)

add_subdirectory(test)
