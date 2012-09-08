cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# actions
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories (/usr/local/src/linuxcan_v4/include)
link_directories (/usr/local/src/linuxcan_v4/canlib)
include_directories (${PROJECT_SOURCE_DIR}/../../CANOpen/include)
link_directories (${PROJECT_SOURCE_DIR}/../../CANOpen/build/src/)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
# dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/motion.cpp src/wheelpod.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(motion_node src/motion_node.cpp)
target_link_libraries(motion_node ${PROJECT_NAME})
target_link_libraries(motion_node canopen)
target_link_libraries(motion_node canlib)

rosbuild_add_executable(teleop src/teleop.cpp)

rosbuild_add_executable(odometry src/odometry.cpp src/lmmin.c)
