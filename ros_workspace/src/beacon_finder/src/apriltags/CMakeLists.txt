cmake_minimum_required(VERSION 2.8.3)
project(apriltags)

set(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function ${CMAKE_C_FLAGS}")

find_package (Threads REQUIRED)

#set this to the location of the unpacked tgz distribution
set(SRC_DIR apriltags-c)

include_directories(${SRC_DIR}/ ${SRC_DIR}/common)

## Declare a cpp library
set(APRILTAGS_C_SOURCES
    ${SRC_DIR}/apriltag.c
    ${SRC_DIR}/apriltag_quad_thresh.c
    ${SRC_DIR}/tag16h5.c
    ${SRC_DIR}/tag25h7.c
    ${SRC_DIR}/tag25h9.c
    ${SRC_DIR}/tag36h10.c
    ${SRC_DIR}/tag36h11.c
    ${SRC_DIR}/tag36artoolkit.c
    ${SRC_DIR}/g2d.c
    ${SRC_DIR}/common/zarray.c
    ${SRC_DIR}/common/zhash.c
    ${SRC_DIR}/common/zmaxheap.c
    ${SRC_DIR}/common/unionfind.c
    ${SRC_DIR}/common/matd.c
    ${SRC_DIR}/common/image_u8.c
    ${SRC_DIR}/common/pnm.c
    ${SRC_DIR}/common/image_f32.c
    ${SRC_DIR}/common/image_u32.c
    ${SRC_DIR}/common/workerpool.c
    ${SRC_DIR}/common/time_util.c
    ${SRC_DIR}/common/svd22.c
    ${SRC_DIR}/common/homography.c
    ${SRC_DIR}/common/string_util.c
    ${SRC_DIR}/common/getopt.c
)

add_library(apriltags
  ${APRILTAGS_C_SOURCES}
)
target_link_libraries(apriltags ${CMAKE_THREAD_LIBS_INIT} m)

add_executable(apriltag_demo ${SRC_DIR}/apriltag_demo.c)
target_link_libraries(apriltag_demo LINK_PUBLIC apriltags ${CMAKE_THREAD_LIBS_INIT} m)
