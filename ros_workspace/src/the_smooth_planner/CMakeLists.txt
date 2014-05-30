cmake_minimum_required(VERSION 2.8.3)
project(the_smooth_planner)

##############################################################################
# Find dependencies
##############################################################################


find_package(catkin REQUIRED COMPONENTS
                costmap_2d
                nav_core
                pluginlib 
                geometry_msgs
                nav_msgs
                ecl_geometry
                platform_motion_msgs
                visualization_msgs
                tf)

# Find SBPL
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(SBPL REQUIRED sbpl)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##############################################################################
# Generate messages
##############################################################################

##############################################################################
# Define package
##############################################################################

# Since TheSmooth is found through pkg-config, we need to manually add its
# libraries and include directories into catkin_package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

##############################################################################
# Build
##############################################################################

add_library(${PROJECT_NAME} src/the_smooth_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${platform_motion_msgs_EXPORTED_TARGETS})

##############################################################################
# Install
##############################################################################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
