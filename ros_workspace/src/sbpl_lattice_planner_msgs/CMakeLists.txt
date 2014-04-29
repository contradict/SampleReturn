cmake_minimum_required(VERSION 2.8.3)
project(sbpl_lattice_planner_msgs)

##############################################################################
# Find dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

##############################################################################
# Generate messages
##############################################################################

add_message_files(FILES SBPLLatticePlannerStats.msg)
generate_messages(DEPENDENCIES geometry_msgs)

##############################################################################
# Define package
##############################################################################

# Since sbpl is found through pkg-config, we need to manually add its
# libraries and include directories into catkin_package
catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime
)

##############################################################################
# Build
##############################################################################

include_directories(
	${catkin_INCLUDE_DIRS}
)
