add_executable(wheel_visual_sensor main.cpp)

target_link_libraries(wheel_visual_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(wheel_visual_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
