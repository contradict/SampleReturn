add_executable(invmap invmaptest.cpp)

add_executable(readbus readbus.cpp)
target_link_libraries(readbus
    ${PROJECT_NAME}
    canlib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

#add_executable(VelocityModeTest VelocityModeTest.cpp)
#target_link_libraries(VelocityModeTest
#    ${PROJECT_NAME}
#    canlib
#    ${Boost_LIBRARIES}
#    ${catkin_LIBRARIES}
#    )

add_executable(Transfer TransferTest.cpp)
target_link_libraries(Transfer
    ${PROJECT_NAME}
    canlib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )


add_executable(Copley CopleyTest.cpp)
target_link_libraries(Copley
    ${PROJECT_NAME}
    canlib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
