add_executable(invmap invmaptest.cpp)

add_executable(readbus readbus.cpp)
target_link_libraries(readbus ${Boost_LIBRARIES})
target_link_libraries(readbus canlib)
target_link_libraries(readbus canopen)

#add_executable(VelocityModeTest VelocityModeTest.cpp)
#target_link_libraries(VelocityModeTest ${Boost_LIBRARIES})
#target_link_libraries(VelocityModeTest canlib)
#target_link_libraries(VelocityModeTest canopen)

add_executable(Transfer TransferTest.cpp)
target_link_libraries(Transfer ${Boost_LIBRARIES})
target_link_libraries(Transfer canlib)
target_link_libraries(Transfer canopen)

add_executable(Copley CopleyTest.cpp)
target_link_libraries(Copley ${Boost_LIBRARIES})
target_link_libraries(Copley canlib)
target_link_libraries(Copley canopen)
