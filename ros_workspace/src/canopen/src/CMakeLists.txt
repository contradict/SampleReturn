add_library(canopen Message.cpp
                    Transfer.cpp
                    Interface.cpp KvaserInterface.cpp
                    NMT.cpp EMCY.cpp SDO.cpp PDO.cpp
                    Bus.cpp
                    DS301.cpp
                    CopleyServo.cpp)
install(
    TARGETS
      canopen
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
