.PHONY: bms_timing_test

LIB=-L/home/zlizer/src/SampleReturn/ros_workspace/devel/lib -L/usr/local/opencv3/lib
INC=-I/usr/local/opencv3/include -I/home/zlizer/src/SampleReturn/ros_workspace/src/saliency_detector/include

bms_timing_test: bms_timing_test.cpp
	g++ -std=c++11 -O3 -g bms_timing_test.cpp $(INC) $(LIB) -lBMS -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -o bms_timing_test
