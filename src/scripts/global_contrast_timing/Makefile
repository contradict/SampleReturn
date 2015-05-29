CUDA_INSTALL_PATH = /usr/local/cuda
CFLAGS = -I$(CUDA_INSTALL_PATH)/include `pkg-config --cflags opencv`
LDFLAGS = `pkg-config --libs opencv` -L$(CUDA_INSTALL_PATH)/lib64 -lcudart

all:
	 g++ -std=gnu++11 -O1 -g -c $(CFLAGS) histogram_contrast.cpp -o histogram_contrast.o
	 g++ histogram_contrast.o $(LDFLAGS) -o histogram_contrast
