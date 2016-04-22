CC=g++
CV=-I/usr/local/include/opencv2 -L/usr/local/lib  -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lopencv_hal
VL=-lvl

OS_NAME = $(shell uname -s)
LC_OS_NAME = $(shell echo $(OS_NAME) | tr '[A-Z]' '[a-z]')

ifeq ($(LC_OS_NAME), linux)
GG=-O2 -L/usr/X11R6/lib -lm -lpthread -lX11 -std=c++11
endif

ifeq ($(LC_OS_NAME), darwin)
GG=-O2 -lm -lpthread -I/usr/X11R6/include -L/usr/X11R6/lib -lm -lpthread -lX11 -std=c++11
endif

all: test

test: test.cpp image_stitch.h
	$(CC) $(GG) $(CV) $(VL) -o test test.cpp

# image_stitch: image_stitch.cpp
# 	$(CC) $(GG) -c image_stitch.cpp


clean:
	rm *.o test