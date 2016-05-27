CC=g++
#CV=-I/usr/local/include/opencv2 -L/usr/local/lib -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lopencv_hal
CV= `pkg-config --cflags --libs opencv`
GG=-std=c++11

OS_NAME = $(shell uname -s)
LC_OS_NAME = $(shell echo $(OS_NAME) | tr '[A-Z]' '[a-z]')

all: test

test: svm.cpp
	$(CC) $(GG) $(CV) -o svm svm.cpp

# image_stitch: image_stitch.cpp
# 	$(CC) $(GG) -c image_stitch.cpp


clean:
	rm *.o test