CC=g++
GG=-O2 -I/usr/X11R6/include -std=c++11

OS_NAME = $(shell uname -s)
LC_OS_NAME = $(shell echo $(OS_NAME) | tr '[A-Z]' '[a-z]')

CV= `pkg-config --cflags --libs opencv`

ifeq ($(LC_OS_NAME), linux)
CIMG=-O2 -L/usr/X11R6/lib -lm -lpthread -lX11 -std=c++11
endif

ifeq ($(LC_OS_NAME), darwin)
CIMG=-O2 -lm -lpthread -I/usr/X11R6/include -L/usr/X11R6/lib -lX11 -std=c++11
endif

all: test

test: global canny hough paper_corection
	$(CC) $(CIMG) $(CV) -o test test.cpp hough.o canny.o gaussian.o paper_corection.o global.o RotateOP.o

paper_corection: CImg.h paper_corection.h paper_corection.cpp
	$(CC) $(GG) -c paper_corection.cpp

canny: CImg.h canny.h canny.cpp gaussian hough
	$(CC) $(GG) -c canny.cpp 

gaussian: gaussian.h gaussian.cpp CImg.h
	$(CC) $(GG) -c gaussian.cpp

hough: hough.h hough.cpp CImg.h
	$(CC) $(GG) -c hough.cpp

global: global.cpp
	$(CC) $(GG) -c global.cpp	

clean:
	rm *.o *.out