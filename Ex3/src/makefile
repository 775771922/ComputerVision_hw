CC=g++

OS_NAME = $(shell uname -s)
LC_OS_NAME = $(shell echo $(OS_NAME) | tr '[A-Z]' '[a-z]')

ifeq ($(LC_OS_NAME), linux)
GG=-O2 -L/usr/X11R6/lib -lm -lpthread -lX11
endif

ifeq ($(LC_OS_NAME), darwin)
GG=-O2 -lm -lpthread -I/usr/X11R6/include -L/usr/X11R6/lib -lm -lpthread -lX11
endif

all: test

test: global canny hough rotate_op paper_corection
	$(CC) $(GG) -o test test.cpp hough.o canny.o gaussian.o paper_corection.o global.o RotateOP.o

paper_corection: CImg.h paper_corection.h paper_corection.cpp
	$(CC) $(GG) -c paper_corection.cpp

rotate_op: RotateOP.cpp
	$(CC) $(GG) -c RotateOP.cpp 

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