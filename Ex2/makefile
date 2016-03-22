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

test: canny hough
	$(CC) $(GG) -o test test.cpp hough.o canny.o gaussian.o

filter: hough CImg.h filter.cpp
	$(CC) $(GG) -o filter.out filter.cpp hough.o

canny: CImg.h canny.h canny.cpp gaussian hough
	$(CC) $(GG) -c canny.cpp 

gaussian: gaussian.h gaussian.cpp CImg.h
	$(CC) $(GG) -c gaussian.cpp

hough: hough.h hough.cpp CImg.h
	$(CC) $(GG) -c hough.cpp

clean:
	rm *.o *.out