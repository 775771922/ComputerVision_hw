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

test: triangle triangulation
	$(CC) $(GG) -o test test.cpp Triangle.o Triangulation.o

triangulation: Triangulation.cpp
	$(CC) $(GG) -c Triangulation.cpp

triangle: Triangle.cpp
	$(CC) $(GG) -c Triangle.cpp

clean:
	rm *.o *.out