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

test: ScaleOP RotateOP DrawOP test.cpp
	$(CC) $(GG) -o test ScaleOP.o RotateOP.o DrawOP.o test.cpp

ScaleOP: CImg.h ScaleOP.h ScaleOP.cpp
	$(CC) $(GG) -c ScaleOP.cpp

RotateOP: CImg.h RotateOP.h RotateOP.cpp
	$(CC) $(GG) -c RotateOP.cpp

DrawOP: CImg.h DrawOP.h DrawOP.cpp
	$(CC) $(GG) -c DrawOP.cpp

clean:
	rm *.o