CFLAGS = -I/usr/local/include/opencv  -L/usr/local/lib -lopencv_core
CFLAGS += -lopencv_highgui
CFLAGS += -g -Wall

SRCS = main.cpp
LIBS = #/home/ballbot/cvblobslib/libblob.a

all:
	g++ $(CFLAGS) $(SRCS) $(LIBS) -o main

clean:
	rm -f main *~
