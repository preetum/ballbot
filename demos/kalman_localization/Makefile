CFLAGS = -I/usr/local/include/opencv  -L/usr/local/lib -lcv -lhighgui 
#CFLAGS += -I/home/ballbot/cvblobslib
CFLAGS += -g -Wall

SRCS = main.cpp
LIBS = #/home/ballbot/cvblobslib/libblob.a

all:
	g++ $(CFLAGS) $(SRCS) $(LIBS) -o main

clean:
	rm -f main *~
