SRCS = main.cpp
OUTPUT = main
LIBS = ../blobs/libblob.a

CC = g++
CFLAGS = -c -g -Wall -I/usr/local/include/opencv
CFLAGS += -I../blobs
LDFLAGS = -L/usr/local/lib -lcv -lhighgui

OBJS = $(SRCS:.cpp=.o)

.PHONY: all clean

all: $(SRCS) $(OUTPUT)

$(OUTPUT): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $(OUTPUT)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(OUTPUT) $(OBJS) *~
