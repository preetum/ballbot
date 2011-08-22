CC = g++
CFLAGS = -I/usr/local/include/opencv
CFLAGS += -g -Wall -O3
LDFLAGS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lm
SOURCES := $(wildcard *.cpp)
OBJECTS := $(SOURCES:.cpp=.o)
EXECUTABLE = main

all: $(SOURCES) $(EXECUTABLE)

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) -c $(CFLAGS) $< -o $@

%: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) $< -o $@
