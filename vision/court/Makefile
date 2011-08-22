CC = g++
CFLAGS = -I/usr/local/include/opencv
CFLAGS += -g -Wall -O
LDFLAGS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lm
LDFLAGS += `pkg-config cvblob --libs`
SOURCES = findlines_model.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = findlines_model

all: $(SOURCES) $(EXECUTABLE)

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) -c $(CFLAGS) $< -o $@

%: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) $< -o $@
