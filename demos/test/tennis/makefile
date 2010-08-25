CFLAGS=-Wall -g -O3 $(shell pkg-config --cflags opencv)
CFLAGS+=$(shell pkg-config --cflags opencv)
LDFLAGS=$(shell pkg-config --libs opencv)

tennis: tennis.c
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f tennis
