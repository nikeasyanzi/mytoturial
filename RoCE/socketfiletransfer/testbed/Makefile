.PHONY: clean

CFLAGS  := -Werror -g -DDEBUG
LD      := gcc
LDLIBS  := ${LDLIBS}

APPS    := client server

all: clean ${APPS}

client: client.o
	${LD} -o $@ $^ ${LDLIBS}

server: server.o
	${LD} -o $@ $^ ${LDLIBS}

clean:
	rm -f *.o ${APPS}

