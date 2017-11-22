.PHONY: clean

CFLAGS  := -Werror
LD      := gcc
LDLIBS  := ${LDLIBS} -lrdmacm -libverbs -lpthread

APPS    := client server

all: ${APPS}

debug: CFLAGS  := -Werror -g -DDEBUG
debug: ${APPS}

client: client.o
	${LD} -o $@ $^ ${LDLIBS}

server: server.o
	${LD} -o $@ $^ ${LDLIBS}

clean:
	rm -f *.o ${APPS}

