.PHONY: clean

CFLAGS  := -Werror -g -DDEBUG
LD      := gcc
LDLIBS  := ${LDLIBS} -lrdmacm -libverbs -lpthread

APPS    := client server

all: ${APPS}

client: client.o
	${LD} -o $@ $^ ${LDLIBS}

server: server.o
	${LD} -o $@ $^ ${LDLIBS}

clean:
	rm -f *.o ${APPS}

