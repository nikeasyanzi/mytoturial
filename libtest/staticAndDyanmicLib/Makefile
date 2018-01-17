
LIBSRC = libmyadd
APP = main
all: static shared dynamic
static:
	gcc -c -fPIC libmyadd.c -o libmyadd.o
	ar crv libmyadd-static.a libmyadd.o
	gcc -g -static -DSTATIC ${APP}.c  libmyadd-static.a -o ${APP}static # keep lib prefix

shared:
	gcc -c -fPIC -o libmyadd.o libmyadd.c
	gcc -s -shared -o libmyadd.so libmyadd.o
	gcc ${APP}.c -DEXPLICITDYNAMIC -L. -lmyadd -o ${APP}-explicit-dynamic -ldl

dynamic:
	gcc -c -fPIC -o libmyadd.o libmyadd.c
	gcc -s -shared -o libmyadd.so libmyadd.o
	gcc ${APP}.c -DIMPLICITDYNAMIC -I. -L. -lmyadd -o ${APP}-implicit-dynamic

clean:
	rm -f ${APP}static ${APP}-explicit-dynamic ${APP}-implicit-dynamic *.o *.a *.so

