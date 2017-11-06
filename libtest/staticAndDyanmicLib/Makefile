
LIBSRC = libmyadd
APP = main
all: static explicit-dynamic implicit-dynamic
static:
	gcc -c -fPIC libmyadd.c -o libmyadd.o
	ar crv libmyadd-static.a libmyadd.o
	gcc -g -static -DSTATIC ${APP}.c  libmyadd-static.a -o ${APP}static # keep lib prefix
	#gcc main.c -L -DSTATIC ./ -lmyadd-static -o main

explicit-dynamic:
	gcc -c -fPIC -o libmyadd.o libmyadd.c
	gcc -s -shared -o libmyadd.so libmyadd.o
	gcc ${APP}.c -DEXPLICITDYNAMIC -L. -lmyadd -o ${APP}-explicit-dynamic -ldl


implicit-dynamic:
	gcc -c -fPIC -o libmyadd.o libmyadd.c
	gcc -s -shared -o libmyadd.so libmyadd.o
	#gcc ${APP}.c -DIMPLICITDYNAMIC -Wl,-rpath-link=. -Wl,-rpath,'$$ORIGIN:${ORIGIN}/../lib' -I. -L. -lmyadd -o ${APP}-implicit-dynamic
	gcc ${APP}.c -DIMPLICITDYNAMIC -I. -L. -lmyadd -o ${APP}-implicit-dynamic
	
