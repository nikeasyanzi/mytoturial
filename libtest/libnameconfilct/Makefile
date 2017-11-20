CC = gcc  
CFLAGS = -fPIC -Wall
DEBUG = -g
INC = -I ./
LIB = -L ./ 

OBJS=foo bar

#-----------------------------------------------------------------------------
# clear suffix list and set new one
.SUFFIXES:
.SUFFIXES: .c .o .cpp .h
#  
#-----------------------------------------------------------------------------


main: foo.a bar.a                                    
	${CC} -static $@.c $^ ${CFLAGS} ${INC} ${LIB} -o $@

%.a: %.o 
	ar crv $*.a $*.o

%.o: %.c %.h                      
	${CC} $*.c ${CFLAGS} ${INC} ${LIB} -c -o $*.o         

.PHONY: clean

clean:                             
		@rm -rf *.o *.a 

