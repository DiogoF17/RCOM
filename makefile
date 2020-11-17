CC = gcc

SRCS := $(wildcard *.c)
BINS := $(SRCS:%.c=%)

dep := 

all: clean receiver sender

clean: 
		rm -rvf *.o $(BINS)

receiver: 
		$(CC) -Wall noncanonical.c $(dep) -g -o receive

sender:
		$(CC) -Wall writenoncanonical.c $(dep) -g -o send