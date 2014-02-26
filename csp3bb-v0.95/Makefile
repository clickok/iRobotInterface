# CSP3 Makefile
#
# Author: BB
# Date  : 06-18-2013
#
# Changelog :
#   1.0 - Version 1
# ------------------------------------------------

# Set compiler options
CC := gcc
CFLAGS := -Wall -I.

# Set libraries to use
LIBS = -lpthread

# Determine sources
SRCS = $(wildcard *.c)

EXECS := $(SRCS:.c=.out)

all: $(EXECS)

%.out: %.c
	$(CC) -o $@ $^ $(LIBS) $(CFLAGS)

# Utilities for working with other computers
clean:
	rm -f $(EXECS)

updateAll:
	make clean
	git pull
	make all
