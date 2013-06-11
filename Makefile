# iRobotInterface Makefile
# Author: BB
# Date  : 05-23-2013
#
# Changelog :
#   2.1 - Version 2
# ------------------------------------------------

# Set compiler options
CC = gcc 
CFLAGS = -g -std=c99 -Wall -I. 

# Set linker options
LINKER   = gcc
LFLAGS   = -g -std=c99 -Wall -I.
LIBS = -lpthread -lm 


calibrate.out: calibrate.c
	gcc -o calibrate.out -Wall -std=c99 calibrate.c