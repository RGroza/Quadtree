CC = gcc
CFLAGS = -std=c99 -pedantic -g3

all: PointsetUnit

PointsetUnit: pointset_unit.o pointset.o point2d.o
	$(CC) $(CFLAGS) -o $@ -g $^ -lm

pointset_unit.o: pointset_unit.c pointset.h point2d.h
pointset.o: pointset.c pointset.h point2d.h
point2d.o: point2d.c point2d.h