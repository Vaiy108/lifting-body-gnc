# Dependency-free build: only libc and libm. No external libraries.
CC ?= gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lm

SRC = src/quat_math.c src/matlib.c src/eskf.c src/control.c
OBJ = $(SRC:src/%.c=build/%.o)

.PHONY: all test clean

all: build/test_main

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/test_main: test/test_main.c $(OBJ) | build
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

build:
	mkdir -p build

test: build/test_main
	./build/test_main test_vectors

clean:
	rm -rf build
