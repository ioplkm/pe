CC = clang
CFLAGS = -Wall -Wextra -Wpedantic -pipe -O3 -std=c99 -s -flto=thin
LDFLAGS = -lm
all: main.c
	$(CC) $(CFLAGS) $(LDFLAGS) main.c -o main
