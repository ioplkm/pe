CC = clang
CFLAGS = -Wall -Wextra -Wpedantic -pipe -O3 -std=c99 -s -flto=thin -shared -fPIC
LDFLAGS = -lm
all: src/*.c
	$(CC) $(CFLAGS) $(LDFLAGS) src/*.c -o libpe.so
install:
	mkdir -p /usr/local/include/pe
	install -m755 libpe.so /usr/local/lib64
	install -m755 inc/*.h /usr/local/include
