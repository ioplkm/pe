CC = clang
CFLAGS = -Wall -Wextra -Wpedantic -pipe -O3 -std=c99 -s -flto=thin -shared -fPIC
LDLIBS = -lm
PREFIX = /usr/local
all: src/*
	$(CC) $(CFLAGS) $(LDLIBS) src/* -o libpe.so
install:
	mkdir -p $(PREFIX)/include/pe
	install -m755 libpe.so $(PREFIX)/lib64
	install -m755 inc/* $(PREFIX)/include/pe
uninstall:
	rm -rf $(PREFIX)/include/pe
	rm -f $(PREFIX)/lib64/libpe.so
clean:
	rm -f libpe.so
