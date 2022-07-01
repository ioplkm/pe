CC = clang
CFLAGS = -Wall -Wextra -Wpedantic -pipe -O3 -std=c99 -s -flto=thin -shared -fPIC
LDFLAGS = -lm 
all: src/*.c
	$(CC) $(CFLAGS) $(LDFLAGS) src/*.c -o libpe.so
install:
	install -m755 libpe.so /usr/local/lib64
	#install -m755 inc/* /usr/local/include
	install -m755 inc/matrix.h /usr/local/include
	install -m755 inc/quaternion.h /usr/local/include
	install -m755 inc/rigidbody.h /usr/local/include
	install -m755 inc/narrowcollision.h /usr/local/include
	install -m755 inc/vector.h /usr/local/include
