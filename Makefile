CC = gcc
CFLAGS += -I. -I/usr/include/libusb-1.0/ -Wall -O2 -fno-math-errno -fno-trapping-math -fno-omit-frame-pointer -fno-asynchronous-unwind-tables -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 
INCLUDES = -I. -I/usr/include/libusb-1.0/
DEST_BASE=/usr
DEST_INC=${DEST_BASE}/include
DEST_LIB=${DEST_BASE}/lib
DEST_BIN=${DEST_BASE}/bin

all: rtl-ais

build: all

.o: %.c
	$(CC) $(INCLUDES) $(CFLAGS) -c $<

rtl-ais: rtl-ais.o msk.o protodec.o librtlsdr.o tuner_e4k.o tuner_r82xx.o
	$(CC) -o rtl-ais rtl-ais.o msk.o protodec.o librtlsdr.o tuner_e4k.o tuner_r82xx.o -lusb-1.0

clean:
	rm -f rtl-ais
	rm -f *.o

