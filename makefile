CC=gcc

CFLAGS=-g -O2 -pthread  -DHAVE_PTHREAD=1 -DHAVE_LIBM=1 -DHAVE_LIBNSL=1 -DSTDC_HEADERS=1 -DHAVE_FCNTL_H=1 -DHAVE_STRINGS_H=1 -DHAVE_SYS_TIME_H=1 -DHAVE_UNISTD_H=1 -DTIME_WITH_SYS_TIME=1 -DSIZEOF_INT=4 -DSIZEOF_LONG=4 -DHAVE_STRFTIME=1 -DHAVE_GETHOSTNAME=1 -DHAVE_GETTIMEOFDAY=1 -DHAVE_SOCKET=1  -DTHRLIB
CPPFLAGS=
LIBS=`libnet/libnet-config --libs` -lpcap -lnsl -lm  
LDFLAGS=
DEFINES = `libnet/libnet-config --defines`  -DHAVE_PTHREAD=1 -DHAVE_LIBM=1 -DHAVE_LIBNSL=1 -DSTDC_HEADERS=1 -DHAVE_FCNTL_H=1 -DHAVE_STRINGS_H=1 -DHAVE_SYS_TIME_H=1 -DHAVE_UNISTD_H=1 -DTIME_WITH_SYS_TIME=1 -DSIZEOF_INT=4 -DSIZEOF_LONG=4 -DHAVE_STRFTIME=1 -DHAVE_GETHOSTNAME=1 -DHAVE_GETTIMEOFDAY=1 -DHAVE_SOCKET=1 
LIBDIR = -Llibnet -L/usr/lib

OBJS=qr.o robustfit.o nrutil.o fabprobe_utils.o fabprobe_snd.o fabprobe_snd_func.o

TARGETS=fabprobe_snd

all:${TARGETS} clean

fabprobe_snd: $(OBJS)
	 $(CC) $(LIBDIR) $(DEFINES) $(OBJS) -o fabprobe_snd $(LIBS) $(LDFLAGS) $(CFLAGS)

fabprobe_snd.o fabprobe_snd_func.o: fabprobe_gbls.h fabprobe_snd.h

clean: 
	 rm -f ${OBJS} config.cache config.log  config.status

distclean: 
	 rm -f ${OBJS} ${TARGETS} config.cache config.log makefile config.status
