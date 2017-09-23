CC=c99
MACCC=gcc
CFLAGS= -Wall -Wextra -pedantic -O0 -g -lm -Wno-unused-variable -Wno-unused-parameter
MACFLAGS = -g -lm  -Wno-deprecated
LIBS = -lGL -lGLU -lglut -lpthread -lm
MACLIBS = -framework OpenGL -framework GLUT

all: demo

demo: demo.c solver.o bmp.o
	$(CC) $(CFLAGS) $^ -o $@  $(LIBS)

solver.o: solver.c
	$(CC) $(CFLAGS) -c $< -o $@

bmp.o: bmp/bmp.c bmp/bmp.h
	$(CC) $(CFLAGS) -c $< -o $@

mac: demo_mac

demo_mac: demo.c solver_mac.o bmp_mac.o
	$(MACCC) $(MACCFLAGS) $^ -o $@  $(MACLIBS)

solver_mac.o: solver.c
	$(MACCC) $(MACFLAGS) -c $< -o $@

bmp_mac.o: bmp/bmp.c bmp/bmp.h
	$(MACCC) $(MACFLAGS) -c $< -o $@

clean:
	rm -f *.o
	rm -f demo demo_mac

