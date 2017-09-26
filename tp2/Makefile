CC=c99
MACCC=gcc
CFLAGS= -Wall -Wextra -pedantic -O0 -g -lm -Wno-unused-variable -Wno-unused-parameter
MACFLAGS = -g -lm  -Wno-deprecated
LIBS = -lGL -lGLU -lglut -lpthread -lm
MACLIBS = -framework OpenGL -framework GLUT
NASM=nasm
NASMFLAGS=-f elf64 -g -F DWARF

all: demo test

test: test.c solver_asm.o solver_c.o
	$(CC) $(CFLAGS) $^ -o $@  $(LIBS)

demo: demo.c solver_c.o solver_asm.o bmp.o
	$(CC) $(CFLAGS) $^ -o $@  $(LIBS)

solver_c.o: solver.c
	$(CC) $(CFLAGS) -c $< -o $@

solver_asm.o: solver.asm
	$(NASM) $(NASMFLAGS) $< -o $@

bmp.o: bmp/bmp.c bmp/bmp.h
	$(CC) $(CFLAGS) -c $< -o $@

mac: demo_mac

demo_mac: demo.c solver_mac.o solver_asm.o bmp_mac.o
	$(MACCC) $(MACCFLAGS) $^ -o $@  $(MACLIBS)

solver_mac.o: solver.c
	$(MACCC) $(MACFLAGS) -c $< -o $@

bmp_mac.o: bmp/bmp.c bmp/bmp.h
	$(MACCC) $(MACFLAGS) -c $< -o $@

clean:
	rm -f *.o
	rm -f demo demo_mac