#   Makefile - antix project
#   version 3
#   Richard Vaughan  

# this should work on Linux with MESA
#GLUTLIBS = -L/usr/X11R6/lib -lGLU -lGL -lglut -lX11 -lXext -lXmu -lXi
#GLUTFLAGS = -I/usr/local/include/GL
GLUTFLAGS= -usr/include/GL 
GLUTLIBS= -usr/local/lib -lGLU -lGL -lglut 
# this works on Mac OS X
#GLUTFLAGS = -framework OpenGL -framework GLUT -framework Cocoa

CC = g++
CXXFLAGS = -g -Wall -O3 $(GLUTFLAGS)
LIBS =  -g -lm $(GLUTLIBS) 

SRC = main.c database.h database.cc

all: antix

antix: $(SRC)
	$(CC) $(CXXFLAGS) libhiredis.so $(LIBS) -o $@ $(SRC) 

clean:
	rm *.o antix

