#   Makefile - antix project
#   version 3
#   Richard Vaughan  

# this should work on Linux with MESA
#GLUTLIBS = -L/usr/X11R6/lib -lGLU -lGL -lglut -lX11 -lXext -lXmu -lXi
#GLUTFLAGS = -I/usr/local/include/GL

# this works on Mac OS X
#GLUTFLAGS = -framework OpenGL -framework GLUT

# this works on cygwin
GLUTFLAGS = -I/usr/include/opengl
EXTRALIBS = -lglut32 -lglu32 -lopengl32
EXTRAFLAGS = -DCYGWIN

CC = g++
CXXFLAGS = -g -Wall -O3 $(GLUTFLAGS)
LIBS =  -g -lm $(GLUTLIBS)

SRC = antix.h antix.cc controller.cc gui.cc main.cc 

all: antix

antix: $(SRC)
	$(CC) $(CXXFLAGS) $(LIBS) $(EXTRAFLAGS) -o $@ $(SRC) $(EXTRALIBS)

clean:
	rm *.o antix antix.exe

