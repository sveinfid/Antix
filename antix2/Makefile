
ZMQ_PATH=/home/$(USER)/zeromq
PROTOBUF_PATH=/home/$(USER)/protobuf

build_dir=.
targets=master operator node client
gui_targets=gui
objs=antix.pb.o

CFLAGS=-O3 -Wall
GLUTLIBS=-L/usr/X11R6/lib -lGLU -lGL -lglut -lX11 -lXext -lXmu -lXi
GLUTFLAGS=-I/usr/include/GL

includes=-I$(ZMQ_PATH)/include -I$(PROTOBUF_PATH)/include
lib_paths=-L$(ZMQ_PATH)/lib -L$(PROTOBUF_PATH)/lib
libraries=-lzmq -lprotobuf

all: $(objs) $(targets) $(gui_targets)

gui: gui.cpp
	g++ $(CFLAGS) -o $(build_dir)/$@ $< $(objs) $(includes) $(GLUTFLAGS) $(lib_paths) $(libraries) $(GLUTLIBS)

antix.pb.o: antix.proto
	$(PROTOBUF_PATH)/bin/protoc -I. --cpp_out=$(build_dir) antix.proto
	g++ $(CFLAGS) -c antix.pb.cc $(includes) $(lib_paths) $(libraries)

.cpp: master.cpp operator.cpp node.cpp client.cpp antix.pb.o antix.cpp entities.cpp map.cpp
	g++ $(CFLAGS) -o $(build_dir)/$@ $< $(objs) $(includes) $(lib_paths) $(libraries)

clean:
	rm -f $(targets) $(gui_targets) antix.pb.*
