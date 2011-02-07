# You must set the environment variables
#
# export ZMQ_PATH=/pathto/zeromq
# export PROTOBUF_PATH=/path/to/protobuf
#
# e.g.


export ZMQ_PATH=/home/gpa7/sfuhome/zeromq
export PROTOBUF_PATH=/home/gpa7/sfuhome/protobuf

build_dir=.
targets=master operator node client master-gptest
objs=antix.pb.o

includes=-I$(ZMQ_PATH)/include -I$(PROTOBUF_PATH)/include
lib_paths=-L$(ZMQ_PATH)/lib -L$(PROTOBUF_PATH)/lib
libraries=-lzmq -lprotobuf

all: $(objs) $(targets)

.cpp: $(objs)
	g++ -o $(build_dir)/$@ $< $(objs) $(includes) $(lib_paths) $(libraries)

antix.pb.o: antix.proto
	$(PROTOBUF_PATH)/bin/protoc -I. --cpp_out=$(build_dir) antix.proto
	g++ -c antix.pb.cc $(includes) $(lib_paths) $(libraries)

clean:
	rm -f $(targets) antix.pb.*
