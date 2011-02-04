# You must set the environment variables
#
# export ZMQ_PATH=/pathto/zeromq
# export PROTOBUF_PATH=/path/to/protobuf
#
# e.g.
# export ZMQ_PATH=/home/wjs2/zeromq
# export PROTOBUF_PATH=/home/wjs2/protobuf
#

targets=master operator node client
objs=antix.pb.o

includes=-I$(ZMQ_PATH)/include -I$(PROTOBUF_PATH)/include
lib_paths=-L$(ZMQ_PATH)/lib -L$(PROTOBUF_PATH)/lib
libraries=-lzmq -lprotobuf

all: $(targets)

.cpp: $(objs)
	g++ -o $@ $< $(objs) $(includes) $(lib_paths) $(libraries)

.cc:
	g++ -c $< $(includes) $(lib_paths) $(libraries)

clean:
	rm -f $(targets)
