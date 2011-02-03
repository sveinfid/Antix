zmq_path=/home/wjs2/zeromq
targets=master operator node

all: $(targets)

master: master.cpp
	g++ -o master master.cpp -I$(zmq_path)/include -L$(zmq_path)/lib -lzmq

operator: operator.cpp
	g++ -o operator operator.cpp -I$(zmq_path)/include -L$(zmq_path)/lib -lzmq

node: node.cpp
	g++ -o node node.cpp -I$(zmq_path)/include -L$(zmq_path)/lib -lzmq
