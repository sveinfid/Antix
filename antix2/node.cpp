/*
	Node / map piece server
*/

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_node_port = "7770";
string master_publish_port = "7773";

int main() {
	zmq::context_t context(1);

	zmq::socket_t node_master_sock(context, ZMQ_REQ);
	node_master_sock.connect(antix::make_endpoint(master_host, master_node_port));
	zmq::socket_t master_publish_sock(context, ZMQ_SUB);


	cout << "Sending master existence notification";
	// XXX get our own ip somehow
	// send message including our own IP

	// receive message back stating our unique ID

	// block on master_publish_sock
}
