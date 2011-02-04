/*
	Some library functions
*/

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

#include "antix.pb.h"

using namespace std;

class antix {
public:
	/*
		Take a host and a port, return c_str
		memleak?
	*/
	static const char *
	make_endpoint(string host, string port) {
		string s = "tcp://" + host + ":" + port;
		return s.c_str();
	}

	/*
		Send the protobuf message pb_obj on socket
	*/
	static void
	send_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj) {
		string pb_as_str;
		pb_obj->SerializeToString(&pb_as_str);

		zmq::message_t msg( pb_as_str.size() );
		memcpy( msg.data(), pb_as_str.c_str(), pb_as_str.size() );

		socket->send(msg);
	}

	/*
		Receive a waiting protobuf message on socket, parse into pb_obj
	*/
	static void
	recv_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj) {
		zmq::message_t msg;
		socket->recv(&msg);

		char raw_pb[msg.size() + 1];
		memcpy(raw_pb, msg.data(), msg.size());
		pb_obj->ParseFromString(raw_pb);
	}
};
