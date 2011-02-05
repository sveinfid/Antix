/*
	Some library functions

	send_pb/recv_pb motivated by s_send/s_recv from
	https://github.com/imatix/zguide/blob/master/examples/C%2B%2B/zhelpers.hpp
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
	*/
	static const char *
	make_endpoint(string host, string port) {
		string s = "tcp://" + host + ":" + port;
		// invalid memory when function returns?
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

	/*
		Go through given Node_list looking for the node with id, and set the
		neighbour pointers
	*/
	static void
	set_neighbours(antixtransfer::Node_list::Node *left,
		antixtransfer::Node_list::Node *right,
		antixtransfer::Node_list *node_list,
		int id)
	{
		antixtransfer::Node_list::Node n;
		for (int i = 0; i < node_list->node_size(); i++) {
			n = node_list->node(i);
			int index_left,
				index_right;
			// found ourself, take the previous as left, next as right
			if (n.id() == id) {
				// if we're far left node, our left is the furthest right
				if (id == 0) {
					index_left = node_list->node_size() - 1;
					index_right = i + 1;

				// if we're far right node, our right is furthest left
				} else if (id == node_list->node_size() - 1) {
					index_left = i - 1;
					index_right = 0;
				} else {
					index_left = i - 1;
					index_right = i + 1;
				}

				left->set_id( node_list->node( index_left ).id() );
				left->set_ip_addr ( node_list->node( index_left ).ip_addr() );

				right->set_id( node_list->node( index_right ).id() );
				right->set_ip_addr ( node_list->node( index_right ).ip_addr() );
				return;
			}
		}
	}
};
