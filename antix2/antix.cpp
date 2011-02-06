/*
	Some library functions & object defs

	Most of the objects are essentially copied from rtv's Antix

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
		Radians to degrees
		From rtv's Antix
	*/
	double
	rtod(double r) {
		return (r * 180.0 / M_PI);
	}
	/*
		Degrees to radians
		From rtv's Antix
	*/
	double
	dtor(double d) {
		return (d * M_PI / 180.0);
	}

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

		zmq::message_t msg( pb_as_str.size() + 1 );
		memcpy( msg.data(), pb_as_str.c_str(), pb_as_str.size() + 1);

		socket->send(msg);
	}

	/*
		Receive a waiting protobuf message on socket, parse into pb_obj

		Parsing from bytes -> string from
		http://www.mail-archive.com/protobuf@googlegroups.com/msg05381.html
	*/
	static void
	recv_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj, int flags) {
		zmq::message_t msg;
		socket->recv(&msg, flags);

		char raw_pb[msg.size()];
		memcpy(raw_pb, msg.data(), msg.size());

		// make a string out of the raw bytes
		string s;
		s.assign(raw_pb, msg.size() + 1);

		pb_obj->ParseFromString(s);
	}

	/*
		Copy the data from src to dest
	*/
	static void
	copy_node(antixtransfer::Node_list::Node *dest, antixtransfer::Node_list::Node *src) {
		dest->set_id( src->id() );
		dest->set_ip_addr( src->ip_addr() );
		dest->set_neighbour_port( src->neighbour_port() );
		dest->set_client_port( src->client_port() );
		dest->set_x_offset( src->x_offset() );
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
				copy_node(left, node_list->mutable_node( index_left ) );
				copy_node(right, node_list->mutable_node( index_right ) );
				return;
			}
		}
	}

	static void
	print_nodes(antixtransfer::Node_list *node_list) {
		const antixtransfer::Node_list::Node *node;
		for (int i = 0; i < node_list->node_size(); i++) {
			node = node_list->mutable_node(i);
			cout << "\tNode id: " << node->id();
			cout << " IP: " << node->ip_addr();
			cout << " Client Port: " << node->client_port();
			cout << " Neighbour Port: " << node->neighbour_port();
			cout << " x offset: " << node->x_offset() << endl;
		}
	}

	static double
	rand_between(double min, double max) {
		return ( (drand48() * (max - min) ) + min );
	}
};

/*
	from rtv's Antix
*/
class Colour {
	public:
	double r, g, b;

	Colour() {
		r = drand48();
		g = drand48();
		b = drand48();
	}
	Colour (double r, double g, double b) : r(r), g(g), b(b) { }
};

class Home {
public:
	double x, y;
	double r;
	Colour colour;

	Home(double x, double y, double r) : x(x), y(y), r(r) {
		colour = Colour();
	}

	Home(double x, double y, double r, Colour colour) : x(x), y(y), r(r), colour(colour) { }

	Home(double r, double world_size) : r(r) {
		x = antix::rand_between(0, world_size);
		y = antix::rand_between(0, world_size);
		colour = Colour;
	}
};

class Puck {
public:
	double x,
		y;
	bool held;

	// random pose stuff is from rtv's Antix
	Puck(double min_x, double max_x, double world_size) {
		x = antix::rand_between(min_x, max_x);
		y = antix::rand_between(0, world_size);
		held = false;
	}
	Puck(double x, double y, bool held) : x(x), y(y), held(held) {}
};

class Robot {
public:
	double x;
	double y;
	// orientation
	double a;
	// forward speed
	double v;
	// turn speed
	double w;

	int team;
	bool has_puck;

	Robot(double x, double y, int team) : x(x), y(y), team(team) {
		a = 0;
		v = 0;
		w = 0;
	}
};
