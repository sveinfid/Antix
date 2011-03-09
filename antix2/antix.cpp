/*
	Some library functions & object defs

	Most of the objects are essentially copied from rtv's Antix

	send_pb/recv_pb motivated by s_send/s_recv from
	https://github.com/imatix/zguide/blob/master/examples/C%2B%2B/zhelpers.hpp
*/

#ifndef ANTIX_H
#define ANTIX_H

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <set>

#include "antix.pb.h"

// To disable asserts, define this
//#define NDEBUG
#include <assert.h>

#define SLEEP 0
#define GUI 1

#define DEBUG 0
#define DEBUG_SYNC 1

#if DEBUG
#define DEBUG_SYNC 1
#endif

// handy STL iterator macro pair. Use FOR_EACH(I,C){ } to get an iterator I to
// each item in a collection C.
// from rtv's Antix
#define VAR(V,init) __typeof(init) V=(init)
#define FOR_EACH(I,C) for(VAR(I,(C).begin());I!=(C).end();I++)

using namespace std;

class antix {
public:
	static double offset_size;
	static double world_size;
	static double my_min_x;
	static int turn;
	static double home_radius;
	
	static unsigned int matrix_width;
	static unsigned int matrix_height;
	static unsigned int matrix_right_x_col;
	static unsigned int matrix_right_world_col;

	/*
		Take a host and a port, return c_str
	*/
	static const char *
	make_endpoint(string host, string port) {
		string s = "tcp://" + host + ":" + port;
		// invalid memory when function returns?
		return s.c_str();
	}

	static const char *
	make_endpoint_ipc(string fname) {
		string s = "ipc://" + fname;
		return s.c_str();
	}

	/*
		sleep for the given milliseconds
		usleep() usage taken from rtv's Antix
	*/
	static void
	sleep(int ms) {
		usleep(ms * 1e3);
	}

	static void
	check_zmq_version() {
		int major, minor, patch;
		zmq_version(&major, &minor, &patch);
		if (major < 2 || minor < 1) {
			cerr << "Error: we require at least ZMQ 2.1.x." << endl;
			exit(-1);
		}
	}

	static void
	send_blank(zmq::socket_t *sock) {
		zmq::message_t blank(1);
		sock->send(blank);
	}

	static int
	recv_blank(zmq::socket_t *sock) {
		zmq::message_t blank(1);
		return sock->recv(&blank);
	}

	static int
	recv_blank(zmq::socket_t *sock, int flags) {
		zmq::message_t blank(1);
		return sock->recv(&blank, flags);
	}

	static void
	send_str(zmq::socket_t *sock, string s) {
		zmq::message_t msg(s.size() + 1);
		memcpy(msg.data(), s.c_str(), s.size() + 1);
		sock->send(msg);
	}

	static string
	recv_str(zmq::socket_t *sock) {
		zmq::message_t msg;
		sock->recv(&msg);
		return string( (char *) msg.data() );
	}

	/*
		Receive a string on the socket into s
		Respects flags, so we can do non blocking recv
		Returns whether successful (1 for success)
	*/
	static int
	recv_str(zmq::socket_t *sock, string *s, int flags) {
		zmq::message_t msg;
		int retval = sock->recv(&msg, flags);

		// If we did a non blocking call, it's possible we don't actually have a msg
		// But this return code doesn't match the ZMQ docs...
		if (retval != 1)
			return retval;

		*s = string( (char *) msg.data() );
		return retval;
	}

	static void
	send_blank_envelope(zmq::socket_t *sock, string address) {
		zmq::message_t type(address.size() + 1);
		memcpy(type.data(), address.c_str(), address.size() + 1);
		sock->send(type, ZMQ_SNDMORE);
		send_blank(sock);
	}

	static int
	send_pb_envelope(zmq::socket_t *sock, google::protobuf::Message *pb_obj, string address) {
		zmq::message_t type(address.size() + 1);
		memcpy(type.data(), address.c_str(), address.size() + 1);
		sock->send(type, ZMQ_SNDMORE);
		return send_pb(sock, pb_obj);
	}

	/*
		Send the protobuf message pb_obj on socket
	*/
	static int
	send_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj) {
		string pb_as_str;
		pb_obj->SerializeToString(&pb_as_str);

		zmq::message_t msg( pb_as_str.size() + 1 );
		memcpy( msg.data(), pb_as_str.c_str(), pb_as_str.size() + 1);

		return socket->send(msg);
	}

	/*
		Receive a waiting protobuf message on socket, parse into pb_obj

		NOTE: ParseFromString() clears the passed protobuf object.

		Parsing from bytes -> string from
		http://www.mail-archive.com/protobuf@googlegroups.com/msg05381.html
	*/
	static int
	recv_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj, int flags) {
		zmq::message_t msg;
		int retval = socket->recv(&msg, flags);

		// If we did a non blocking call, it's possible we don't actually have a msg
		// But this return code doesn't match the ZMQ docs...
		if (retval != 1)
			return retval;

		char raw_pb[msg.size()];
		memcpy(raw_pb, msg.data(), msg.size());

		// make a string out of the raw bytes
		string s;
		s.assign(raw_pb, msg.size() + 1);

		pb_obj->ParseFromString(s);
		return retval;
	}

	/*
		Send req_sock a message stating we're done a turn
		Then wait until sub_sock contacts us (so that all nodes/clients are in sync)
	*/
	static string
	wait_for_next_turn(zmq::socket_t *req_sock,
		zmq::socket_t *sub_sock,
		int id,
		antixtransfer::done::Type type) {

		antixtransfer::done done_msg;
		done_msg.set_my_id( id );
		done_msg.set_type( type );
		int ret = send_pb_envelope(req_sock, &done_msg, "done");
		assert(ret == 1);
#if DEBUG
		cout << "Sync: Sent done signal" << endl;
#endif
		// necessary response due to REQ socket
		recv_blank(req_sock);

#if DEBUG
		cout << "Sync: Got rep from done send. Waiting for begin signal..." << endl;
#endif

		// now we block on PUB sock awaiting begin
		string s = recv_str(sub_sock);
#if DEBUG
		cout << "Sync: Received awaited signal" << endl;
#endif
		return s;
	}

	/*
		Copy the data from src to dest
	*/
	static void
	copy_node(antixtransfer::Node_list::Node *dest, antixtransfer::Node_list::Node *src) {
		dest->set_id( src->id() );
		dest->set_ip_addr( src->ip_addr() );
		dest->set_neighbour_port( src->neighbour_port() );
		dest->set_gui_port( src->gui_port() );
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
		cout << "Left neighbour id: " << left->id() << " " << left->ip_addr() << " neighbour port " << left->neighbour_port() << endl;
		cout << "Right neighbour id: " << right->id() << " " << right->ip_addr() << " neighbour port " << right->neighbour_port() << endl;
	}

	static void
	print_nodes(antixtransfer::Node_list *node_list) {
		const antixtransfer::Node_list::Node *node;
		for (int i = 0; i < node_list->node_size(); i++) {
			node = node_list->mutable_node(i);
			cout << "\tNode id: " << node->id();
			cout << " IP: " << node->ip_addr();
			cout << " Neighbour port: " << node->neighbour_port();
			cout << " GUI port: " << node->gui_port();
			cout << " x offset: " << node->x_offset() << endl;
		}
	}

	/*
		Random double between the two given doubles
	*/
	static double
	rand_between(double min, double max) {
		return ( (drand48() * (max - min) ) + min );
	}

	/*
		Radians to degrees
		From rtv's Antix
	*/
	static double
	rtod(double r) {
		return (r * 180.0 / M_PI);
	}

	/*
		Degrees to radians
		From rtv's Antix
	*/
	static double
	dtor(double d) {
		return (d * M_PI / 180.0);
	}

	/*
		wrap around torus
		from rtv's Antix
	*/
	static double
	WrapDistance(double d) {
		const double halfworld( world_size * 0.5 );

		if( d > halfworld )
		d -= world_size;
		else if( d < -halfworld )
		d += world_size;

		return d;
	}

	/*
		Normalize a length to within 0 to worldsize
		from rtv's Antix
	*/
	static double
	DistanceNormalize(double d) {
		while( d < 0 ) d += world_size;
		while( d > world_size ) d -= world_size;
		return d; 
	}

	/*
		Normalize an angle to within +/- M_PI
		from rtv's Antix
	*/
	static double
	AngleNormalize(double a) {
		while( a < -M_PI ) a += 2.0*M_PI;
		while( a >  M_PI ) a -= 2.0*M_PI;	 
		return a;
	}

	/*
		these cell methods similar/same to those from rtv's antix
	*/

	static inline unsigned int
	Cell_x(double x) {
		const double d = antix::offset_size / (double) matrix_width;

		// this can be < 0 and cause invalid value due to unsigned
		const double new_x = x - my_min_x;
		if (new_x < 0)
			x = 0;
		else
			x = new_x;

		// wraparound
		// we don't wrap around x
		/*
		while (x > world_size)
			x -= world_size;

		while (x < 0)
			x += world_size;
		*/
		
		//unsigned int i = floor(x/d);
		//assert(i < matrix_width * matrix_height + 1000);

		return floor(x / d);
	}

	static inline unsigned int
	Cell_y(double x) {
		const double d = antix::world_size / (double) matrix_height;

		// wraparound
		while (x > world_size)
			x -= world_size;
		while (x < 0)
			x += world_size;

		//unsigned int i = floor(x/d);
		//assert(i < matrix_width * matrix_height + 1000);

		return floor(x / d);
	}

	static inline unsigned int
	CellWrap(int x) {
		while (x >= (int) matrix_height)
			x -= matrix_height;
		while (x < 0)
			x += matrix_height;
		return x;
	}

	static inline unsigned int
	Cell(double x, double y) {
		unsigned int cx = Cell_x(x);
		unsigned int cy = Cell_y(y);
		unsigned int i = cx + cy * matrix_width;
		//cout << "Cell: cx " << cx << " x " << x << " cy " << cy << " y " << y << " = " << i << endl;
		//unsigned int i = Cell_x(x) + Cell_y(y) * matrix_width;
		//assert(i < matrix_width * matrix_height + 1000);
		return ( Cell_x(x) + ( Cell_y(y) * matrix_width ) );
	}
};

double antix::offset_size;
double antix::world_size;
double antix::my_min_x;
double antix::home_radius;
unsigned int antix::matrix_width;
unsigned int antix::matrix_height;
unsigned int antix::matrix_right_x_col;
unsigned int antix::matrix_right_world_col;
int antix::turn = 0;

#endif
