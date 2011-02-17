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
#include <assert.h>
#include <set>

#include "antix.pb.h"

#define SLEEP 0
#define DEBUG 0
#define GUI 0
// To disable asserts, define this
#define NDEBUG

// handy STL iterator macro pair. Use FOR_EACH(I,C){ } to get an iterator I to
// each item in a collection C.
// from rtv's Antix
#define VAR(V,init) __typeof(init) V=(init)
#define FOR_EACH(I,C) for(VAR(I,(C).begin());I!=(C).end();I++)

using namespace std;

/*
	Random double between the two given doubles
*/
static double
rand_between(double min, double max) {
	return ( (drand48() * (max - min) ) + min );
}

/*
	Normalize a length to within 0 to worldsize
	from rtv's Antix
*/
static double
DistanceNormalize(double d, double world_size) {
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
	int team;

	Home(double x, double y, double r, int team) : x(x), y(y), r(r), team(team) {
		colour = Colour();
	}

	Home(double x, double y, double r, Colour colour) : x(x), y(y), r(r), colour(colour) { }

	Home(double r, double world_size) : r(r) {
		x = rand_between(0, world_size);
		y = rand_between(0, world_size);
		colour = Colour();
	}
};

class Robot;

class Puck {
public:
	double x,
		y;
	bool held;
	Robot *robot;

	// random pose stuff is from rtv's Antix
	Puck(double min_x, double max_x, double world_size) {
		x = rand_between(min_x, max_x);
		y = rand_between(0, world_size);
		held = false;
		robot = NULL;
	}
	Puck(double x, double y, bool held) : x(x), y(y), held(held) {
		robot = NULL;
	}
};

class SeePuck {
public:
	double range;
	Puck *puck;

	SeePuck(Puck *puck, double range) : puck(puck), range(range) {}
};

class Robot {
public:
	double x, y;
	// orientation
	double a;
	// forward speed
	double v;
	// turn speed
	double w;

	// together uniquely identifies the robot
	// team = client id, essentially
	int team;
	int id;

	bool has_puck;
	Puck *puck;
	Home *home;

	// store what pucks we can see
	vector<SeePuck> see_pucks;

	Robot(double x, double y, int id, int team) : x(x), y(y), id(id), team(team) {
		a = 0;
		v = 0;
		w = 0;
		puck = NULL;
		has_puck = false;
	}

	Robot(double x, double y, int team, double a) : x(x), y(y), team(team), a(a) {
		id = -1;
		v = 0;
		w = 0;
		puck = NULL;
		has_puck = false;
	}

	/*
		update the pose of a single robot
		Taken from rtv's Antix
	*/
	void
	update_pose(double world_size) {
		double dx = v * cos(a);
		double dy = v * sin(a);
		double da = w;

		x = DistanceNormalize(x + dx, world_size);
		y = DistanceNormalize(y + dy, world_size);
		a = AngleNormalize(a + da);

		// If we're holding a puck, it must move also
		if (has_puck) {
			assert(puck != NULL);
			assert(puck->robot == this);
			assert(puck->held == true);
			puck->x = x;
			puck->y = y;
		}
	}
};

// Robot class for clients
class CRobot {
public:
	double last_x;
	double last_y;

	CRobot(double last_x, double last_y) : last_x(last_x), last_y(last_y) {}
	CRobot() {
		// XXX potentially bad
		last_x = 0.0;
		last_y = 0.0;
	}
};

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
		sleep for the given milliseconds
		usleep() usage taken from rtv's Antix
	*/
	static void
	sleep(int ms) {
		usleep(ms * 1e3);
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

	static void
	send_blank_envelope(zmq::socket_t *sock, string address) {
		zmq::message_t type(address.size() + 1);
		memcpy(type.data(), address.c_str(), address.size() + 1);
		sock->send(type, ZMQ_SNDMORE);
		send_blank(sock);
	}

	static void
	send_pb_envelope(zmq::socket_t *sock, google::protobuf::Message *pb_obj, string address) {
		zmq::message_t type(address.size() + 1);
		memcpy(type.data(), address.c_str(), address.size() + 1);
		sock->send(type, ZMQ_SNDMORE);
		send_pb(sock, pb_obj);
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
		Send master a message stating we're done a turn
		Then wait until master contacts us so that all nodes/clients are in sync
	*/
	static void
	wait_for_next_turn(zmq::socket_t *master_req_sock,
		zmq::socket_t *master_sub_sock,
		int id,
		antixtransfer::done::Type type) {

		antixtransfer::done done_msg;
		done_msg.set_my_id( id );
		done_msg.set_type( type );
		antix::send_pb_envelope(master_req_sock, &done_msg, "done");
#if DEBUG
		cout << "Sent done signal to master" << endl;
#endif
		// necessary response due to REQ socket
		antix::recv_blank(master_req_sock);

		// now we block on PUB sock awaiting begin
		antix::recv_blank(master_sub_sock);
#if DEBUG
		cout << "Received begin turn signal from master" << endl;
#endif
	}

	/*
		Copy the data from src to dest
	*/
	static void
	copy_node(antixtransfer::Node_list::Node *dest, antixtransfer::Node_list::Node *src) {
		dest->set_id( src->id() );
		dest->set_ip_addr( src->ip_addr() );
		dest->set_neighbour_port( src->neighbour_port() );
		dest->set_control_port( src->control_port() );
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
	}

	static void
	print_nodes(antixtransfer::Node_list *node_list) {
		const antixtransfer::Node_list::Node *node;
		for (int i = 0; i < node_list->node_size(); i++) {
			node = node_list->mutable_node(i);
			cout << "\tNode id: " << node->id();
			cout << " IP: " << node->ip_addr();
			cout << " Neighbour port: " << node->neighbour_port();
			cout << " Control port: " << node->control_port();
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
	WrapDistance(double d, double world_size) {
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
	DistanceNormalize(double d, double world_size) {
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
		Update the home pointer on the Robot r from the given homes vector
	*/
	static void
	set_robot_home(Robot *r, vector<Home> *homes) {
		for (vector<Home>::iterator it = homes->begin(); it != homes->end(); it++) {
			if (r->team == it->team) {
				r->home = &*it;
				return;
			}
		}
		r->home = NULL;
		assert(r->home != NULL);
	}
};

