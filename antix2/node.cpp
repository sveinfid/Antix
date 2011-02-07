/*
	Node / map piece server

	The majority of the ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all
	Most notably the pub/sub message enveloping example used in the functions
	move_robot() and parse_neighbour_message(). Similar to:
	https://github.com/imatix/zguide/blob/master/examples/C++/psenvpub.cpp
	https://github.com/imatix/zguide/blob/master/examples/C++/psenvsub.cpp

	Bugs:
	- Synchronization issue with pub/sub sockets: some initial messages may/will
	be lost
*/

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_node_port = "7770";
string master_publish_port = "7773";

int my_id;
double world_size;
double my_min_x;
// where neighbour begins
double my_max_x;
int sleep_time;
int initial_puck_amount;

vector<Puck> pucks;
vector<Robot> robots;
vector<Puck> left_foreign_pucks;
vector<Robot> left_foreign_robots;
vector<Puck> right_foreign_pucks;
vector<Robot> right_foreign_robots;

antixtransfer::Node_list::Node left_node;
antixtransfer::Node_list::Node right_node;

// Connect to master & identify ourselves. Get state
zmq::socket_t *master_control_sock;
// Master publishes list of nodes to us when beginning simulation
zmq::socket_t *master_publish_sock;
// neighbours publish foreign entities & move bot msgs on these
zmq::socket_t *left_sub_sock;
zmq::socket_t *right_sub_sock;
// we publish our border entities to these sockets
zmq::socket_t *left_pub_sock;
zmq::socket_t *right_pub_sock;
// control socket to service client commands
zmq::socket_t *control_sock;
// control sockets to speak to neighbours. useful for synchronization
//zmq::socket_t *left_control_sock;
//zmq::socket_t	*right_control_sock;

/*
	Find our offset in node_list and set my_min_x, my_max_x
*/
void
set_dimensions(antixtransfer::Node_list *node_list) {
	antixtransfer::Node_list::Node *node;
	double offset_size = world_size / node_list->node_size();

	for (int i = 0; i < node_list->node_size(); i++) {
		node = node_list->mutable_node(i);
		if (node->id() == my_id) {
			my_min_x = node->x_offset();
			my_max_x = my_min_x + offset_size;
			break;
		}
	}
	cout << "Set dimensions of this node. min x: " << my_min_x << " max x: " << my_max_x << endl;
}

/*
	Place pucks randomly within our region
*/
void
generate_pucks() {
	for (int i = 0; i < initial_puck_amount; i++) {
		pucks.push_back( Puck(my_min_x, my_max_x, world_size) );
	}
}

void
print_foreign_entities() {
	cout << "Current foreign entities: " << endl;
	for (vector<Robot>::iterator it = left_foreign_robots.begin(); it != left_foreign_robots.end(); it++)
		cout << "\tRobot at " << it->x << ", " << it->y << endl;
	for (vector<Robot>::iterator it = right_foreign_robots.begin(); it != right_foreign_robots.end(); it++)
		cout << "\tRobot at " << it->x << ", " << it->y << endl;
	for (vector<Puck>::iterator it = left_foreign_pucks.begin(); it != left_foreign_pucks.end(); it++)
		cout << "\tPuck at " << it->x << ", " << it->y << endl;
	for (vector<Puck>::iterator it = right_foreign_pucks.begin(); it != right_foreign_pucks.end(); it++)
		cout << "\tPuck at " << it->x << ", " << it->y << endl;
}

/*
	Given a map which may or may not contain entities, add any entities therein
	to our internal records of foreign robots & pucks
*/
void
update_foreign_entities(antixtransfer::SendMap *map, vector<Robot> *foreign_robots, vector<Puck> *foreign_pucks) {
	foreign_robots->clear();
	foreign_pucks->clear();
	// foreign robots
	for (int i = 0; i < map->robot_size(); i++) {
		foreign_robots->push_back( Robot( map->robot(i).x(), map->robot(i).y(), map->robot(i).id(), map->robot(i).team() ) );
	}
	// foreign pucks
	for (int i = 0; i < map->puck_size(); i++) {
		foreign_pucks->push_back( Puck( map->puck(i).x(), map->puck(i).y(), map->puck(i).held() ) );
	}
}

/*
	Send entities which are within sight distance of our left border
	and within sight distance of our right border
*/
void
send_border_entities() {
	// container for robots & pucks
	antixtransfer::SendMap send_map;
	// XXX Right now we only have one big list of robots, so all are sent
	for(vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		antixtransfer::SendMap::Robot *robot = send_map.add_robot();
		robot->set_x( it->x );
		robot->set_y( it->y );
		robot->set_team( it->team );
		robot->set_id( it->id );
	}

	// XXX only one list of pucks right now
	for(vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		antixtransfer::SendMap::Puck *puck = send_map.add_puck();
		puck->set_x( it->x );
		puck->set_y( it->y );
		puck->set_held( it->held );
	}

	cout << "Sending " << send_map.puck_size() << " pucks and " << send_map.robot_size() << " robots to our neighbours..." << endl;
	zmq::message_t type_left(2);
	memcpy(type_left.data(), "f", 2);
	left_pub_sock->send(type_left, ZMQ_SNDMORE);
	antix::send_pb(left_pub_sock, &send_map);

	zmq::message_t type_right(2);
	memcpy(type_right.data(), "f", 2);
	right_pub_sock->send(type_right, ZMQ_SNDMORE);
	antix::send_pb(right_pub_sock, &send_map);
	cout << "Sent border entities to neighbours." << endl;
}

/*
	Know there is a message waiting from a neighbour. Handle it
*/
void
parse_neighbour_message(zmq::socket_t *sub_sock, zmq::message_t *type_msg, vector<Robot> *foreign_robots, vector<Puck> *foreign_pucks) {
	// First we read the envelope address (specifies from neighbour or a client)
	// type_msg contains this
	string s = string((char *) type_msg->data());
	cout << "Received node control message: " << s << endl;

	// Foreign entity update
	if (s == "f") {
		antixtransfer::SendMap map_foreign;
		antix::recv_pb(sub_sock, &map_foreign, 0);
		update_foreign_entities(&map_foreign, foreign_robots, foreign_pucks);
		cout << "Received " << map_foreign.puck_size() << " pucks and " << map_foreign.robot_size() << " robots from a neighbour" << endl;
	}
	// Move bot
	else if (s == "m") {
		antixtransfer::RequestRobotTransfer move_msg;
		antix::recv_pb(sub_sock, &move_msg, 0);
		Robot r(move_msg.x(), move_msg.y(), move_msg.id(), move_msg.team());
		r.a = move_msg.a();
		r.v = move_msg.v();
		r.w = move_msg.w();
		r.has_puck = move_msg.has_puck();
		// If the robot is carrying a puck, we have to add a puck to our records
		if (r.has_puck) {
			Puck p(r.x, r.y, true);
			pucks.push_back(p);
			r.puck = &p;
		}
		robots.push_back(r);
		cout << "Robot transferred to this node." << endl;
	}
}

/*
	Clear old foreign robots/pucks and attempt to re-fill based on msgs from our
	neighbours
*/
void
recv_neighbour_messages() {
	zmq::message_t type_msg;
	// Not really fair, but good enough for now
	// (as we first do all of the left node's, then the right's
	while (left_sub_sock->recv(&type_msg, ZMQ_NOBLOCK) == 1) {
		parse_neighbour_message(left_sub_sock, &type_msg, &left_foreign_robots, &left_foreign_pucks);
	}
	while (right_sub_sock->recv(&type_msg, ZMQ_NOBLOCK) == 1) {
		parse_neighbour_message(right_sub_sock, &type_msg, &right_foreign_robots, &right_foreign_pucks);
	}
	print_foreign_entities();
}

/*
	Remove the puck that robot r is carrying, if it is carrying one
*/
void
remove_puck(Robot *r) {
	if (!r->has_puck)
		return;
	for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		if (it->robot == r) {
			pucks.erase(it);
			break;
		}
	}
}

/*
	Robot has been found to be outside of our map portion
	Transfer the robot (and possibly its puck) to the given node
	Note: Removal of robot from local records takes place elsewhere
*/
void
move_robot(Robot *r) {
	antixtransfer::Node_list::Node *node;
	zmq::socket_t *pub_sock;

	if (r->x < my_min_x) {
		node = &left_node;
		pub_sock = left_pub_sock;
		cout << "Moving robot to left_pub_sock" << endl;
	} else {
		node = &right_node;
		pub_sock = right_pub_sock;
		cout << "Moving robot to right_pub_sock" << endl;
	}

	// transfer the robot
	antixtransfer::RequestRobotTransfer transfer_msg;
	transfer_msg.set_id(r->id);
	transfer_msg.set_team(r->team);
	transfer_msg.set_x(r->x);
	transfer_msg.set_y(r->y);
	transfer_msg.set_a(r->a);
	transfer_msg.set_v(r->v);
	transfer_msg.set_w(r->w);
	transfer_msg.set_has_puck(r->has_puck);

	// First we send the address/type message
	zmq::message_t type(2);
	memcpy(type.data(), "m", 2);
	pub_sock->send(type, ZMQ_SNDMORE);
	// Then we send our content msg
	antix::send_pb(pub_sock, &transfer_msg);

	cout << "Transferred robot " << r->id << " on team " << r->team << endl;
}

/*
	Go through our local robots & update their poses
*/
void
update_poses() {
	// For each robot, update its pose
	vector<Robot>::iterator it = robots.begin();
	// while loop as it can be updated other than from for iteration
	while (it != robots.end()) {
		cout << "Updating poses!" << endl;
		it->update_pose(world_size);

		// Then check each robot for being outside of our range
		// separate from update_pose() as we must be careful deleting from vector
		// while using an iterator
		if (it->x < my_min_x || it->x >= my_max_x) {
			cout << "Robot " << it->id << " team " << it->team << " out of range, moving..." << endl;
			// Remove puck if robot is carrying one
			remove_puck(&*it);
			move_robot(&*it);
			cout << "Past move robot " <<endl;
			it = robots.erase(it);
			cout << "Past robot erase" << endl;
		} else {
			it++;
		}
	}
	cout << "Poses updated for all robots." << endl;
}

/*
	Wait for a response from our neighbours which indicates synchronization
	Without this messages are lost on the neighbour_publish_socket

	XXX Not absolutely correct? Nodes can still start slightly unsynced - although
	this is an improvement

	This is based on the example from
	http://github.com/imatix/zguide/blob/master/examples/C%2B%2B/syncsub.cpp
*/
/*
void
synchronize_neighbours(zmq::context_t *context, zmq::socket_t *control_sock) {
	// First we send a blank message to both of our neighbours
	zmq::message_t blank_left(1);
	left_control_sock->send(blank_left);
	zmq::message_t blank_right(1);
	right_control_sock->send(blank_right);

	int acks_received = 0;
	int syncs_responded = 0;
	// same polling as in master
	zmq::pollitem_t items[] = {
		{ *control_sock, 0, ZMQ_POLLIN, 0},
		{ *left_control_sock, 0, ZMQ_POLLIN, 0},
		{ *right_control_sock, 0, ZMQ_POLLIN, 0}
	};
	// Then we wait for a response & respond to those sent to us
	while (acks_received < 2 || syncs_responded < 2) {
		zmq::message_t response;
		zmq::poll(&items [0], 3, -1);

		// sync requested on control port
		if (items[0].revents & ZMQ_POLLIN) {
			cout << "Received sync request. Sending response..." << endl;
			control_sock->recv(&response);
			zmq::message_t blank(1);
			control_sock->send(blank);
			syncs_responded++;
		}
		// left sync response
		if (items[1].revents & ZMQ_POLLIN) {
			cout << "Received sync response from left node" << endl;
			left_control_sock->recv(&response);
			acks_received++;
		}
		// right sync response
		if (items[2].revents & ZMQ_POLLIN) {
			cout << "Received sync response from right node" << endl;
			right_control_sock->recv(&response);
			acks_received++;
		}
	}
}
*/

Robot *
find_robot(int team, int id) {
	for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		if (it->team == team && it->id == id)
			return &*it;
	}
	return NULL;
}

/*
	A client has sent an add_bot message with an arbitrary number of robots
*/
void
add_bot(antixtransfer::control_message *msg) {
	for (int i = 0; i < msg->robot_size(); i++) {
		Robot r(antix::rand_between(my_min_x, my_max_x), antix::rand_between(0, world_size), msg->robot(i).id(), msg->team());
		robots.push_back(r);
	}
	// required response, but nothing much to say
	antix::send_blank(control_sock);
}

/*
	A client has sent a sense message. We send the map for each of its robots as
	a response
*/
void
sense(antixtransfer::control_message *msg) {
	// XXX right now there is only one map, so just reply with all our entities
	antixtransfer::SendMap map;
	for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		antixtransfer::SendMap::Puck *puck = map.add_puck();
		puck->set_x( it->x );
		puck->set_y( it->y );
		puck->set_held( it->held );
	}
	for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		antixtransfer::SendMap::Robot *robot = map.add_robot();
		robot->set_team( it->team );
		robot->set_id( it->id );
		robot->set_x( it->x );
		robot->set_y( it->y );
	}

	antix::send_pb(control_sock, &map);
}

/*
	For each robot, update the speed entry (if the robot is held by us)
*/
void
setspeed(antixtransfer::control_message *msg) {
	for (int i = 0; i < msg->robot_size(); i++) {
		// XXX unacceptable cost to find each robot this way!
		Robot *r = find_robot(msg->team(), msg->robot(i).id());
		if (r != NULL) {
			r->v = msg->robot(i).v();
			r->w = msg->robot(i).w();
		}
	}

	antix::send_blank(control_sock);
}

void
pickup(antixtransfer::control_message *msg) {
	// TODO

	// XXX must send a response
	antix::send_blank(control_sock);
}

void
drop(antixtransfer::control_message *msg) {
	// TODO

	// XXX must send a response
	antix::send_blank(control_sock);
}

/*
	Service control messages from clients
*/
void
service_control_messages() {
	antixtransfer::control_message msg;
	while (antix::recv_pb(control_sock, &msg, ZMQ_NOBLOCK) == 1) {
		cout << "Received a client control message" << endl;
		if (msg.type() == antixtransfer::control_message::ADD_BOT) {
			add_bot(&msg);
		} else if (msg.type() == antixtransfer::control_message::SENSE) {
			sense(&msg);
		} else if (msg.type() == antixtransfer::control_message::SETSPEED) {
			setspeed(&msg);
		} else if (msg.type() == antixtransfer::control_message::PICKUP) {
			pickup(&msg);
		} else {
			drop(&msg);
		}
	}
}

int main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);
	
	if (argc != 5) {
		cerr << "Usage: " << argv[0] << " <IP to listen on> <left node port> <right node port> <control port>" << endl;
		return -1;
	}

	// socket to announce ourselves to master on
	master_control_sock = new zmq::socket_t(context, ZMQ_REQ);
	master_control_sock->connect(antix::make_endpoint(master_host, master_node_port));
	cout << "Connecting to master..." << endl;

	// socket to receive list of nodes on
	master_publish_sock = new zmq::socket_t(context, ZMQ_SUB);
	// subscribe to all messages on this socket: should just be a list of nodes
	master_publish_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock->connect(antix::make_endpoint(master_host, master_publish_port));

	// send message announcing ourself. includes our own IP
	cout << "Sending master our existence notification..." << endl;

	// create & send pb msg identifying ourself
	antixtransfer::connect_init_node pb_init_msg;
	pb_init_msg.set_ip_addr( string(argv[1]) );
	pb_init_msg.set_left_port( string(argv[2]) );
	pb_init_msg.set_right_port( string(argv[3]) );
	pb_init_msg.set_control_port( string(argv[4]) );
	antix::send_pb(master_control_sock, &pb_init_msg);

	// receive message back stating our unique ID
	antixtransfer::connect_init_response init_response;
	antix::recv_pb(master_control_sock, &init_response, 0);
	my_id = init_response.id();
	world_size = init_response.world_size();
	sleep_time = init_response.sleep_time();
	initial_puck_amount = init_response.puck_amount();
	cout << "We are now node id " << my_id << endl;

	// receive node list
	// blocks until master publishes list of nodes
	antixtransfer::Node_list node_list;
	antix::recv_pb(master_publish_sock, &node_list, 0);
	cout << "Received list of nodes:" << endl;
	antix::print_nodes(&node_list);

	if (node_list.node_size() < 3) {
		cout << "Error: we need at least 3 nodes. Only received " << node_list.node_size() << " node(s)." << endl;
		return -1;
	}

	// calculate our min / max x from the offset assigned to us in node_list
	set_dimensions(&node_list);

	// find our left/right neighbours
	antix::set_neighbours(&left_node, &right_node, &node_list, my_id);
	cout << "Left neighbour id: " << left_node.id() << " " << left_node.ip_addr() << " left port " << left_node.left_port() << " right port " << left_node.right_port() << endl;
	cout << "Right neighbour id: " << right_node.id() << " " << right_node.ip_addr() << " left port " << right_node.left_port() << " right port " << right_node.right_port() << endl;

	// connect & subscribe to both neighbour's PUB sockets
	// these receive foreign entities that are near our border
	left_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
	left_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	left_sub_sock->connect(antix::make_endpoint(left_node.ip_addr(), left_node.right_port()));

	right_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
	right_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	right_sub_sock->connect(antix::make_endpoint(right_node.ip_addr(), right_node.left_port()));

	// open PUB socket neighbours where we publish entities close to the borders
	left_pub_sock = new zmq::socket_t(context, ZMQ_PUB);
	left_pub_sock->bind(antix::make_endpoint( argv[1], argv[2] ));
	right_pub_sock = new zmq::socket_t(context, ZMQ_PUB);
	right_pub_sock->bind(antix::make_endpoint( argv[1], argv[3] ));

	// create REP socket that receives control messages from clients
	control_sock = new zmq::socket_t(context, ZMQ_REP);
	control_sock->bind(antix::make_endpoint( argv[1], argv[4] ));

	// connect out to the control sockets of our neighbours
	//connect_neighbour_control_socks(&context);

	// Before we enter main loop, we must synchronize our connection to our
	// neighbours PUB sockets (neighbour_publish_sock), or else we risk losing
	// the initial messages
	// Buggy, interferes with control sockets currently
	//synchronize_neighbours(&context, &control_sock);

	// generate pucks
	generate_pucks();
	cout << "Created " << pucks.size() << " pucks." << endl;
	for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		cout << "Puck at " << it->x << "," << it->y << endl;
	}

	// enter main loop
	while (1) {
		// send entities within sight_range of our borders to our neighbours
		send_border_entities();

		// read from our neighbour SUB sockets
		// - update our foreign entity knowledge
		// - handle any move_bot messages
		recv_neighbour_messages();

		// update poses for internal robots & move any robots outside our control
		update_poses();

		// service control messages on our REP socket
		service_control_messages();

		antix::sleep(sleep_time);
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
