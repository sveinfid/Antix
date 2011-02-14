/*
	Node / map piece server

	The majority of the ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all
	Most notably the pub/sub message enveloping example used in the functions
	move_robot() and parse_neighbour_message(). Similar to:
	https://github.com/imatix/zguide/blob/master/examples/C++/psenvpub.cpp
	https://github.com/imatix/zguide/blob/master/examples/C++/psenvsub.cpp
*/

#include "antix.cpp"

using namespace std;

string master_host;
string master_node_port = "7770";
string master_publish_port = "7773";

string my_ip;
string my_neighbour_port;
string my_control_port;

int my_id;
double world_size;
double offset_size;
double my_min_x;
// where neighbour begins
double my_max_x;
int sleep_time;
int initial_puck_amount;
double vision_range;

// the robots & pucks we control
vector<Puck> pucks;
vector<Robot> robots;
// sent to us by neighbours
vector<Puck> foreign_pucks;
vector<Robot> foreign_robots;
// robots & pucks near our borders that we send to our neighbours
antixtransfer::SendMap border_map_left;
antixtransfer::SendMap border_map_right;

antixtransfer::Node_list node_list;
antixtransfer::Node_list::Node left_node;
antixtransfer::Node_list::Node right_node;

// Connect to master & identify ourselves. Get state
zmq::socket_t *master_req_sock;
// Master publishes list of nodes to us when beginning simulation
zmq::socket_t *master_publish_sock;

// request border entities from neighbour on these REQ sockets
zmq::socket_t *right_req_sock;
zmq::socket_t *left_req_sock;
// handle neighbours requesting border entities on this REP sock (neighbour port)
zmq::socket_t *neighbour_rep_sock;

// clients request commands on this sock;
zmq::socket_t *control_rep_sock;

/*
	Find our offset in node_list and set my_min_x, my_max_x
*/
void
set_dimensions(antixtransfer::Node_list *node_list) {
	antixtransfer::Node_list::Node *node;

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
	cout << "Created " << pucks.size() << " pucks." << endl;
	for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		cout << "Puck at " << it->x << "," << it->y << endl;
	}
}

/*
	Useful for debugging: Print out what foreign entities we know of
*/
void
print_foreign_entities() {
	cout << "Current foreign entities: " << endl;
	for (vector<Robot>::iterator it = foreign_robots.begin(); it != foreign_robots.end(); it++)
		cout << "\tRobot at " << it->x << ", " << it->y << endl;
	for (vector<Puck>::iterator it = foreign_pucks.begin(); it != foreign_pucks.end(); it++)
		cout << "\tPuck at " << it->x << ", " << it->y << endl;
}

/*
	Output the information on our robots
*/
void
print_local_robots() {
	cout << "Current local robots:" << endl;
	for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		cout << "Robot " << it->id << " on team " << it->team << " at (" << it->x << ", " << it->y << ") a: " << it->a << " v: " << it->v << endl;
	}
}

/*
	A map is waiting to be read on given sock
	The map may or may not contain entities, add any entities therein
	to our internal records of foreign robots & pucks
*/
void
update_foreign_entities(zmq::socket_t *sock) {
	antixtransfer::SendMap map;
	antix::recv_pb(sock, &map, 0);

	// foreign robots
	for (int i = 0; i < map.robot_size(); i++) {
		foreign_robots.push_back( Robot( map.robot(i).x(), map.robot(i).y(), map.robot(i).id(), map.robot(i).team() ) );
	}
	// foreign pucks
	for (int i = 0; i < map.puck_size(); i++) {
		foreign_pucks.push_back( Puck( map.puck(i).x(), map.puck(i).y(), map.puck(i).held() ) );
	}
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
	Add the relevant data to a new Robot entry in the given move_bot message
*/
void
add_move_robot(Robot *r, antixtransfer::move_bot *move_bot_msg) {
	antixtransfer::move_bot::Robot *r_move = move_bot_msg->add_robot();
	r_move->set_id(r->id);
	r_move->set_team(r->team);
	r_move->set_x(r->x);
	r_move->set_y(r->y);
	r_move->set_a(r->a);
	r_move->set_v(r->v);
	r_move->set_w(r->w);
	r_move->set_has_puck(r->has_puck);
}

/*
	Look through our list of robots to see if any are outside of our range
	If found, add the relevant data to a move_bot message
	and remove relevant data from our records
*/
void
build_move_message(antixtransfer::move_bot *move_left_msg, antixtransfer::move_bot *move_right_msg) {
	vector<Robot>::iterator it = robots.begin();
	// while loop as iterator may be updated other due to deletion
	while (it != robots.end()) {
		// We do these 4 cases as it's possible the robot has looped around the world
		// where the robot will be less than our x but actually travels to the right
		// node (one case)

		// If robot's x is less than ours and bigger than our left neighbours, send
		// to our left neighbour
		if (it->x < my_min_x && it->x > my_min_x - offset_size) {
			add_move_robot(&*it, move_left_msg);
			remove_puck(&*it);
			it = robots.erase(it);

		// Otherwise if it's less than ours and smaller than our left neighbour's,
		// assume that we are the far right node: send it to our right neighbour
		} else if (it->x < my_min_x) {
			add_move_robot(&*it, move_right_msg);
			remove_puck(&*it);
			it = robots.erase(it);

		// If robot's x is bigger than ours and smaller than our right neighbour's, we
		// send it to our right neighbour
		} else if (it->x >= my_max_x && it->x < my_max_x + offset_size) {
			add_move_robot(&*it, move_right_msg);
			remove_puck(&*it);
			it = robots.erase(it);

		// Otherwise it's bigger than ours and bigger than our right neighbour's,
		// assume we are the far left node: send it to our left neighbour
		} else if (it->x >= my_max_x) {
			add_move_robot(&*it, move_left_msg);
			remove_puck(&*it);
			it = robots.erase(it);

		} else {
			it++;
		}
	}
}

/*
	We know a node has sent a move request message
	Read it and add all the robots in the message to our local robot listing
*/
void
handle_move_request(antixtransfer::move_bot *move_bot_msg) {
	// now we get a message of type move_bot
	// for each robot in the message, add it to our list
	int i;
	for(i = 0; i < move_bot_msg->robot_size(); i++) {
		Robot r(move_bot_msg->robot(i).x(), move_bot_msg->robot(i).y(), move_bot_msg->robot(i).id(), move_bot_msg->robot(i).team());
		r.a = move_bot_msg->robot(i).a();
		r.v = move_bot_msg->robot(i).v();
		r.w = move_bot_msg->robot(i).w();
		r.has_puck = move_bot_msg->robot(i).has_puck();
		// If the robot is carrying a puck, we have to add a puck to our records
		if (r.has_puck) {
			Puck p(r.x, r.y, true);
			p.robot = &r;
			pucks.push_back(p);
			r.puck = &p;
		}
		robots.push_back(r);
	}
	cout << i << " robots transferred to this node." << endl;
}

/*
	Move any of our robots to neighbours if necessary
	Receive any of the same

	Each neighbour must send us one request (even if blank), and we must send one
	request (even if blank)
*/
void
send_move_messages() {
	// First we build our own move messages to be sent to our neighbours
	antixtransfer::move_bot move_left_msg;
	move_left_msg.set_from_right(true);
	antixtransfer::move_bot move_right_msg;
	move_right_msg.set_from_right(false);
	build_move_message(&move_left_msg, &move_right_msg);

	// Send our move messages
	antix::send_pb(left_req_sock, &move_left_msg);
	antix::send_pb(right_req_sock, &move_right_msg);

	cout << "Movement messages sent." << endl;
}

/*
	Add the given puck to the given map
*/
void
add_border_puck(antixtransfer::SendMap *map, Puck *p) {
	antixtransfer::SendMap::Puck *p_pb = map->add_puck();
	p_pb->set_x( p->x );
	p_pb->set_y( p->y );
	p_pb->set_held( p->held );
}

/*
	Add given robot to the given map
*/
void
add_border_robot(antixtransfer::SendMap *map, Robot *r) {
	antixtransfer::SendMap::Robot *r_pb = map->add_robot();
	r_pb->set_x( r->x );
	r_pb->set_y( r->y );
	r_pb->set_team( r->team );
	r_pb->set_id( r->id );
}

/*
	Clear the last iteration's border entities, and place any of our robots &
	pucks where they belong
*/
void
rebuild_border_entities() {
	border_map_left.clear_robot();
	border_map_left.clear_puck();
	border_map_right.clear_robot();
	border_map_right.clear_puck();

	for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		if (it->x > my_max_x - vision_range)
			add_border_puck(&border_map_right, &*it);
		else if (it->x < my_min_x + vision_range)
			add_border_puck(&border_map_left, &*it);
	}

	for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		if (it->x > my_max_x - vision_range)
			add_border_robot(&border_map_right, &*it);
		else if (it->x < my_min_x + vision_range)
			add_border_robot(&border_map_left, &*it);
	}

	/*
	// for debugging
	cout << "Left border entities: " << endl;
	for (int i = 0; i < border_map_left.robot_size(); i++) {
		cout << "Robot at " << border_map_left.robot(i).x() << ", " << border_map_left.robot(i).y() << endl;
	}
	for (int i = 0; i < border_map_left.puck_size(); i++) {
		cout << "Puck at " << border_map_left.puck(i).x() << ", " << border_map_left.puck(i).y() << endl;
	}

	cout << "Right border entities: " << endl;
	for (int i = 0; i < border_map_right.robot_size(); i++) {
		cout << "Robot at " << border_map_right.robot(i).x() << ", " << border_map_right.robot(i).y() << endl;
	}
	for (int i = 0; i < border_map_right.puck_size(); i++) {
		cout << "Puck at " << border_map_right.puck(i).x() << ", " << border_map_right.puck(i).y() << endl;
	}
	*/
}

/*
	Send our foreign/border entities to our 2 neighbours
	Receive the same from each neighbour
*/
void
exchange_foreign_entities() {
	// first we re-calculate what entities local to us are near borders
	rebuild_border_entities();

	// clear old foreign entities
	foreign_pucks.clear();
	foreign_robots.clear();

	// We wait for any requests (move requests in this case), to which we respond
	// by giving the requester a list of our foreign entities
	// We also wait for a response from our earlier move requests
	zmq::pollitem_t items [] = {
		{ *left_req_sock, 0, ZMQ_POLLIN, 0 },
		{ *right_req_sock, 0, ZMQ_POLLIN, 0},
		{ *neighbour_rep_sock, 0, ZMQ_POLLIN, 0}
	};
	// Both of these must be 2 before we continue
	int responses = 0;
	int requests = 0;
	// Keep waiting for messages until we've received the number we expect
	while (responses < 2 || requests < 2) {
		zmq::poll(&items [0], 3, -1);

		// left_req response
		if (items[0].revents & ZMQ_POLLIN) {
			update_foreign_entities(left_req_sock);
			responses++;
			cout << "Received foreign entities response from left req sock" << endl;
		}

		// right_req response
		if (items[1].revents & ZMQ_POLLIN) {
			update_foreign_entities(right_req_sock);
			responses++;
			cout << "Received foreign entities response from right req sock" << endl;
		}

		// neighbour request
		if (items[2].revents & ZMQ_POLLIN) {
			antixtransfer::move_bot move_bot_msg;
			antix::recv_pb(neighbour_rep_sock, &move_bot_msg, 0);
			// we have received a move request: first update our local records with
			// the sent bots
			handle_move_request(&move_bot_msg);

			// send back a list of foreign neighbours depending on which neighbour
			// the move request was from
			if (move_bot_msg.from_right() == false)
				antix::send_pb(neighbour_rep_sock, &border_map_left);
			else
				antix::send_pb(neighbour_rep_sock, &border_map_right);
			requests++;
			cout << "Received move request from a neighbour, sent border entities" << endl;
		}
	}

	cout << "Done exchanging foreign entities." << endl;
	print_foreign_entities();
}

/*
	Go through our local robots & update their poses
*/
void
update_poses() {
	for(vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		it->update_pose(world_size);
	}
	cout << "Poses updated for all robots." << endl;
}

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
		// XXX for testing
		r.v = 0.01;
		robots.push_back(r);
		cout << "Created a bot: Team: " << r.team << " id: " << r.id << " at (" << r.x << ", " << r.y << ")" << endl;
	}
	// required response, but nothing much to say
	antix::send_blank(control_rep_sock);
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

	antix::send_pb(control_rep_sock, &map);
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

	antix::send_blank(control_rep_sock);
}

void
pickup(antixtransfer::control_message *msg) {
	// TODO

	// XXX must send a response
	antix::send_blank(control_rep_sock);
}

void
drop(antixtransfer::control_message *msg) {
	// TODO

	// XXX must send a response
	antix::send_blank(control_rep_sock);
}

/*
	Service control messages from clients
*/
void
service_control_messages() {
	antixtransfer::control_message msg;
	while (antix::recv_pb(control_rep_sock, &msg, ZMQ_NOBLOCK) == 1) {
		cout << "Received a client control message: " << msg.type() << endl;
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

/*
	Send our master a message stating we're done
	Then wait until master contacts us so that all nodes are in sync
*/
void
wait_for_next_turn() {
	antixtransfer::node_master_done done_msg;
	done_msg.set_my_id( my_id );
	antix::send_pb_envelope(master_req_sock, &done_msg, "done");
	cout << "Sent done signal to master" << endl;
	antix::recv_blank(master_req_sock);
	cout << "Received begin turn signal from master" << endl;
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);
	
	if (argc != 5) {
		cerr << "Usage: " << argv[0] << " <IP of master> <IP to listen on> <neighbour port> <control port>" << endl;
		return -1;
	}

	master_host = string(argv[1]);
	my_ip = string(argv[2]);
	my_neighbour_port = string(argv[3]);
	my_control_port = string(argv[4]);

	// socket to announce ourselves to master on
	master_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	master_req_sock->connect(antix::make_endpoint(master_host, master_node_port));
	cout << "Connecting to master..." << endl;

	// socket to receive list of nodes on (and receive turn begin signal)
	master_publish_sock = new zmq::socket_t(context, ZMQ_SUB);
	// subscribe to all messages on this socket
	master_publish_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock->connect(antix::make_endpoint(master_host, master_publish_port));

	// send message announcing ourself. includes our own IP
	cout << "Sending master our existence notification..." << endl;

	// create & send pb msg identifying ourself
	antixtransfer::connect_init_node pb_init_msg;
	pb_init_msg.set_ip_addr(my_ip);
	pb_init_msg.set_neighbour_port(my_neighbour_port);
	pb_init_msg.set_control_port(my_control_port);
	antix::send_pb_envelope(master_req_sock, &pb_init_msg, "connect");

	// receive message back stating our unique ID
	antixtransfer::connect_init_response init_response;
	antix::recv_pb(master_req_sock, &init_response, 0);
	my_id = init_response.id();
	world_size = init_response.world_size();
	sleep_time = init_response.sleep_time();
	initial_puck_amount = init_response.puck_amount();
	vision_range = init_response.vision_range();
	cout << "We are now node id " << my_id << endl;

	// receive node list
	// blocks until master publishes list of nodes: indicates simulation begin
	antix::recv_pb(master_publish_sock, &node_list, 0);
	cout << "Received list of nodes:" << endl;
	antix::print_nodes(&node_list);

	if (node_list.node_size() < 3) {
		cout << "Error: we need at least 3 nodes. Only received " << node_list.node_size() << " node(s)." << endl;
		return -1;
	}

	// calculate our min / max x from the offset assigned to us in node_list
	offset_size = world_size / node_list.node_size();
	set_dimensions(&node_list);

	// find our left/right neighbours
	antix::set_neighbours(&left_node, &right_node, &node_list, my_id);
	cout << "Left neighbour id: " << left_node.id() << " " << left_node.ip_addr() << " neighbour port " << left_node.neighbour_port() << " control port " << left_node.control_port() << endl;
	cout << "Right neighbour id: " << right_node.id() << " " << right_node.ip_addr() << " neighbour port " << right_node.neighbour_port() << " control port " << right_node.control_port() << endl;

	// connect to both of our neighbour's REP sockets
	// we request foreign entities to this socket
	left_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	left_req_sock->connect(antix::make_endpoint(left_node.ip_addr(), left_node.neighbour_port()));

	right_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	right_req_sock->connect(antix::make_endpoint(right_node.ip_addr(), right_node.neighbour_port()));

	// open REP socket where neighbours request border entities
	neighbour_rep_sock = new zmq::socket_t(context, ZMQ_REP);
	neighbour_rep_sock->bind(antix::make_endpoint(my_ip, my_neighbour_port));

	// create REP socket that receives control messages from clients
	control_rep_sock = new zmq::socket_t(context, ZMQ_REP);
	control_rep_sock->bind(antix::make_endpoint(my_ip, my_control_port));

	// generate pucks
	generate_pucks();

	// enter main loop
	while (1) {
		// update poses for internal robots
		update_poses();

		// the following two unrelated messages are part of the same conversation:
		// movement only requires an ACK, so in this ACK we send a list of our
		// foreign entities to the move message. Simplifies communication, but
		// requires us to even send move message if its blank (which makes syncing
		// behaviour easier

		// send movement requests to neighbouring nodes (a bot moved out of range)
		send_move_messages();
		// receive move requests & respond with foreign entities, receive move responses
		exchange_foreign_entities();

		// service control messages on our REP socket
		service_control_messages();

		// XXX debug
		print_local_robots();

		// tell master we're done the work for this turn & wait for signal
		wait_for_next_turn();

		antix::sleep(sleep_time);
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
