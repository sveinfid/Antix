/*
	Node / map piece server

	The majority of the ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all
	Most notably the pub/sub message enveloping example:
	https://github.com/imatix/zguide/blob/master/examples/C++/psenvpub.cpp
	https://github.com/imatix/zguide/blob/master/examples/C++/psenvsub.cpp
*/

#include "map.cpp"

using namespace std;

string master_host;
string master_node_port = "7770";
string master_publish_port = "7773";

string ipc_fname_prefix = "/tmp/node";
string ipc_id;

string my_ip;
string my_neighbour_port;
string my_gui_port;

int my_id;
int sleep_time;
int total_teams;

Map *my_map;

// robots & pucks near our borders that we send to our neighbours
antixtransfer::SendMap border_map_left;
antixtransfer::SendMap border_map_right;

antixtransfer::Node_list node_list;
antixtransfer::Node_list::Node left_node;
antixtransfer::Node_list::Node right_node;

/*
	Repeated protobuf messages: Declare them once as constructors are expensive
*/
// messages that are sent repeatedly when bots move left or right
antixtransfer::move_bot move_left_msg;
antixtransfer::move_bot move_right_msg;
// used in update_foreign_entities
antixtransfer::SendMap sendmap_recv;
// used in exchange_foreign_entities
antixtransfer::move_bot move_bot_msg;

// Connect to master & identify ourselves. Get state
zmq::socket_t *master_req_sock;
// Master publishes list of nodes to us when beginning simulation
zmq::socket_t *master_sub_sock;

// request border entities from neighbour on these REQ sockets
zmq::socket_t *right_req_sock;
zmq::socket_t *left_req_sock;
// handle neighbours requesting border entities on this REP sock (neighbour port)
zmq::socket_t *neighbour_rep_sock;

// clients request commands on this sock;
zmq::socket_t *control_rep_sock;
// clients send done message on this sock
zmq::socket_t *sync_rep_sock;
// send clients begin on this sock
zmq::socket_t *sync_pub_sock;

// gui requests entities on this sock
zmq::socket_t *gui_rep_sock;

/*
	Find our starting x offset
*/
double
find_map_offset(antixtransfer::Node_list *node_list) {
	antixtransfer::Node_list::Node *node;

	for (int i = 0; i < node_list->node_size(); i++) {
		node = node_list->mutable_node(i);
		if (node->id() == my_id) {
			return node->x_offset();
		}
	}
	cerr << "Error: didn't find my offset!" << endl;
	exit(-1);
}

/*
	A map is waiting to be read on given sock
	The map may or may not contain entities. Add any entities therein
	to our internal records of foreign robots & pucks
*/
void
update_foreign_entities(zmq::socket_t *sock) {
	antix::recv_pb(sock, &sendmap_recv, 0);

	// foreign robots
	for (int i = 0; i < sendmap_recv.robot_size(); i++) {
		my_map->add_foreign_robot(sendmap_recv.robot(i).x(), sendmap_recv.robot(i).y(), sendmap_recv.robot(i).id(), sendmap_recv.robot(i).team());
	}
	// foreign pucks
	for (int i = 0; i < sendmap_recv.puck_size(); i++) {
		my_map->add_foreign_puck(sendmap_recv.puck(i).x(), sendmap_recv.puck(i).y(), sendmap_recv.puck(i).held() );
	}
}

/*
	We know a node has sent a move request message
	Read it and add all the robots in the message to our local robot listing
*/
void
handle_move_request(antixtransfer::move_bot *move_bot_msg) {
	// for each robot in the message, add it to our list
	int i;
	for(i = 0; i < move_bot_msg->robot_size(); i++) {
		my_map->add_robot(move_bot_msg->robot(i).x(), move_bot_msg->robot(i).y(),
			move_bot_msg->robot(i).id(), move_bot_msg->robot(i).team(),
			move_bot_msg->robot(i).a(), move_bot_msg->robot(i).v(),
			move_bot_msg->robot(i).w(), move_bot_msg->robot(i).has_puck(),
			move_bot_msg->robot(i).last_x(), move_bot_msg->robot(i).last_y()
		);
	}
#if DEBUG
	cout << i << " robots transferred to this node." << endl;
#endif
}

/*
	Move any of our robots to neighbours if necessary
	Receive any of the same

	Each neighbour must send us one request (even if blank), and we must send one
	request (even if blank)

	How the move / move response / foreign entities conversation goes:
	the following two unrelated messages are part of the same conversation:
	movement only requires an ACK, so in this ACK we send a list of our
	foreign entities to the move message. Simplifies communication, but
	requires us to even send move message if it's blank (which makes syncing
	behaviour easier)
*/
void
send_move_messages() {
	// First we build our own move messages to be sent to our neighbours
	my_map->build_move_message(&move_left_msg, &move_right_msg);

	// Send our move messages
	antix::send_pb(left_req_sock, &move_left_msg);
	antix::send_pb(right_req_sock, &move_right_msg);

#if DEBUG
	cout << "Movement messages sent." << endl;
#endif
}

/*
	Send our foreign/border entities to our 2 neighbours
	Receive the same from each neighbour
*/
void
exchange_foreign_entities() {
	// first we re-calculate what entities local to us are near borders
	my_map->rebuild_border_entities(&border_map_left, &border_map_right);

	// We wait for any requests (move requests in this case), to which we respond
	// by giving the requester a list of our foreign entities
	// We also wait for a response from our earlier move requests
	zmq::pollitem_t items [] = {
		{ *left_req_sock, 0, ZMQ_POLLIN, 0 },
		{ *right_req_sock, 0, ZMQ_POLLIN, 0},
		{ *neighbour_rep_sock, 0, ZMQ_POLLIN, 0}
	};
	// Both of these must be 2 before we continue (hear from both neighbour nodes)
	int responses = 0;
	int requests = 0;
#if DEBUG
	cout << "Sync: Waiting for responses in exchange_foreign_entities()" << endl;
#endif
	// Keep waiting for messages until we've received the number we expect
	while (responses < 2 || requests < 2) {
		zmq::poll(&items [0], 3, -1);

		// left_req response
		if (items[0].revents & ZMQ_POLLIN) {
#if DEBUG
			cout << "Received foreign entities response from left req sock" << endl;
#endif
			update_foreign_entities(left_req_sock);
			responses++;
		}

		// right_req response
		if (items[1].revents & ZMQ_POLLIN) {
#if DEBUG
			cout << "Received foreign entities response from right req sock" << endl;
#endif
			update_foreign_entities(right_req_sock);
			responses++;
		}

		// neighbour request
		if (items[2].revents & ZMQ_POLLIN) {
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
#if DEBUG
			cout << "Received move request from a neighbour, sent border entities" << endl;
#endif
		}
	}
#if DEBUG
	cout << "Sync: done exchange_foreign_entities" << endl;
#endif
}

/*
	For each robot in the message from a client, apply the action
*/
void
parse_client_message(antixtransfer::control_message *msg) {
	Robot *r;
	for (int i = 0; i < msg->robot_size(); i++) {
		r = my_map->find_robot(msg->team(), msg->robot(i).id());
		if (r == NULL) {
			cerr << "Error: got a control message for a robot I couldn't find!" << endl;
			continue;
		}

		if (msg->robot(i).type() == antixtransfer::control_message::SETSPEED) {
			r->setspeed(msg->robot(i).v(), msg->robot(i).w(), msg->robot(i).last_x(), msg->robot(i).last_y());
#if DEBUG
			cout << "(SETSPEED) Got last x " << r->last_x << " and last y " << r->last_y << " from client on turn " << antix::turn << endl;
#endif

		} else if (msg->robot(i).type() == antixtransfer::control_message::PICKUP) {
			r->pickup(&my_map->pucks, msg->robot(i).last_x(), msg->robot(i).last_y());
#if DEBUG
			cout << "(PICKUP) Got last x " << r->last_x << " and last y " << r->last_y << " from client on turn " << antix::turn << endl;
#endif

		} else if (msg->robot(i).type() == antixtransfer::control_message::DROP) {
			r->drop(&my_map->pucks);

		} else {
			cerr << "Error: Unknown message type from client (parse_client_message())" << endl;
			exit(-1);
		}
	}
}

/*
	Service control messages from clients

	XXX Currently each robot on a team only gets one move per turn
	Move = setspeed OR pickup OR drop
*/
void
service_control_messages() {
#if DEBUG
	cout << "Sync: Waiting for control requests from clients..." << endl;
#endif

	// Each client will have one sense request message, and possibly one control msg
	// Though they are the same protobuf type, the sense request
	// message will have 0 robots in it to differentiate between them

	// The number of messages we expect is:
	// - sense_messages: N = # of clients in the world
	// - control messages: M = # of teams we are currently holding robots for
	//   - we know this through sense_map.size()
	for (int i = 0; i < total_teams + my_map->sense_map.size(); i++) {
		// XXX declare this once?
		antixtransfer::control_message msg;
		antix::recv_pb(control_rep_sock, &msg, 0);

		// If more than 0 robots given, this is a message indicating commands for robots
		if (msg.robot_size() > 0) {
#if DEBUG
			cout << "Got a command message for team " << msg.team() << " with commands for " << msg.robot_size() << " robots." << endl;
#endif
			parse_client_message(&msg);
			// no confirmation or anything (for now)
			antix::send_blank(control_rep_sock);

		// Otherwise it's a sense request message
		} else {
			// if we have sense data for that team, send it on
			if (my_map->sense_map.count( msg.team() ) > 0) {
#if DEBUG
				cout << "Sending sense data for team " << msg.team() << " with " << my_map->sense_map[msg.team()]->robot_size() << " robots " << endl;
#endif
				antix::send_pb(control_rep_sock, my_map->sense_map[msg.team()]);

			// otherwise give a blank (no robots) sense message
			} else {
				// XXX declare once
				antixtransfer::sense_data blank_sense_msg;
				antix::send_pb(control_rep_sock, &blank_sense_msg);
#if DEBUG
				cout << "Sending sense data for team " << msg.team() << " with 0 robots (BLANK)" << endl;
#endif
			}
		}
	}

#if DEBUG
	cout << "Sync: Done responding to client control messages." << endl;
#endif
}

/*
	Respond to at most one GUI request per turn
*/
void
service_gui_requests() {
#if DEBUG
	cout << "Sync: Checking GUI requests..." << endl;
#endif
	antix::recv_blank(gui_rep_sock);
	// Respond by sending a list of our entities
	// XXX declare only once
	antixtransfer::SendMap_GUI gui_map;
	my_map->build_gui_map(&gui_map);
	antix::send_pb(gui_rep_sock, &gui_map);
#if DEBUG
	cout << "Sync: Sent GUI response." << endl;
#endif
}

/*
	Block until a client sends a message stating it is done. Only return
	once all clients have been heard from
*/
void
wait_for_clients() {
	// Every client process on the machine must contact us before beginning next turn
	// XXX declare only once
	set<int> clients_done;
#if DEBUG
	cout << "Sync: Waiting for clients..." << endl;
#endif
	while (clients_done.size() < total_teams) {
		string type = antix::recv_str(sync_rep_sock);

		// XXX declare only once
		antixtransfer::done done_msg;
		antix::recv_pb(sync_rep_sock, &done_msg, 0);

		// respond since rep sock
		antix::send_blank(sync_rep_sock);

		// if haven't yet heard from this client
		if (clients_done.count( done_msg.my_id() ) == 0) {
			clients_done.insert(done_msg.my_id());
		}
#if DEBUG
		cout << "Sync: Just received done from client " << done_msg.my_id() << endl;
		cout << "Sync: Heard done from " << clients_done.size() << " clients. There are " << total_teams << " teams." << endl;
#endif
	}
#if DEBUG
	cout << "Sync: Heard from all clients." << endl;
#endif
}

/*
	Send message to clients to begin next turn
*/
void
begin_clients() {
	antix::send_blank(sync_pub_sock);
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);
	srand( time(NULL) );
	srand48( time(NULL) );
	
	if (argc != 6) {
		cerr << "Usage: " << argv[0] << " <IP of master> <IP to listen on> <neighbour port> <GUI port> <IPC ID # (unique to this computer)>" << endl;
		return -1;
	}

	master_host = string(argv[1]);
	my_ip = string(argv[2]);
	my_neighbour_port = string(argv[3]);
	my_gui_port = string(argv[4]);
	ipc_id = string(argv[5]);

	// initialize move messages: only needs to be done once, so may as well do it here
	move_left_msg.set_from_right(true);
	move_right_msg.set_from_right(false);

	// socket to announce ourselves to master on
	master_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	master_req_sock->connect(antix::make_endpoint(master_host, master_node_port));
	cout << "Connecting to master..." << endl;

	// socket to receive list of nodes on (and receive turn begin signal)
	master_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
	// subscribe to all messages on this socket
	master_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_sub_sock->connect(antix::make_endpoint(master_host, master_publish_port));

	// send message announcing ourself. includes our own IP
	cout << "Sending master our existence notification..." << endl;

	// create & send pb msg identifying ourself
	antixtransfer::connect_init_node pb_init_msg;
	pb_init_msg.set_ip_addr(my_ip);
	pb_init_msg.set_neighbour_port(my_neighbour_port);
	pb_init_msg.set_gui_port(my_gui_port);
	antix::send_pb_envelope(master_req_sock, &pb_init_msg, "connect");

	// receive message back stating our unique ID & simulation settings
	antixtransfer::connect_init_response init_response;
	antix::recv_pb(master_req_sock, &init_response, 0);
	my_id = init_response.id();
	sleep_time = init_response.sleep_time();
	int initial_puck_amount = init_response.puck_amount();
	antix::world_size = init_response.world_size();
	Robot::vision_range = init_response.vision_range();
	Robot::fov = init_response.fov();
	Robot::pickup_range = init_response.pickup_range();

	antix::matrix_width = floor(antix::world_size / Robot::vision_range);

	cout << "We are now node ID " << my_id << endl;

	// sync rep sock which receives done messages from clients
	sync_rep_sock = new zmq::socket_t(context, ZMQ_REP);
	sync_rep_sock->bind(antix::make_endpoint_ipc(ipc_fname_prefix + ipc_id + "r"));

	// sync pub sock which sends begin message to clients
	sync_pub_sock = new zmq::socket_t(context, ZMQ_PUB);
	sync_pub_sock->bind(antix::make_endpoint_ipc(ipc_fname_prefix + ipc_id + "p"));

	// receive node list
	// blocks until master publishes list of nodes: indicates simulation begin
	antix::recv_pb(master_sub_sock, &node_list, 0);
	cout << "Received list of nodes:" << endl;
	antix::print_nodes(&node_list);

	if (node_list.node_size() < 3) {
		cout << "Error: we need at least 3 nodes. Only received " << node_list.node_size() << " node(s)." << endl;
		return -1;
	}

	// calculate our min / max x from the offset assigned to us in node_list
	antix::offset_size = antix::world_size / node_list.node_size();

	// Initialize map object
	my_map = new Map( find_map_offset(&node_list), &node_list, initial_puck_amount, my_id);

	// Get number of teams
	total_teams = node_list.robots_on_node_size();
#if DEBUG
	cout << "Total teams: " << total_teams << endl;
#endif

	// find our left/right neighbours
	antix::set_neighbours(&left_node, &right_node, &node_list, my_id);

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
	control_rep_sock->bind(antix::make_endpoint_ipc(ipc_fname_prefix + ipc_id + "c"));

	// create REP socket that receives queries from GUI
	gui_rep_sock = new zmq::socket_t(context, ZMQ_REP);
	gui_rep_sock->bind(antix::make_endpoint(my_ip, my_gui_port));

	// enter main loop
	while (1) {
		// update poses for internal robots
		my_map->update_poses();

		// send movement requests to neighbouring nodes (a bot moved out of range)
		send_move_messages();
		// receive move requests & respond with foreign entities, receive move responses
		exchange_foreign_entities();

		// build message for each client of what their robots can see
		my_map->build_sense_messages();
		
#if DEBUG
		my_map->print_local_robots();
#endif

		// service control messages on our REP socket
		service_control_messages();

		// service GUI entity requests
#if GUI
		service_gui_requests();
#endif

		// wait for all clients to be done
		wait_for_clients();

		// tell master we're done the work for this turn & wait for signal
		antix::wait_for_next_turn(master_req_sock, master_sub_sock, my_id, antixtransfer::done::NODE);

		// tell clients to begin
		begin_clients();
#if DEBUG
		cout << "Turn " << antix::turn++ << " done." << endl;
#endif

#if SLEEP
		antix::sleep(sleep_time);
#endif
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
