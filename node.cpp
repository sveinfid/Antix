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

string ipc_id;

string my_ip;
string my_neighbour_port;
string my_gui_port;

int my_id;
int sleep_time;
int total_teams;

Map *my_map;

// robots & pucks near our borders that we send/recv with our neighbours
antixtransfer::SendMap crit_map;

antixtransfer::Node_list node_list;
antixtransfer::Node_list::Node left_node;
antixtransfer::Node_list::Node right_node;

/*
	Repeated protobuf message objects / other objects
	Declare them once as constructors are expensive
*/
// used in update_foreign_entities
antixtransfer::SendMap sendmap_recv;
// used in exchange_foreign_entities
antixtransfer::move_bot move_bot_msg;
// used in service_control_messages
antixtransfer::control_message control_msg;
antixtransfer::sense_data blank_sense_msg;
// used in service_gui_requests
antixtransfer::SendMap_GUI gui_map;
antixtransfer::GUI_Request gui_req;
// used in wait_for_clients
antixtransfer::done done_msg;
set<int> clients_done;
// used for wait_for_next_turn()
antixtransfer::done master_done_msg;

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
	Wait until we hear from total_teams unique client connections
	Add their data to pb_init_msg
*/
void
wait_on_initial_clients(antixtransfer::connect_init_node *pb_init_msg) {
	set<int> heard_clients;
	antixtransfer::connect_init_client init_client;
	while (heard_clients.size() < total_teams) {
		int rc = antix::recv_pb(sync_rep_sock, &init_client, 0);
		assert(rc == 1);
		// Blank since nothing to tell the client yet
		antix::send_blank(sync_rep_sock);
		// haven't yet heard
		if (heard_clients.count( init_client.id() ) == 0) {
			heard_clients.insert( init_client.id() );
			antixtransfer::connect_init_node::Team *team = pb_init_msg->add_team();
			team->set_id( init_client.id() );
			team->set_num_robots( init_client.num_robots() );
			cout << "Start up: Got initialization message from client " << init_client.id();
			cout << ". Expecting " << total_teams - heard_clients.size() << " more." << endl;
		// have already heard. Shouldn't ever happen...
		} else {
			cerr << "Error: heard duplicate init message from a client" << endl;
			exit(-1);
		}
	}
}

/*
	Continue to send messages on our pub sock until all clients have told us
	that they have heard us
*/
void
synchronise_clients() {
	set<int> heard_clients;
	antixtransfer::node_master_sync sync_msg;
	while (heard_clients.size() < total_teams) {
		antix::send_blank_envelope(sync_pub_sock, "cli_sync");
		// if hear a message
		if (antix::recv_pb(sync_rep_sock, &sync_msg, ZMQ_NOBLOCK) == 1) {
			// since rep sock, must respond
			antix::send_blank(sync_rep_sock);
			if (heard_clients.count( sync_msg.my_id() ) == 0) {
				heard_clients.insert( sync_msg.my_id() );
				cout << "Client with team " << sync_msg.my_id() << " synchronised. ";
				cout << heard_clients.size() << " sync'd, expecting " << total_teams << endl;
			}
		}
		antix::sleep(100);
	}
}

/*
	PUB/SUB can be desynchronised after initial connection. Synchronise them.

	Listen on sub sock until we hear a message
	Make a request on req_sock after we have heard one
*/
void
synchronize_sub_sock(zmq::socket_t *sub_sock, zmq::socket_t *req_sock) {
	antixtransfer::node_master_sync sync_msg;
	sync_msg.set_my_id(my_id);

	antix::recv_blank(sub_sock);

	antix::send_pb_envelope(req_sock, &sync_msg, "sync");
	antix::recv_blank(req_sock);
}

/*
	Take the init_response message we were given from master and add to it
	a list of homes from node_list message.

	Then send init_response to our local clients
*/
void
initial_begin_clients(antixtransfer::connect_init_response *init_response,
	antixtransfer::Node_list *node_list) {
	for (int i = 0; i < node_list->home_size(); i++) {
		antixtransfer::connect_init_response::Home *h = init_response->add_home();
		h->set_team( node_list->home(i).team() );
		h->set_x( node_list->home(i).x() );
		h->set_y( node_list->home(i).y() );
	}
	antix::send_pb_envelope(sync_pub_sock, init_response, "cli_begin");
	cout << "Start up: Sent simulation parameters to clients." << endl;
}

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
	int rc = antix::recv_pb(sock, &sendmap_recv, 0);
	assert(rc == 1);

	// foreign robots
	int robot_size = sendmap_recv.robot_size();
	for (int i = 0; i < robot_size; i++) {
		my_map->add_foreign_robot(sendmap_recv.robot(i).x(), sendmap_recv.robot(i).y(), sendmap_recv.robot(i).id(), sendmap_recv.robot(i).team());
	}
	// foreign pucks
	int puck_size = sendmap_recv.puck_size();
	for (int i = 0; i < puck_size; i++) {
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
	int robot_size = move_bot_msg->robot_size();
	Robot *r;
	for(i = 0; i < robot_size; i++) {
		r = my_map->add_robot(move_bot_msg->robot(i).x(), move_bot_msg->robot(i).y(),
			move_bot_msg->robot(i).id(), move_bot_msg->robot(i).team(),
			move_bot_msg->robot(i).a(), move_bot_msg->robot(i).v(),
			move_bot_msg->robot(i).w(), move_bot_msg->robot(i).has_puck(),
			move_bot_msg->robot(i).last_x(), move_bot_msg->robot(i).last_y()
		);

		assert( r != NULL );

		r->sensor_bbox.x.min = move_bot_msg->robot(i).bbox_x_min();
		r->sensor_bbox.x.max = move_bot_msg->robot(i).bbox_x_max();
		r->sensor_bbox.y.min = move_bot_msg->robot(i).bbox_y_min();
		r->sensor_bbox.y.max = move_bot_msg->robot(i).bbox_y_max();

		int ints_size = move_bot_msg->robot(i).ints_size();
		for (int j = 0; j < ints_size; j++)
			r->ints.push_back( move_bot_msg->robot(i).ints(j) );
		int doubles_size = move_bot_msg->robot(i).doubles_size();
		for (int j = 0; j < doubles_size; j++)
			r->doubles.push_back( move_bot_msg->robot(i).doubles(j) );
	}
#if DEBUG
	cout << i+1 << " robots in move message." << endl;
#endif
}

/*
	Do the handshake with both of our neighbours to agree on the state
	of the critical sections (those sections within sight distance of border).

	We also deal with robots moving between nodes here.

	This is done as follows:
	- After poses are updated for all robots that were previously not in the
	  critical sections, we request a list of robots in the left neighbour's
	  right critical section.
	- The left neighbour responds by sending us this list
	- We update the poses of our robots in our left critical section (that have
	  not already been moved, such as those that moved into the critical section
	  during this turn)
	- We make another request to our left neighbour. This request contains a list
	  of robots we are moving to it, and all of our robots currently in our left
	  critical section.
	- The left neighbour responds by first updating the poses of its robots in
	  its right critical section, and sending back the robots it needs to move
	  to us, and its updated robot list of those in its right critical section.

	Thus we must send the requests to the left neighbour, and respond to those
	requests that will be sent from our right neighbour to us.
*/
void
neighbours_handshake() {
	// Ask our left neighbour to send us its robots in its right critical section
	antix::send_blank(left_req_sock);

	// Now we wait for the response from our left neighbour, and for requests
	// from our right neighbour asking its left neighbour (us)
	zmq::pollitem_t items [] = {
		{ *left_req_sock, 0, ZMQ_POLLIN, 0 },
		{ *neighbour_rep_sock, 0, ZMQ_POLLIN, 0}
	};

	// A complete handshake has both of these as 2
	int left_responses_heard = 0;
	int right_requests_heard = 0;

	int rc;
	while ( left_responses_heard < 2 || right_requests_heard < 2 ) {
		zmq::poll(&items [0], 2, -1);

		// response from our left neighbour
		if (items[0].revents & ZMQ_POLLIN) {
			// If it's the first response, it contains a list of robots in their
			// right critical section
			if (left_responses_heard == 0) {
				antix::recv_pb(left_req_sock, &sendmap_recv, 0);
				my_map->update_left_crit_region(&sendmap_recv, &move_bot_msg, &crit_map);

				// Initiate new request
				// Send move message
				antix::send_pb_flags(left_req_sock, &move_bot_msg, ZMQ_SNDMORE);
				// And send the robots in our left critical section
				antix::send_pb_flags(left_req_sock, &crit_map, 0);
			}

			// Second response contains robots that move to this node
			// and updated positions of robots in the critical section
			else if (left_responses_heard == 1) {
				// moved bots
				antix::recv_pb(left_req_sock, &move_bot_msg, 0);
				// and robot positions in its right critical section
				antix::recv_pb(left_req_sock, &crit_map, 0);

				handle_move_request(&move_bot_msg);
				//cout << "Add critical region robots in left neighbour response" << endl;
				my_map->add_critical_region_robots(&crit_map);
			}

			else {
				// Should never get here
				assert( 1 == 0 );
			}

			left_responses_heard++;
		}

		// right neighbour sent us a request
		if (items[1].revents & ZMQ_POLLIN) {
			// If it's the first request, send the robots in our right crit section
			if (right_requests_heard == 0) {
				antix::recv_blank(neighbour_rep_sock);
				my_map->build_right_crit_map(&crit_map);
				antix::send_pb_flags(neighbour_rep_sock, &crit_map, 0);
			}

			// The second request contains robots to move to this node
			// and updated positions of robots in the critical section
			// add the robots and update the positions of our robots
			else if (right_requests_heard == 1) {
				antix::recv_pb(neighbour_rep_sock, &move_bot_msg, 0);
				antix::recv_pb(neighbour_rep_sock, &crit_map, 0);

				handle_move_request(&move_bot_msg);
				//cout << "Add critical region robots in right neighbour request" << endl;
				my_map->add_critical_region_robots(&crit_map);

				// Update poses for our robots
				my_map->update_right_crit_region(&move_bot_msg, &crit_map);

				// Respond by sending a list of all the robots in our right crit region
				// and our bots to move to that node
				antix::send_pb_flags(neighbour_rep_sock, &move_bot_msg, ZMQ_SNDMORE);
				antix::send_pb_flags(neighbour_rep_sock, &crit_map, 0);
			}

			else {
				assert( 1 == 0 );
			}

			right_requests_heard++;
		}
	}
}

/*
	For each robot in the message from a client, apply the action
*/
void
parse_client_message(antixtransfer::control_message *msg) {
	Robot *r;
	int robot_size = msg->robot_size();
	for (int i = 0; i < robot_size; i++) {
		r = my_map->find_robot(msg->team(), msg->robot(i).id());
		assert(r != NULL);

		if (msg->robot(i).puck_action() == antixtransfer::control_message::PICKUP) {
			r->pickup(&my_map->pucks);
#if DEBUG
			cout << "(PICKUP) Got last x " << r->last_x << " and last y " << r->last_y << " from client on turn " << antix::turn << endl;
#endif
		} else if (msg->robot(i).puck_action() == antixtransfer::control_message::DROP) {
			r->drop(&my_map->pucks, &my_map->local_homes);
#if DEBUG
			cout << "Puck dropped on turn " << antix::turn << endl;
#endif
		}

		// Always set speed
		r->setspeed(msg->robot(i).v(), msg->robot(i).w(), msg->robot(i).last_x(), msg->robot(i).last_y());
#if DEBUG
		cout << "(SETSPEED) Got last x " << r->last_x << " and last y " << r->last_y << " from client on turn " << antix::turn << endl;
#endif

		// Always update robot's individual memory
		r->ints.clear();
		int ints_size = msg->robot(i).ints_size();
		for (int j = 0; j < ints_size; j++)
			r->ints.push_back( msg->robot(i).ints(j) );
		r->doubles.clear();
		int doubles_size = msg->robot(i).doubles_size();
		for (int j = 0; j < doubles_size; j++)
			r->doubles.push_back( msg->robot(i).doubles(j) );
	}
}

/*
	Service control messages from clients
*/
void
service_control_messages() {
#if DEBUG_SYNC
	cout << "Sync: Waiting for control requests from clients..." << endl;
#endif

	// Each client will have one sense request message, and possibly one control msg
	// Though they are the same protobuf type, the sense request
	// message will have 0 robots in it to differentiate between them

	// The number of messages we expect is:
	// - sense_messages: N = # of clients in the world
	// - control messages: M = # of teams we are currently holding robots for
	//   - we know this through sense_map.size()
	int rc;
	int expected_messages = total_teams + my_map->sense_map.size();
	for (int i = 0; i < expected_messages; i++) {
		rc = antix::recv_pb(control_rep_sock, &control_msg, 0);
		assert(rc == 1);

		// If > 0 robots given, this is a message indicating commands for robots
		if (control_msg.robot_size() > 0) {
#if DEBUG
			cout << "Got a command message for team " << control_msg.team() << " with commands for " << control_msg.robot_size() << " robots." << endl;
#endif
			parse_client_message(&control_msg);
			// no confirmation or anything (for now)
			antix::send_blank(control_rep_sock);

		// Otherwise it's a sense request message
		} else {
			// if we have sense data for that team, send it on
			if (my_map->sense_map.count( control_msg.team() ) > 0) {
#if DEBUG
				cout << "Sending sense data for team " << control_msg.team();
				cout << " with " << my_map->sense_map[control_msg.team()]->robot_size();
				cout << " robots " << endl;
#endif
				antix::send_pb(control_rep_sock, my_map->sense_map[control_msg.team()]);

			// otherwise give a blank (no robots) sense message
			} else {
				antix::send_pb(control_rep_sock, &blank_sense_msg);
#if DEBUG
				cout << "Sending sense data for team " << control_msg.team();
				cout << " with 0 robots (BLANK)" << endl;
#endif
			}
		}
	}

#if DEBUG_SYNC
	cout << "Sync: Done responding to client control messages." << endl;
#endif
}

/*
	Respond to at most one GUI request per turn
*/
void
service_gui_requests() {
#if DEBUG_SYNC
	cout << "Sync: Checking GUI requests..." << endl;
#endif
	int rc = antix::recv_pb(gui_rep_sock, &gui_req, 0);
	assert(rc == 1);
	
	//only sent map if GUI request for it
	if(gui_req.r()){
		// Respond by sending a list of our entities
		my_map->build_gui_map(&gui_map);
		antix::send_pb(gui_rep_sock, &gui_map);
	}else {
		antix::send_blank(gui_rep_sock);
	}
#if DEBUG_SYNC
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
	clients_done.clear();

#if DEBUG_SYNC
	cout << "Sync: Waiting for clients..." << endl;
#endif
	string type;
	int rc;
	while (clients_done.size() < total_teams) {
		type = antix::recv_str(sync_rep_sock);

		rc = antix::recv_pb(sync_rep_sock, &done_msg, 0);
		assert(rc == 1);

		// respond since rep sock
		antix::send_blank(sync_rep_sock);

		// if haven't yet heard from this client
		//if (clients_done.count( done_msg.my_id() ) == 0) {
			clients_done.insert(done_msg.my_id());
		//}
#if DEBUG_SYNC
		cout << "Sync: Just received done from client " << done_msg.my_id() << endl;
		cout << "Sync: Heard done from " << clients_done.size();
		cout << " clients. There are " << total_teams << " teams." << endl;
#endif
	}
#if DEBUG_SYNC
	cout << "Sync: Heard from all clients." << endl;
#endif
}

/*
	Every TURNS_SEND_SCORE turns, add all of our recorded scores for teams to
	our done message before we send it
	Then clear our scores
*/
void
update_scores_to_send(antixtransfer::done *done_msg) {
	const int rem = antix::turn % TURNS_SEND_SCORE;

	// send them on every TURNS_SEND_SCORE turns
	if (rem == 0) {
		const vector<Home *>::const_iterator homes_end = my_map->local_homes.end();
		for (vector<Home *>::const_iterator it = my_map->local_homes.begin(); it != homes_end; it++) {
			antixtransfer::done::Score *score = done_msg->add_scores();
			score->set_team_id( (*it)->team );
			score->set_score( (*it)->score );
			(*it)->score = 0;
		}

	// clear on TURNS_SEND_SCORE + 1
	} else if (rem == 1) {
		done_msg->clear_scores();
	}
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
	
	if (argc != 7) {
		cerr << "Usage: " << argv[0] << " <IP of master> <IP to listen on> <neighbour port> <GUI port> <IPC ID # (unique to this computer)> <number of teams>" << endl;
		return -1;
	}

	master_host = string(argv[1]);
	my_ip = string(argv[2]);
	my_neighbour_port = string(argv[3]);
	my_gui_port = string(argv[4]);
	ipc_id = string(argv[5]);
	total_teams = atoi(argv[6]);
	assert(total_teams > 0);

	// socket to announce ourselves to master on
	while (1) {
		try {
			master_req_sock = new zmq::socket_t(context, ZMQ_REQ);
		} catch (zmq::error_t e) {
			cout << "Error: Master req sock new: " << e.what() << endl;
			delete master_req_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			master_req_sock->connect(antix::make_endpoint(master_host, master_node_port));
		} catch (zmq::error_t e) {
			cout << "Error: Master req sock connect: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// socket to receive list of nodes on (and receive turn begin signal)
	while (1) {
		try {
			master_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
		} catch (zmq::error_t e) {
			cout << "Error: Master sub sock new: " << e.what() << endl;
			delete master_sub_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			// subscribe to all messages on this socket
			master_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
		} catch (zmq::error_t e) {
			cout << "Error: Master sub sock set subs: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			master_sub_sock->connect(antix::make_endpoint(master_host, master_publish_port));
		} catch (zmq::error_t e) {
			cout << "Error: Master sub sock connect: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	string ipc_fname_prefix = IPC_PREFIX;
	cout << "Using IPC prefix " << ipc_fname_prefix << endl;

	// sync rep sock which receives done messages from clients
	while (1) {
		try {
			sync_rep_sock = new zmq::socket_t(context, ZMQ_REP);
		} catch (zmq::error_t e) {
			cout << "Error: Sync rep sock new: " << e.what() << endl;
			delete sync_rep_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			sync_rep_sock->bind(antix::make_endpoint_ipc(ipc_fname_prefix + ipc_id + "r"));
		} catch (zmq::error_t e) {
			cout << "Error: Sync rep sock bind: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// sync pub sock which sends begin message to clients
	while (1) {
		try {
			sync_pub_sock = new zmq::socket_t(context, ZMQ_PUB);
		} catch (zmq::error_t e) {
			cout << "Error: Sync pub sock new: " << e.what() << endl;
			delete sync_pub_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			sync_pub_sock->bind(antix::make_endpoint_ipc(ipc_fname_prefix + ipc_id + "p"));
		} catch (zmq::error_t e) {
			cout << "Error: Sync pub sock bind: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// rep socket that receives control messages from clients
	while (1) {
		try {
			control_rep_sock = new zmq::socket_t(context, ZMQ_REP);
		} catch (zmq::error_t e) {
			cout << "Error: Control rep sock new: " << e.what() << endl;
			delete control_rep_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			control_rep_sock->bind(antix::make_endpoint_ipc(ipc_fname_prefix + ipc_id + "c"));
		} catch (zmq::error_t e) {
			cout << "Error: Control rep sock bind: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	cout << "Waiting for connection from " << total_teams << " teams." << endl;

	// Build message we will send to master
	antixtransfer::connect_init_node pb_init_msg;
	pb_init_msg.set_ip_addr(my_ip);
	pb_init_msg.set_neighbour_port(my_neighbour_port);
	pb_init_msg.set_gui_port(my_gui_port);

	// this message also includes data on our teams: wait for teams to connect & set this
	wait_on_initial_clients(&pb_init_msg);

	// Make sure all clients can hear messages on our PUB sock before continuing
	synchronise_clients();

	// Now we connect to master & send our initialization data
	// In response we get simulation parameters, node list, home list,
	// and list of where robots are initially created

	// Send our init message announcing ourself
	cout << "Sending master our existence notification..." << endl;
	antix::send_pb_envelope(master_req_sock, &pb_init_msg, "connect");

	// receive message back stating our unique ID & the simulation settings
	antixtransfer::connect_init_response init_response;
	int rc = antix::recv_pb(master_req_sock, &init_response, 0);
	assert(rc == 1);
	my_id = init_response.id();
	sleep_time = init_response.sleep_time();
	antix::world_size = init_response.world_size();
	antix::home_radius = init_response.home_radius();
	Robot::vision_range = init_response.vision_range();
	Robot::vision_range_squared = Robot::vision_range * Robot::vision_range;
	Robot::robot_radius = init_response.robot_radius();
	Robot::fov = init_response.fov();
	Robot::pickup_range = init_response.pickup_range();

	cout << "We are now node ID " << my_id << endl;

	// need only be set once, may as well do here
	master_done_msg.set_my_id( my_id );
	master_done_msg.set_type( antixtransfer::done::NODE );

	// ZMQ pub/sub can lose initial messages: synchronize them
	cout << "Synchronizing PUB/SUB with master..." << endl;
	synchronize_sub_sock(master_sub_sock, master_req_sock);

	cout << "Waiting for master to start simulation..." << endl;

	// blocks until master publishes list of nodes: indicates simulation begin

	// should only be start
	string s;
	while (s != "start") {
		s = antix::recv_str(master_sub_sock);
	}
	cout << "Got start signal from master. Getting list of nodes..." << endl;

	// this message includes a list of homes & robot initial creation locations
	rc = antix::recv_pb(master_sub_sock, &node_list, 0);
	assert(rc == 1);
	cout << "Received list of nodes:" << endl;
	antix::print_nodes(&node_list);

	if (node_list.node_size() < 3) {
		cout << "Error: we need at least 3 nodes. Only received " << node_list.node_size() << " node(s)." << endl;
		exit(-1);
	}

	// node list has number of pucks to create
	int initial_puck_amount = node_list.initial_pucks_per_node();

	// Now that we have simulation params & home locations, pass them on to our
	// local clients
	initial_begin_clients(&init_response, &node_list);

	// calculate our min / max x from the offset assigned to us in node_list
	antix::offset_size = antix::world_size / node_list.node_size();

	//antix::matrix_height = ceil(antix::world_size / Robot::vision_range);
	antix::matrix_height = floor(antix::world_size / Robot::vision_range);
	//antix::matrix_width = ceil(antix::offset_size / Robot::vision_range);

	// Initialize map object
	my_map = new Map( find_map_offset(&node_list), &node_list, initial_puck_amount, my_id);
	antix::matrix_left_x_col = antix::Cell_x(antix::my_min_x);
	antix::matrix_right_x_col = antix::Cell_x(antix::my_min_x + antix::offset_size);
	antix::matrix_right_world_x_col = antix::Cell_x(antix::world_size);
#if DEBUG
	cout << "Matrix left x col " << antix::matrix_left_x_col << endl;
	cout << "Matrix right x col " << antix::matrix_right_x_col << endl;
	cout << "Matrix right world x col " << antix::matrix_right_world_x_col << endl;
#endif

#if DEBUG
	cout << "Total teams: " << total_teams << endl;
#endif

	// find our left/right neighbours
	antix::set_neighbours(&left_node, &right_node, &node_list, my_id);

	// connect to both of our neighbour's REP sockets
	// we request foreign entities to this socket
	while (1) {
		try {
			left_req_sock = new zmq::socket_t(context, ZMQ_REQ);
		} catch (zmq::error_t e) {
			cout << "Error: Left req sock new: " << e.what() << endl;
			delete left_req_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			left_req_sock->connect(antix::make_endpoint(left_node.ip_addr(), left_node.neighbour_port()));
		} catch (zmq::error_t e) {
			cout << "Error: Left req sock connect: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			right_req_sock = new zmq::socket_t(context, ZMQ_REQ);
		} catch (zmq::error_t e) {
			cout << "Error: Right req sock new: " << e.what() << endl;
			delete right_req_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			right_req_sock->connect(antix::make_endpoint(right_node.ip_addr(), right_node.neighbour_port()));
		} catch (zmq::error_t e) {
			cout << "Error: Right req sock connect: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// open REP socket where neighbours request border entities
	while (1) {
		try {
			neighbour_rep_sock = new zmq::socket_t(context, ZMQ_REP);
		} catch (zmq::error_t e) {
			cout << "Error: Neighbour rep sock new: " << e.what() << endl;
			delete neighbour_rep_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			neighbour_rep_sock->bind(antix::make_endpoint(my_ip, my_neighbour_port));
		} catch (zmq::error_t e) {
			cout << "Error: Neighbour rep sock bind: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// create REP socket that receives queries from GUI
	while (1) {
		try {
			gui_rep_sock = new zmq::socket_t(context, ZMQ_REP);
		} catch (zmq::error_t e) {
			cout << "Error: GUI rep sock new: " << e.what() << endl;
			delete gui_rep_sock;
			antix::sleep(1000);
			continue;
		}
		break;
	}
	while (1) {
		try {
			gui_rep_sock->bind(antix::make_endpoint(my_ip, my_gui_port));
		} catch (zmq::error_t e) {
			cout << "Error: GUI rep sock bind: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// response from master (sync message)
	string response;

	// enter main loop
	while (1) {
		// update scores: decrement lifetimes, assign scores + respawn pucks if nec
		my_map->update_scores();

		// update poses for internal robots
		my_map->update_poses();

		// Exchange robots/pucks on border, and agree on collisions near borders
		neighbours_handshake();

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

#if DEBUG_SYNC
		cout << "Sync: Sending done to master & awaiting response..." << endl;
#endif
		// tell master we're done the work for this turn & wait for signal
		update_scores_to_send(&master_done_msg);
		string response = antix::wait_for_next_turn(master_req_sock, master_sub_sock, &master_done_msg);
		if (response == "s")
			// leave loop
			break;
#ifndef NDEBUG
		else if (response == "sync") {
			cout << "Error: Got PUB/SUB sync in main loop" << endl;
			exit(-1);
		}
#endif
		assert(response == "b");
#if DEBUG_SYNC
		cout << "Sync: Received begin from master, sending begin to clients..." << endl;
#endif

		// tell clients to begin
		begin_clients();

		antix::turn++;
#if DEBUG
		cout << "Turn " << antix::turn << " done." << endl;
#endif

#if SLEEP
		antix::sleep(sleep_time);
#endif
	}

	cout << "Received shutdown message from master. Shutting down..." << endl;
	cout << "Sending shutdown message to our clients..." << endl;
	antix::send_str(sync_pub_sock, "s");

	delete my_map;

	delete master_req_sock;
	delete master_sub_sock;
	delete right_req_sock;
	delete left_req_sock;
	delete neighbour_rep_sock;
	delete control_rep_sock;
	delete sync_rep_sock;
	delete sync_pub_sock;
	delete gui_rep_sock;

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
