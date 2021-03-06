/*
	Master sets up the nodes & clients

	The majority of ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all

	Polling sockets stuff from:
	https://github.com/imatix/zguide/blob/master/examples/C++/mspoller.cpp
*/

#include <map>
#include "antix.cpp"

using namespace std;

/*
	Simulation settings
*/
// set from command line
double world_size;
int num_pucks;

const int sleep_time = 20;
// pucks per node to initially create
// range of robot sight
const double vision_range = 0.1;
// robot fov
const double fov = antix::dtor(90.0);
// radius of homes
const double home_radius = 0.1;
// radius of robot
const double robot_radius = 0.01;
const double pickup_range = vision_range / 5.0;

bool shutting_down = false;
// track whether simulation has begun
bool begun = false;

time_t start_time;

/*
	Master network settings
*/
string host;
string node_port = "7770";
string client_port = "7771";
string operator_port = "7772";
string publish_port = "7773";
string gui_port = "7774";

int next_node_id = 0;
// client_id :: robot_count
map<int, int> client_robot_map;
antixtransfer::Node_list node_list;

// in handle_done()
antixtransfer::done done_msg;

// used for synchronous turns
set<int> nodes_done;

// global scores - not updated every turn, but in general
map<int, int> scores;

/*
	Return pointer to the node with id id in the node list
*/
antixtransfer::Node_list::Node *
find_node_by_id(int id) {
	for (int i = 0; i < node_list.node_size(); i++) {
		antixtransfer::Node_list::Node *node = node_list.mutable_node(i);
		if (node->id() == id)
			return node;
	}
	return NULL;
}

/*
	Go through our list of nodes & assign an x offset to the node for which
	it is responsible to manage the map for
*/
void
set_node_offsets() {
	double offset_size = world_size / node_list.node_size();
	double position = 0;

	// First set nonsensical offset for each node to ensure we set it after
	for (int i = 0; i < node_list.node_size(); i++) {
		antixtransfer::Node_list::Node *node = node_list.mutable_node(i);
		node->set_x_offset(-1.0);
	}

	// Start from first node in the list. Make it the node with x offset 0
	// Follow its neighbours in a chain until all nodes have an offset assigned
	antixtransfer::Node_list::Node *node = node_list.mutable_node(0);
	set<int> nodes_offsets_assigned;
	while (nodes_offsets_assigned.size() != node_list.node_size()) {
		node->set_x_offset( position );
		cout << "Assigned node " << node->id() << " x offset " << node->x_offset() << endl;
		position += offset_size;

		if (nodes_offsets_assigned.count( node->id() )) {
			cout << "Already saw this node (assigning offsets)" << endl;
			cout << "Starting regardless with only " << nodes_offsets_assigned.size() << " nodes." << endl;
			break;
		}
		nodes_offsets_assigned.insert( node->id() );

		node = find_node_by_id( node->right_neighbour_id() );
		if (node == NULL) {
			cerr << "Error: Didn't find a neighbour node in set_node_offsets()" << endl;
			exit(-1);
		}
	}

	// make sure all nodes have an offset
	for (int i = 0; i < node_list.node_size(); i++) {
		antixtransfer::Node_list::Node *node = node_list.mutable_node(i);
		if (node->x_offset() == -1) {
			cerr << "Error: bad x offset on node " << node->id() << endl;
		}
		assert( node->x_offset() != -1.0 );
		//cout << "Node " << node->id() << " has x offset " << node->x_offset() << endl;
	}
}

/*
	Create one home for each client ID
	Assign each a random location
	Add it to the node_list
*/
void
setup_homes() {
	cout << "Adding homes to node list..." << endl;
	
	int num_node = node_list.node_size();
	double offset_size = world_size / node_list.node_size();
	int home_iterator = 0;
	
	for (map<int, int>::iterator it = client_robot_map.begin(); it != client_robot_map.end(); it++) {
		antixtransfer::Node_list::Home *h = node_list.add_home();
		h->set_team( it->first );
		
		// only need to spread home equally among nodes by x coordinates
		double x_low = (home_iterator%num_node) * offset_size;
		double x_high = (home_iterator%num_node + 1) * offset_size;
		
		// XXX should ensure cannot overlap
		h->set_x( antix::rand_between(x_low, x_high) );
		h->set_y( antix::rand_between(0, world_size) );
		cout << "Created home for team " << h->team() << " at (" << h->x() << ", " << h->y() << ")" << endl;
		
		home_iterator++;
	}
}

/*
	Initially we tell a node to create all of the robots for one team
	Each node may be assigned multiple teams
	Though the robots may spread out to other nodes as the game progresses
*/
void
assign_robots_to_node() {
	antixtransfer::Node_list::Robots_on_Node *rn;
	
	int num_node = node_list.node_size();
	int home_iterator = 0;
	
	for (int i = 0; i < node_list.robots_on_node_size(); i++) {
		rn = node_list.mutable_robots_on_node(i);
		// 0 to max node id
		// create robots where the home is located at
		rn->set_node(home_iterator%num_node);
		cout << "Assigned robots from team " << node_list.robots_on_node(i).team() << " to node " << node_list.robots_on_node(i).node() << endl;
		
		home_iterator++;
	}
}

/*
	We have all the nodes we are going to use in the simulation
	Assign each node's left and right neighbour intelligently
	i.e. take into account if nodes are running on the same machine by looking
	at their IPs
*/
void
set_node_neighbours() {
	// Make sure each neighbour id is -1 initially
	for (int i = 0; i < node_list.node_size(); i++) {
		node_list.mutable_node(i)->set_left_neighbour_id( -1 );
		node_list.mutable_node(i)->set_right_neighbour_id( -1 );
	}

	// Maintain a map of ip_addr to first left neighbour id in the local chain
	map<string, int> left_nodes;

	// For each node which doesn't yet have a right neighbour, go through the
	// remaining nodes and try to find one on the same local IP.
	// If find one, set it as our right neighbour
	// Do this until we have linked all local nodes to each other
	for (int i = 0; i < node_list.node_size(); i++) {
		antixtransfer::Node_list::Node *node = node_list.mutable_node(i);

		if (node->right_neighbour_id() != -1) {
			cerr << "Failure 1 in set_node_neighbours()" << endl;
			exit(-1);
		}

		// Add first node in chain on local machine to the map
		if (left_nodes.count( node->ip_addr()) == 0) {
			if (node->left_neighbour_id() != -1) {
				cerr << "Failure 2 in set_node_neighbours()" << endl;
				exit(-1);
			}
			left_nodes.insert( pair<string, int>( node->ip_addr(), node->id() ) );
		}

		// Look for nodes after this one with the same IP which do not yet have
		// a left neighbour
		for (int j = i + 1; j < node_list.node_size(); j++) {
			antixtransfer::Node_list::Node *other_node = node_list.mutable_node(j);
			// already has a left neighbour
			if (other_node->left_neighbour_id() != -1)
				continue;

			// if on same machine, link together
			if (other_node->ip_addr() == node->ip_addr()) {
				node->set_right_neighbour_id( other_node->id() );
				other_node->set_left_neighbour_id( node->id() );
				break;
			}
		}
	}

	cout << "left nodes map size " << left_nodes.size() << endl;
	for (map<string, int>::iterator it = left_nodes.begin(); it != left_nodes.end(); it++) {
		cout << "Local parent " << it->second << " has path " << endl;
		antixtransfer::Node_list::Node *node = find_node_by_id( it->second );
		while (node->right_neighbour_id() != -1) {
			cout << "\t" << node->id() << " link to " << node->right_neighbour_id() << endl;
			node = find_node_by_id( node->right_neighbour_id() );
		}
	}

	// After all local machine nodes are linked together, link the chains of
	// local nodes to each other

	// Each chain has one entry in the left_nodes map. Link its far right node to
	// the left node of next chain
	for (map<string, int>::iterator it = left_nodes.begin(); it != left_nodes.end(); it++) {
		// At first this is the far left node in the chain of local nodes
		antixtransfer::Node_list::Node *node = find_node_by_id( it->second );

		// Get the far right node in this local chain
		while (node->right_neighbour_id() != -1) {
			node = find_node_by_id( node->right_neighbour_id() );
			assert(node != NULL);
		}

		// Start after our current position in map
		map<string, int>::iterator it2 = it;
		it2++;

		// Either we link to the next node in the map, or we are the last node
		// in the map, in which case we link to the first one

		// We are last node in the map. Link to the first.
		if (it2 == left_nodes.end()) {
			it2 = left_nodes.begin();
			// Otherwise linking to the next should be acceptable
		}

		// Perform the link
		antixtransfer::Node_list::Node *other_node = find_node_by_id( it2->second );
		assert( other_node != NULL );
		assert( other_node->left_neighbour_id() == -1 );
		if (other_node == NULL || other_node->left_neighbour_id() != -1) {
			cerr << "Failure 5 in set_node_neighbours()" << endl;
			exit(-1);
		}
		node->set_right_neighbour_id( other_node->id() );
		other_node->set_left_neighbour_id( node->id() );

		if (node->right_neighbour_id() == -1) {
			cerr << "Failure 3 in set_node_neighbours()" << endl;
			exit(-1);
		}
		assert( node->right_neighbour_id() != -1 );
	}

	// After all local machine nodes are linked together, link the chains of
	// local nodes to each other
	/*
	for (int i = 0; i < node_list.node_size(); i++) {
		antixtransfer::Node_list::Node *node = node_list.mutable_node(i);
		// Node doesn't have a right neighbour
		if (node->right_neighbour_id() == -1) {
			// Find next node which doesn't have a left neighbour and link them
			for (int j = i + 1; j < node_list.node_size(); j++) {
				antixtransfer::Node_list::Node *other_node = node_list.mutable_node(j);
				if (other_node->left_neighbour_id() == -1) {
					node->set_right_neighbour_id( other_node->id() );
					other_node->set_left_neighbour_id( node->id() );
					break;
				}
			}

			// If we still didn't get a right neighbour, we must be on the far right
			// Set our right neighbour to be the far left
			if (node->right_neighbour_id() == -1) {
				antixtransfer::Node_list::Node *far_left_node = node_list.mutable_node(0);
				assert(far_left_node->left_neighbour_id() == -1);
				node->set_right_neighbour_id( far_left_node->id() );
				far_left_node->set_left_neighbour_id( node->id() );
			}
		}
	}
	*/

	// make sure all node's have a right and left neighbour id
	for (int i = 0; i < node_list.node_size(); i++) {
		antixtransfer::Node_list::Node *node = node_list.mutable_node(i);
		if (node->left_neighbour_id() == -1 || node->right_neighbour_id() == -1) {
			cerr << "Failure 4 in set_node_neighbours()" << endl;
			exit(-1);
		}
		assert(node->left_neighbour_id() != -1);
		assert(node->right_neighbour_id() != -1);
	}
	cout << "Done setting node neighbours" << endl;
}

/*
	Follow the chain of nodes, printing out their id, ip, x offsets
*/
void
output_node_offsets() {
	antixtransfer::Node_list::Node *node = node_list.mutable_node(0);
	set<int> seen_nodes;
	while (seen_nodes.size() != node_list.node_size()) { 
		seen_nodes.insert( node->id() );
		cout << "Node " << node->id() << " with IP " << node->ip_addr();
		cout << " has x offset " << node->x_offset() << endl;
		node = find_node_by_id( node->right_neighbour_id() );
	}
}

/*
	Node connected initially
	Add it to our node list and give it an ID
*/
void
handle_node_init(zmq::socket_t *nodes_socket) {
	// this message should be node giving its ip & listening port
	antixtransfer::connect_init_node init_msg;
	int rc = antix::recv_pb(nodes_socket, &init_msg, 0);
	assert(rc == 1);

	// respond with an id for the node & config info
	antixtransfer::connect_init_response init_response;
	init_response.set_id(next_node_id++);
	init_response.set_world_size(world_size);
	init_response.set_sleep_time(sleep_time);
	init_response.set_vision_range(vision_range);
	init_response.set_home_radius(home_radius);
	init_response.set_robot_radius(robot_radius);
	init_response.set_fov(fov);
	init_response.set_pickup_range(pickup_range);
	antix::send_pb(nodes_socket, &init_response);

	// add node to internal listing of nodes
	antixtransfer::Node_list::Node *node = node_list.add_node();
	node->set_ip_addr( init_msg.ip_addr() );
	node->set_id( next_node_id - 1 );
	node->set_neighbour_port( init_msg.neighbour_port() );
	node->set_gui_port( init_msg.gui_port() );

	cout << "Node connected. IP: " << node->ip_addr();
	cout << " Neighbour port: " << node->neighbour_port();
	cout << " GUI port: " << node->gui_port();
	cout << " Assigned id " << node->id() << "." << endl;

	// Init message sent by node also contains info on teams
	for (int i = 0; i < init_msg.team_size(); i++) {
		int client_id = init_msg.team(i).id();

		if (client_robot_map.count( client_id ) == 0) {
			client_robot_map.insert( pair<int, int>(client_id, init_msg.team(i).num_robots() ) );
			// record id & how many robots in node_list for transmission to nodes
			antixtransfer::Node_list::Robots_on_Node *rn = node_list.add_robots_on_node();
			rn->set_num_robots( init_msg.team(i).num_robots() );
			rn->set_team( client_id );
		}
	}

	cout << "Total nodes: " << node_list.node_size() << "." << endl;
}

/*
	A node has sent a message indicating it has heard a message on our pub sock
	Add it to the set if it is not already there
*/
void
handle_node_sync(zmq::socket_t *node_rep_sock, set<int> *nodes_synced) {
	antixtransfer::node_master_sync sync_msg;
	int rc = antix::recv_pb(node_rep_sock, &sync_msg, 0);
	assert(rc == 1);
	antix::send_blank(node_rep_sock);

	if (nodes_synced->count( sync_msg.my_id() ) == 0) {
		nodes_synced->insert( sync_msg.my_id() );
		cout << "Node " << sync_msg.my_id() << " synchronised. Need " << node_list.node_size() - nodes_synced->size() << " more." << endl;
	}
}

/*
	Send simulation parameters and an ID to the client
*/
void
send_client_init(zmq::socket_t *client_rep_sock, int id) {
	antixtransfer::MasterServerClientInitialization init_response;

	init_response.set_id( id );
	init_response.set_vision_range(vision_range);
	init_response.set_fov(fov);
	init_response.set_world_size(world_size);
	init_response.set_home_radius(home_radius);
	init_response.set_robot_radius(robot_radius);
	init_response.set_sleep_time(sleep_time);
	init_response.set_pickup_range(pickup_range);

	antix::send_pb(client_rep_sock, &init_response);
}

/*
	Node has said it has finished its turn
	Add to list if it is not already there
	If list is complete, send out next turn message
*/
void
handle_done(zmq::socket_t *rep_sock,
	zmq::socket_t *publish_sock,
	set<int> *nodes_done) {

	int rc = antix::recv_pb(rep_sock, &done_msg, 0);
	assert(rc == 1);

	// we must respond since this is a REP socket
	antix::send_blank(rep_sock);

	if (done_msg.type() == antixtransfer::done::NODE) {
		// Record node if we haven't heard from it before
		nodes_done->insert(done_msg.my_id());

		// Update global scores (though they are not sent every turn)
		int scores_size = done_msg.scores_size();
		for (int i = 0; i < scores_size; i++) {
			// if no score record in map, create one (not really necessary,
			// we could populate this at start, but doesn't matter too much
			if (scores.count( done_msg.scores(i).team_id() ) == 0 ) {
				scores.insert(
					pair<int, int>( done_msg.scores(i).team_id(), done_msg.scores(i).score() )
				);
			} else {
				scores[done_msg.scores(i).team_id()] = scores[done_msg.scores(i).team_id()] + done_msg.scores(i).score();
			}
		}

	} else {
		cerr << "Error: Bad type in done message." << endl;
		exit(-1);
	}
	
	// If we've heard from all nodes, start next turn
	if (nodes_done->size() == node_list.node_size()) {
		// Output scores to stdout
		if (antix::turn % TURNS_SEND_SCORE == 0) {
			for (map<int, int>::const_iterator it = scores.begin(); it != scores.end(); it++) {
				//cout << "Team " << it->first << " has score " << it->second << "." << endl;
			}
		}

		if (shutting_down) {
			cout << "Sending shutdown message to nodes..." << endl;
			antix::send_str(publish_sock, "s");

		} else {
#if DEBUG_SYNC
			cout << "Sync: Heard from " << nodes_done->size() << " nodes. Starting next turn." << endl;
#endif
#ifndef PRINT_TURNS_EVERY_TURN
			if (antix::turn % 20 == 0) {
#endif
				const double seconds = time(NULL) - start_time;
				if (seconds != 0)
					cout << antix::turn / seconds << " turns/second (" << antix::turn << " turns)" << endl;
#ifndef PRINT_TURNS_EVERY_TURN
			}
#endif
			antix::turn++;
			//cout << "Turn " << turns << " done." << endl;
			antix::send_str(publish_sock, "b");
		}
		nodes_done->clear();
	}
}

/*
	GUI sends a req. Send back a list of scores for each home
*/
void
handle_gui(zmq::socket_t *gui_rep_sock) {
	antix::recv_blank(gui_rep_sock);

	antixtransfer::Scores scores_msg;

	for (map<int, int>::const_iterator it = scores.begin(); it != scores.end(); it++) {
		antixtransfer::Scores::Score *score = scores_msg.add_score();
		score->set_id( it->first );
		score->set_score( it->second );
	}

	int rc = -1;
	while (rc != 1) {
		rc = antix::send_pb(gui_rep_sock, &scores_msg);
	}
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	antix::check_zmq_version();
	zmq::context_t context(1);
	srand( time(NULL) );
	srand48( time(NULL) );

	if (argc != 4) {
		cerr << "Usage: " << argv[0] << " <IP to listen on> <num pucks (approx)> <world size>" << endl;
		return -1;
	}

	host = string(argv[1]);
	num_pucks = atof(argv[2]);
	world_size = atof(argv[3]);

#ifndef NDEBUG
	cout << endl;
	cout << " *** STARTING SIMULATION WITH DEBUGGING ON *** " << endl << endl;
#endif

	// nodes/client socket are for nodes/clients connecting & giving their
	// ip. in return they get assigned an id
	zmq::socket_t nodes_socket(context, ZMQ_REP);
	zmq::socket_t clients_socket(context, ZMQ_REP);

	// to start the simulation, operator connects to this
	zmq::socket_t operators_socket(context, ZMQ_REP);

	// when operator starts, node list is published on this socket
	zmq::socket_t publish_socket(context, ZMQ_PUB);

	// gui gets score data on this socket
	zmq::socket_t gui_rep_sock(context, ZMQ_REP);

	// start listening for connections
	nodes_socket.bind(antix::make_endpoint(host, node_port));
	clients_socket.bind(antix::make_endpoint(host, client_port));
	operators_socket.bind(antix::make_endpoint(host, operator_port));
	publish_socket.bind(antix::make_endpoint(host, publish_port));
	gui_rep_sock.bind(antix::make_endpoint(host, gui_port));

	cout << "Master started." << endl;

	set<int> nodes_synced;

	// Init nodes, GUI client
	// Sync PUB sock
	// Handle operator begin
	while (!begun) {
		string type;

		antix::sleep(200);

		// send a message on our pub sock so connected nodes can hear it (for sync)
		antix::send_blank_envelope(&publish_socket, "sync");

		// attempt to recv from node
		if (antix::recv_str(&nodes_socket, &type, ZMQ_NOBLOCK) == 1) {
			if (type == "connect") {
				handle_node_init(&nodes_socket);

			} else if (type == "sync") {
				handle_node_sync(&nodes_socket, &nodes_synced);

			} else {
				cerr << "Error: unknown type from node connection" << endl;
				exit(-1);
			}
		}

		// GUI client
		if (antix::recv_str(&clients_socket, &type, ZMQ_NOBLOCK) == 1) {
			if (type == "init_gui_client") {
				cout << "GUI client connected." << endl;
				send_client_init(&clients_socket, -1);
			} else {
				cerr << "Error: unknown type from client connection" << endl;
				exit(-1);
			}
		}
		
		// Operator
		if (antix::recv_blank(&operators_socket, ZMQ_NOBLOCK) == 1) {
			cout << "Operator sent begin signal." << endl;
			antix::send_blank(&operators_socket);

			// ensure we have at least 3 nodes
			if ( node_list.node_size() < 3 ) {
				cerr << "Error starting simulation: we need at least 3 nodes." << endl;
				continue;
			}

			// ensure all nodes are synced on our pub sock
			if ( nodes_synced.size() != node_list.node_size() ) {
				cerr << "Error: Nodes are not yet synchronised on PUB socket." << endl;
				continue;
			}

			// set homes & their locations in the node list
			setup_homes();

			// assign nodes to create robots for each team initially
			assign_robots_to_node();

			// set pucks per node
			node_list.set_initial_pucks_per_node( ceil( (double) num_pucks / (double) node_list.node_size() ) );
			cout << "Made " << node_list.initial_pucks_per_node() << " pucks per node." << endl;

#ifdef MULTI_NODES_MACHINE
			// set each node's neighbour in the node list
			set_node_neighbours();
#endif

			// assign each node in our list an x offset
			set_node_offsets();

			// Print out the nodes in order & with their offsets
			output_node_offsets();

			// send message on publish_socket containing a list of nodes
			antix::send_pb_envelope(&publish_socket, &node_list, "start");
			cout << "Sent node list to nodes." << endl;
			cout << "Nodes sent:" << endl;
			antix::print_nodes(&node_list);
			cout << "Homes sent:" << endl;
			for (int i = 0; i < node_list.home_size(); i++)
				cout << "Home with team id " << node_list.home(i).team() << endl;

			cout << "Simulation begun." << endl;
			begun = true;
		}
	}

	start_time = time(NULL);

	// polling set
	zmq::pollitem_t items [] = {
		{ nodes_socket, 0, ZMQ_POLLIN, 0 },
		{ operators_socket, 0, ZMQ_POLLIN, 0},
		{ gui_rep_sock, 0, ZMQ_POLLIN, 0}
	};
	// respond to shutdown messages from operator
	// and turn done messages from node
	// expect operator sending shutdown
	while (1) {
		//zmq::message_t message;
		zmq::poll(&items [0], 3, -1);

		// message from a node
		if (items[0].revents & ZMQ_POLLIN) {
			string type = antix::recv_str(&nodes_socket);

			// Multiple possible messages from node, but expect only 'done'
			// a node stating it has finished its work for this turn
			if (type == "done") {
				handle_done(&nodes_socket, &publish_socket, &nodes_done);

			// should never get here...
			} else {
				cerr << "Error: Unknown node message!" << endl;
				exit(-1);
			}
		}

		// message from an operator
		if (items[1].revents & ZMQ_POLLIN) {
			// this is a shutdown signal
			antix::recv_blank(&operators_socket);
			cout << "Operator sent shutdown signal. Shutting down..." << endl;
			shutting_down = true;
			antix::send_blank(&operators_socket);
		}

		// message from gui
		if (items[2].revents & ZMQ_POLLIN) {
			handle_gui(&gui_rep_sock);
		}
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
