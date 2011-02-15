/*
	Master sets up the nodes & clients

	The majority of ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all

	Polling sockets stuff from:
	https://github.com/imatix/zguide/blob/master/examples/C++/mspoller.cpp
*/

#include "antix.cpp"

using namespace std;

/*
	Simulation settings
*/
const double world_size = 1.0;
// 5 seconds
const int sleep_time = 5000;
// pucks per node to initially create
const int initial_pucks_per_node = 10;
// range of robot sight
const double vision_range = 0.1;
// robot fov
const double fov = antix::dtor(90.0);
// radius of homes
const double home_radius = 0.1;
// radius of robot
const double robot_radius = 0.01;
const double pickup_range = vision_range / 5.0;

// track whether simulation has begun
bool begun = false;

/*
	Master network settings
*/
string host;
string node_port = "7770";
string client_port = "7771";
string operator_port = "7772";
string publish_port = "7773";

int next_node_id = 0;
int next_client_id = 0;
antixtransfer::Node_list node_list;

// used for synchronous turns
set<int> nodes_done;

/*
	Go through our list of nodes & assign an x offset to the node for which
	it is responsible to manage the map for
*/
void
set_node_offsets() {
	double offset_size = world_size / node_list.node_size();
	double position = 0;

	antixtransfer::Node_list::Node *node;
	for (int i = 0; i < node_list.node_size(); i++) {
		node = node_list.mutable_node(i);
		node->set_x_offset(position);
		cout << "Assign node with id " << node->id() << " x offset " << position << endl;
		position = position + offset_size;
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
	for (int i = 0; i < next_client_id; i++) {
		antixtransfer::Node_list::Home *h = node_list.add_home();
		h->set_team( i );
		// XXX should ensure cannot overlap
		h->set_x( antix::rand_between(0, world_size) );
		h->set_y( antix::rand_between(0, world_size) );
		cout << "Created home for team " << h->team() << " at (" << h->x() << ", " << h->y() << ")" << endl;
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
	for (int i = 0; i < node_list.robots_on_node_size(); i++) {
		rn = node_list.mutable_robots_on_node(i);
		// 0 to max node id
		rn->set_node( rand() % (next_node_id - 1) );
		cout << "Assigned robots from team " << node_list.robots_on_node(i).team() << " to node " << node_list.robots_on_node(i).node() << endl;
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
	antix::recv_pb(nodes_socket, &init_msg, 0);

	// respond with an id for the node & config info
	antixtransfer::connect_init_response init_response;
	init_response.set_id(next_node_id++);
	init_response.set_world_size(world_size);
	init_response.set_sleep_time(sleep_time);
	init_response.set_puck_amount(initial_pucks_per_node);
	init_response.set_vision_range(vision_range);
	init_response.set_fov(fov);
	antix::send_pb(nodes_socket, &init_response);

	// add node to internal listing of nodes
	antixtransfer::Node_list::Node *node = node_list.add_node();
	node->set_ip_addr( init_msg.ip_addr() );
	node->set_id( next_node_id - 1 );
	node->set_neighbour_port( init_msg.neighbour_port() );
	node->set_control_port( init_msg.control_port() );
	node->set_gui_port( init_msg.gui_port() );

	cout << "Node connected. IP: " << node->ip_addr();
	cout << " Neighbour port: " << node->neighbour_port();
	cout << " Control port: " << node->control_port();
	cout << " GUI port: " << node->gui_port();
	cout << " Assigned id " << node->id() << "." << endl;

	cout << "Total nodes: " << node_list.node_size() << "." << endl;
}

/*
	Node has said it has finished its turn
	Add to list of completed nodes
	If every node is in the list, send out next turn message
*/
void
handle_node_done(zmq::socket_t *nodes_socket, zmq::socket_t *publish_socket) {
	antixtransfer::node_master_done done_msg;
	antix::recv_pb(nodes_socket, &done_msg, 0);

	// we must respond to the node since this is a REP socket
	antix::send_blank(nodes_socket);

	// Record node if we haven't heard from it before
	if (nodes_done.count(done_msg.my_id()) == 0) {
		nodes_done.insert(done_msg.my_id());
	}
	
	// If we've heard from every node, start next turn
	if (nodes_done.size() == node_list.node_size()) {
		antix::send_blank(publish_socket);
	}
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);
  srand( time(NULL) );

	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <IP to listen on>" << endl;
		return -1;
	}

	host = string(argv[1]);

	// nodes/client socket are for nodes/clients connecting & giving their
	// ip. in return they get assigned an id
	zmq::socket_t nodes_socket(context, ZMQ_REP);
	zmq::socket_t clients_socket(context, ZMQ_REP);

	// to start the simulation, operator connects to this
	zmq::socket_t operators_socket(context, ZMQ_REP);

	// when operator starts, node list is published on this socket
	zmq::socket_t publish_socket(context, ZMQ_PUB);

	// start listening for connections
	nodes_socket.bind(antix::make_endpoint(host, node_port));
	clients_socket.bind(antix::make_endpoint(host, client_port));
	operators_socket.bind(antix::make_endpoint(host, operator_port));
	publish_socket.bind(antix::make_endpoint(host, publish_port));

	cout << "Master started." << endl;

	// polling set
	zmq::pollitem_t items [] = {
		{ nodes_socket, 0, ZMQ_POLLIN, 0 },
		{ clients_socket, 0, ZMQ_POLLIN, 0},
		{ operators_socket, 0, ZMQ_POLLIN, 0}
	};
	// respond to messages forever
	while (1) {
		zmq::message_t message;
		zmq::poll(&items [0], 3, -1);

		// message from a node
		if (items[0].revents & ZMQ_POLLIN) {
			nodes_socket.recv(&message, 0);
			string type = string( (char *) message.data() );

			// Multiple possible messages from node
			// a message upon initial connection
			if (type == "connect") { 
				handle_node_init(&nodes_socket);

			// a node stating it has finished its work for this turn
			} else if (type == "done") {
				handle_node_done(&nodes_socket, &publish_socket);

			// should never get here...
			} else {
				cerr << "Error: Unknown node message!" << endl;
			}
		}

		// message from a client / GUI
		if (items[1].revents & ZMQ_POLLIN) {
			// type indicates whether client or GUI client
			string type = antix::recv_str(&clients_socket);
			antixtransfer::MasterServerClientInitialization init_response;

			// only assign unique id to a regular client
			if (type == "client") {
				// client sends a second message part indicating # of robots it wants
				antixtransfer::connect_init_client init_request;
				antix::recv_pb(&clients_socket, &init_request, 0);

				init_response.set_id(next_client_id++);
				cout << "Client connected. Assigned ID " << init_response.id() << ". Wants " << init_request.num_robots() << " robots." << endl;
				
				// record id & how many robots in node_list for transmission to nodes
				antixtransfer::Node_list::Robots_on_Node *rn = node_list.add_robots_on_node();
				rn->set_num_robots( init_request.num_robots() );
				rn->set_team( init_response.id() );

			} else {
				init_response.set_id(0);
				cout << "GUI client connected." << endl;
			}

			// and the simulation parameters
			init_response.set_vision_range(vision_range);
			init_response.set_fov(fov);
			init_response.set_world_size(world_size);
			init_response.set_home_radius(home_radius);
			init_response.set_robot_radius(robot_radius);
			init_response.set_sleep_time(sleep_time);

			antix::send_pb(&clients_socket, &init_response);
		}

		// message from an operator
		// this can only be a message indicating the beginning of a simulation
		if (items[2].revents & ZMQ_POLLIN) {
			cout << "Operator sent begin signal." << endl;
			antix::recv_blank(&operators_socket);
			antix::send_blank(&operators_socket);

			// ensure we have at least 3 nodes
			if ( node_list.node_size() < 3 ) {
				cout << "Error starting simulation: we need at least 3 nodes." << endl;
				continue;
			}

			// assign each node in our list an x offset
			set_node_offsets();

			// set homes & their locations in the node list
			setup_homes();

			// assign nodes to create robots for each team initially
			assign_robots_to_node();

			// send message on publish_socket containing a list of nodes
			antix::send_pb(&publish_socket, &node_list);
			cout << "Sent node list to nodes." << endl;
			cout << "Nodes sent:" << endl;
			antix::print_nodes(&node_list);
			cout << "Homes sent:" << endl;
			for (int i = 0; i < node_list.home_size(); i++)
				cout << "Home with team id " << node_list.home(i).team() << endl;

			cout << "Simulation begun." << endl;
			begun = true;

			// any nodes that connect after this get unexpected results
		}
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
