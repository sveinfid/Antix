/*
	Master sets up the nodes & clients

	The majority of ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all

	Polling sockets stuff from:
	https://github.com/imatix/zguide/blob/master/examples/C++/mspoller.cpp
*/

#include "antix.cpp"

using namespace std;

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
const double home_radius = 0.01;
// radius of robot
const double robot_radius = 0.1;
const double pickup_range = vision_range / 5.0;

// listening on
string host;
string node_port = "7770";
string client_port = "7771";
string operator_port = "7772";
string publish_port = "7773";

int next_node_id = 0;
int next_client_id = 0;

antixtransfer::Node_list node_list;

// used for synchronization
map<int, map<int, bool> > nodes_heard;

void
print_sync_status() {
	for(map<int, map<int, bool> >::iterator it = nodes_heard.begin(); it != nodes_heard.end(); it++) {
		cout << "Node " << it->first << " has heard " << it->second.size() << " others" << endl;
	}
}

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

int
check_sync_done() {
	for (map<int, map<int, bool> >::iterator it = nodes_heard.begin(); it != nodes_heard.end(); it++) {
		// -1 since node won't hear from itself!
		if (it->second.size() != nodes_heard.size()-1)
			return 0;
	}
	return 1;
}

int
add_heard(int id, int heard_id) {
	map<int, bool> nodes_heard_i = nodes_heard[id];
	// node hasn't heard of heard_id yet
	if (nodes_heard_i.count(heard_id) == 0) {
		nodes_heard[id].insert( pair<int, bool>(heard_id, true) );
	}
	return check_sync_done();
}

int main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);

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
			// Two types of messages expected from node:
			// - an initial connection
			// - an msg of having heard another node
			if (type == "connect") { 
				// this message should be node giving its ip (& port maybe?)
				antixtransfer::connect_init_node init_msg;
				antix::recv_pb(&nodes_socket, &init_msg, 0);

				// respond with an id for the node & config info
				antixtransfer::connect_init_response init_response;
				init_response.set_id(next_node_id++);
				init_response.set_world_size(world_size);
				init_response.set_sleep_time(sleep_time);
				init_response.set_puck_amount(initial_pucks_per_node);
				antix::send_pb(&nodes_socket, &init_response);

				// add node to internal listing of nodes
				antixtransfer::Node_list::Node *node = node_list.add_node();
				node->set_ip_addr( init_msg.ip_addr() );
				node->set_id( next_node_id - 1 );
				node->set_left_port( init_msg.left_port() );
				node->set_right_port( init_msg.right_port() );
				node->set_control_port( init_msg.control_port() );

				cout << "Node connected. IP: " << node->ip_addr() << " Left port: " << node->left_port() << " Right port: " << node->right_port() << " Control port: " << node->control_port() << " Assigned id " << node->id() << "." << endl;
				cout << "Total nodes: " << node_list.node_size() << "." << endl;
			
			// Otherwise it's a node sync message
			} else {
				antixtransfer::node_master_sync sync_msg;
				antix::recv_pb(&nodes_socket, &sync_msg, 0);
				cout << "Got a sync message from node " << sync_msg.my_id() << " saying it heard " << sync_msg.heard_id() << endl;
				print_sync_status();
				// reply with a blank since this is a rep socket
				antix::send_blank(&nodes_socket);
				// everyone's heard everyone
				if (add_heard(sync_msg.my_id(), sync_msg.heard_id()) == 1) {
					antix::send_blank(&publish_socket);
				}
			}
		}

		// message from a client
		if (items[1].revents & ZMQ_POLLIN) {
			// this message should be client giving its ip & port
			antixtransfer::connect_init_client init_msg;
			antix::recv_pb(&clients_socket, &init_msg, 0);
			// respond with an id for the client
			antixtransfer::MasterServerClientInitialization init_response;
			init_response.set_id(next_client_id++);
			init_response.set_visionrange(vision_range);
			init_response.set_fieldofview(fov);
			init_response.set_serverwidth(world_size);
			init_response.set_serverheight(world_size);
			init_response.set_homeradius(home_radius);
			init_response.set_sleep_time(sleep_time);
			cout << "Client connected (" << init_msg.ip_addr() << "). Assigned id " << init_response.id() << endl;
			antix::send_pb(&clients_socket, &init_response);
		}

		// message from an operator
		if (items[2].revents & ZMQ_POLLIN) {
			// setup our sync map
			// map<int, map<int, bool> > nodes_heard;
			for (int i = 0; i < next_node_id; i++) {
				map<int, bool> blank_map;
				nodes_heard.insert( pair<int, map<int, bool> >(i, blank_map) );
			}

			// only message is begin right now
			cout << "Got begin message from an operator" << endl;
			operators_socket.recv(&message);

			zmq::message_t blank(1);
			operators_socket.send(blank);

			// ensure we have at least 3 nodes
			if ( node_list.node_size() < 3 ) {
				cout << "Error running begin: we need at least 3 nodes." << endl;
				continue;
			}

			// assign each node in our list an x offset
			set_node_offsets();

			// send message on publish_socket containing a list of nodes
			antix::send_pb(&publish_socket, &node_list);
			cout << "Sent node list to nodes & clients." << endl;
			cout << "Nodes sent:" << endl;
			antix::print_nodes(&node_list);

			cout << "Simulation begun." << endl;

			// any nodes/clients that connect after this get unexpected results
		}
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
