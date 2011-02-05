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
const int node_pucks = 100;
// range of robot sight
const double sight_range = 0.1;
// robot fov
//const double fov = dtor(90.0);
const double fov = 1.5;

// listening on
string host = "*";
string node_port = "7770";
string client_port = "7771";
string operator_port = "7772";
string publish_port = "7773";

int next_node_id = 0;
int next_client_id = 0;

antixtransfer::Node_list node_list;

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

int main() {
	zmq::context_t context(1);

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
			// this message should be node giving its ip (& port maybe?)
			antixtransfer::connect_init_node init_msg;
			antix::recv_pb(&nodes_socket, &init_msg);

			// respond with an id for the node & config info
			antixtransfer::connect_init_response init_response;
			init_response.set_id(next_node_id++);
			init_response.set_world_size(world_size);
			init_response.set_sleep_time(sleep_time);
			antix::send_pb(&nodes_socket, &init_response);

			// add node to internal listing of nodes
			antixtransfer::Node_list::Node *node = node_list.add_node();
			node->set_ip_addr( init_msg.ip_addr() );
			node->set_id( next_node_id - 1 );

			cout << "Node with IP " << node->ip_addr() << " connected. Assigned id " << node->id() << "." << endl;
			cout << "Total nodes: " << node_list.node_size() << "." << endl;
		}

		// message from a client
		if (items[1].revents & ZMQ_POLLIN) {
			cout << "Got msg from a client" << endl;
			clients_socket.recv(&message);
			// this message should be client giving its ip & port
			// respond with an id for the client
		}

		// message from an operator
		if (items[2].revents & ZMQ_POLLIN) {
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

	return 0;
}
