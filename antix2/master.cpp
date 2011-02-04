/*
	Master sets up the nodes & clients
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
			cout << "Got msg from a node" << endl;
			nodes_socket.recv(&message);
			// this message should be node giving its ip & port
			// respond with an id for the node

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
			cout << "Got begin message from an operator" << endl;
			operators_socket.recv(&message);
			// only message is begin right now
			// send message on publish_socket containing a list
			// of nodes

			// any nodes/clients that connect after this get unexpected results
			// right now
		}
	}

	return 0;
}