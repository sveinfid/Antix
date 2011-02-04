/*
	Node / map piece server
*/

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_node_port = "7770";
string master_publish_port = "7773";

int main(int argc, char **argv) {
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <IP to listen on>" << endl;
		return -1;
	}

	zmq::context_t context(1);

	// socket to announce ourselves to master on
	zmq::socket_t node_master_sock(context, ZMQ_REQ);
	node_master_sock.connect(antix::make_endpoint(master_host, master_node_port));

	zmq::socket_t master_publish_sock(context, ZMQ_SUB);
	// make sure to filter this SUB to get all messages (should just have list of
	// nodes message)

	cout << "Sending master existence notification...";
	// send message announcing ourself. includes our own IP

	// receive message back stating our unique ID

	// block on master_publish_sock

	// once we have received message from master publish sock, we have a list
	// of nodes

	// find our left/right neighbour & connect with SUB socket
	// this socket will receive foreign entities that are near our border
	//connect(left)
	//connect(right)

	// open PUB socket for left neighbour where we publish entities close to left
	// do the same for the right neighbour (2 sockets)


	// create REP socket that receives messages from clients
	// (sense, setspeed, pickup, drop)

	// generate pucks

	// enter main loop
	while (1) {
		// publish data to left neighbour containing foreign entities
		// publish data to right neighbour containing foreign entities

		// read from our SUB socket & update our foreign entity knowledge

		// update poses for internal robots

		// service client messages on REP socket

		// sleep
	}

	return 0;
}
