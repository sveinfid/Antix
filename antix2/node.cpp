/*
	Node / map piece server
*/

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_node_port = "7770";
string master_publish_port = "7773";

int main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <IP to listen on>" << endl;
		return -1;
	}

	zmq::context_t context(1);

	// socket to announce ourselves to master on
	zmq::socket_t node_master_sock(context, ZMQ_REQ);
	node_master_sock.connect(antix::make_endpoint(master_host, master_node_port));
	cout << "connecting to master..." << endl;
	zmq::socket_t master_publish_sock(context, ZMQ_SUB);
	// subscribe to all messages on this socket: should just be a list of nodes
	master_publish_sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock.connect(antix::make_endpoint(master_host, master_publish_port));

	// send message announcing ourself. includes our own IP
	cout << "Sending master existence notification..." << endl;

	// create pb msg
	//
	antixtransfer::connect_init_node pb_init_msg;
	pb_init_msg.set_ip_addr( string(argv[1]) );

	antix::send_pb(&node_master_sock, &pb_init_msg);

	// receive message back stating our unique ID
	antixtransfer::connect_init_response init_response;
	antix::recv_pb(&node_master_sock, &init_response);
	cout << "Got id " << init_response.id() << endl;

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

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
