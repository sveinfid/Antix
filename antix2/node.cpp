/*
	Node / map piece server

	The majority of the ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all
*/

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_node_port = "7770";
string master_publish_port = "7773";

int my_id;
antixtransfer::Node_list::Node left_node;
antixtransfer::Node_list::Node right_node;

int main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	
	if (argc != 3) {
		cerr << "Usage: " << argv[0] << " <IP to listen on> <port>" << endl;
		return -1;
	}

	zmq::context_t context(1);

	// socket to announce ourselves to master on
	zmq::socket_t node_master_sock(context, ZMQ_REQ);
	node_master_sock.connect(antix::make_endpoint(master_host, master_node_port));
	cout << "Connecting to master..." << endl;

	zmq::socket_t master_publish_sock(context, ZMQ_SUB);
	// subscribe to all messages on this socket: should just be a list of nodes
	master_publish_sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock.connect(antix::make_endpoint(master_host, master_publish_port));

	// send message announcing ourself. includes our own IP
	cout << "Sending master our existence notification..." << endl;

	// create & send pb msg
	antixtransfer::connect_init_node pb_init_msg;
	pb_init_msg.set_ip_addr( string(argv[1]) );
	pb_init_msg.set_port( string(argv[2]) );
	antix::send_pb(&node_master_sock, &pb_init_msg);

	// receive message back stating our unique ID
	antixtransfer::connect_init_response init_response;
	antix::recv_pb(&node_master_sock, &init_response);
	my_id = init_response.id();
	cout << "We are now node id " << my_id << endl;

	// receive node list
	// blocks until master publishes list of nodes
	antixtransfer::Node_list node_list;
	antix::recv_pb(&master_publish_sock, &node_list);
	cout << "Received list of nodes:" << endl;
	antix::print_nodes(&node_list);

	if (node_list.node_size() < 3) {
		cout << "Error: we need at least 3 nodes. Only received " << node_list.node_size() << " node(s)." << endl;
		return -1;
	}

	// find our left/right neighbours
	antix::set_neighbours(&left_node, &right_node, &node_list, my_id);
	cout << "Left neighbour: " << left_node.id() << " : " << left_node.ip_addr() << endl;
	cout << "Right neighbour: " << right_node.id() << " : " << right_node.ip_addr() << endl;

	// connect & subscribe to both neighbour's PUB sockets
	// this socket will receive foreign entities that are near our border
	zmq::socket_t neighbour_publish_sock(context, ZMQ_SUB);
	// subscribe to all messages on this socket: should just be a list of nodes
	neighbour_publish_sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
	//neighbour_publish_sock.connect(antix::make_endpoint(master_host, master_publish_port));

	// ensure socket is set to filter to recv all messages
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
