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
double world_size;
double my_min_x;
// where neighbour begins
double my_max_x;
int sleep_time;
int initial_puck_amount;
antixtransfer::Node_list::Node left_node;
antixtransfer::Node_list::Node right_node;
vector<Puck> pucks;

/*
	Find our offset in node_list and set my_min_x, my_max_x
*/
void
set_dimensions(antixtransfer::Node_list *node_list) {
	antixtransfer::Node_list::Node *node;
	double offset_size = world_size / node_list->node_size();

	for (int i = 0; i < node_list->node_size(); i++) {
		node = node_list->mutable_node(i);
		if (node->id() == my_id) {
			my_min_x = node->x_offset();
			my_max_x = my_min_x + offset_size;
			return;
		}
	}
}

/*
	Place pucks randomly within our region
*/
void
generate_pucks() {
	for (int i = 0; i < initial_puck_amount; i++) {
		pucks.push_back( Puck(my_min_x, my_max_x, world_size) );
	}
}

int main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	
	if (argc != 4) {
		cerr << "Usage: " << argv[0] << " <IP to listen on> <node-node port> <client-node port>" << endl;
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
	pb_init_msg.set_neighbour_port( string(argv[2]) );
	pb_init_msg.set_client_port( string(argv[3]) );
	antix::send_pb(&node_master_sock, &pb_init_msg);

	// receive message back stating our unique ID
	antixtransfer::connect_init_response init_response;
	antix::recv_pb(&node_master_sock, &init_response);
	my_id = init_response.id();
	world_size = init_response.world_size();
	sleep_time = init_response.sleep_time();
	// XXX need to add this in master
	// initial_puck_amount = init_response.puck_amount();
	initial_puck_amount = 10;
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

	// calculate our min / max x from the offset assigned to us in node_list
	set_dimensions(&node_list);

	// find our left/right neighbours
	antix::set_neighbours(&left_node, &right_node, &node_list, my_id);
	cout << "Left neighbour: " << left_node.id() << endl;
	cout << "Right neighbour: " << right_node.id() << endl;

	// connect & subscribe to both neighbour's PUB sockets
	// this socket will receive foreign entities that are near our border
	zmq::socket_t neighbour_publish_sock(context, ZMQ_SUB);
	// XXX subscribe only to messages intended for us?
	// as the neighbours will be publishing data for both of its neighbours
	// on the same port
	// right now we receive all messages
	neighbour_publish_sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);

	// connect to the neighbours on this socket
	neighbour_publish_sock.connect(antix::make_endpoint(left_node.ip_addr(), left_node.neighbour_port()));
	neighbour_publish_sock.connect(antix::make_endpoint(right_node.ip_addr(), right_node.neighbour_port()));

	// open PUB socket neighbours where we publish entities close to the borders
	zmq::socket_t send_to_neighbour_sock(context, ZMQ_PUB);
	send_to_neighbour_sock.bind(antix::make_endpoint( argv[1], argv[2] ));

	// create REP socket that receives messages from clients
	// (sense, setspeed, pickup, drop)
	zmq::socket_t client_sock(context, ZMQ_REP);
	client_sock.bind(antix::make_endpoint( argv[1], argv[3] ));

	// generate pucks
	generate_pucks();
	cout << "Created " << pucks.size() << " pucks." << endl;
	for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		cout << "Puck at " << it->x << "," << it->y << endl;
	}

	// enter main loop
	while (1) {
		// publish data to neighbours of entities near left border
		// publish data to neighbours of entities near right border

		// read from our neighbour SUB socket & update our foreign entity knowledge

		// update poses for internal robots

		// service client messages on REP socket

		// sleep
		// code from rtv's Antix
		usleep( sleep_time * 1e3);
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
