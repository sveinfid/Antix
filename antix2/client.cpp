/*
	Client connects to master to get a list of nodes
	and then connects to all nodes
*/

#include <map>
#include "antix.cpp"


using namespace std;

string master_host = "localhost";
string master_client_port = "7771";
string master_publish_port = "7773";
string client_node_port = "7774";

int pid;
typedef map<int, zmq::socket_t*> node_map;

node_map get_node_map(antixtransfer::Node_list *node_list){
	map<int, zmq::socket_t*> node_map;
	antixtransfer::Node_list::Node *node;
	zmq::context_t context(1);
	for (int i = 0; i < node_list->node_size(); i++) {
		node = node_list->mutable_node(i);
		zmq::socket_t *client_node_sock = new zmq::socket_t(context, ZMQ_REQ);
		node_map.insert(pair<int, zmq::socket_t*> (node->id(), client_node_sock));
	}
	return node_map;
}

int main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	if (argc != 3){
		cerr << "Usage: " << argv[0] << " <IP to listen on> <# of robots>" << endl;
		return -1;
	}
	zmq::context_t context(1);

	// REQ socket to master_cli port
	zmq::socket_t client_master_sock(context, ZMQ_REQ);
	// send message giving our IP
	printf("connecting to master...\n");
	client_master_sock.connect(antix::make_endpoint(master_host, master_client_port));
	antixtransfer::connect_init_client init_connect;
	assert(atoi(argv[2]) > 0);
	init_connect.set_number_of_robots_requested(atoi(argv[2]));
	init_connect.set_ip_addr(atoi(argv[1]));
	antix::send_pb(&client_master_sock, &init_connect);
	
	antixtransfer::MasterServerClientInitialization init_response;
	antix::recv_pb(&client_master_sock, &init_response, 0);
	int num_robots = init_response.robotsallocated();
	cout << "the numer of robots allocated " << num_robots << endl;

	zmq::socket_t *master_publish_sock = new zmq::socket_t(context, ZMQ_SUB);
	master_publish_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock->connect(antix::make_endpoint(master_host, master_publish_port));

	// receive message from master to receive a list of nodes from master
	antixtransfer::Node_list node_list;
	//waiting for master to send a list of nodes
	antix::recv_pb(master_publish_sock, &node_list, 0);
	cout << "Received pub_msg from master" << endl;
	antix::print_nodes(&node_list);

	map<int, zmq::socket_t*> map;
	map = get_node_map(&node_list);
	cout << "made a map of node_list" << endl;
	
	/*TODO: need to iterate through a map and make connection with nodes
	map<int, zmq::socket_t*>::iterator it;
	for(it = map.begin; map.end(); it++)
	{
		pair<map<int, zmq::socket_t*>> p = *it;
		//client_node_sock.connect(antix::make_endpoint(master_host, client_node_sock));

	}
	*/

	//antixtransfer::Node_list::Node 		
	
		
	// get message giving our ID

	// SUB socket to master
	// make sure to filter to get all messages

	// block on SUB until receive list of nodes

	// connect REQ socket to all nodes

	// generate robots
	// - message the REQ socket that is connected to all nodes indicating
	//   where to place robot

	// enter main loop
	while (1) {
		// request sense data for our robots

		// receive it

		// decide commands to send for each robot

		// send them

		// receive response?

		// sleep
	}
	google::protobuf::ShutdownProtobufLibrary();

	return 0;
}
