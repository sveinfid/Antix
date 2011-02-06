/*
	Client connects to master to get a list of nodes
	and then connects to all nodes
*/

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_client_port = "7771";
string master_publish_port = "7773";

int pid;

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
	antix::recv_pb(&client_master_sock, &init_response);
	int num_robots = init_response.robotsallocated();
	cout << "the numer of robots allocated " << num_robots << endl;
	
		
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

	return 0;
}
