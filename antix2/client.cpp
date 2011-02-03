/*
	Client connects to master to get a list of nodes
	and then connects to all nodes
*/

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_client_port = "7771";
string master_publish_port = "7773";

int main() {
	zmq::context_t context(1);

	// REQ socket to master_cli port
	// send message giving our IP
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
