/*
	This connects to the master node & sends a blank message to trigger
	simulation begin
*/

#include "antix.cpp"

using namespace std;

string master_host;
// operator port
string master_port = "7772";

int main(int argc, char **argv) {
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <IP of master>" << endl;
		return -1;
	}
	master_host = string(argv[1]);

	zmq::context_t context(1);
	zmq::socket_t sock(context, ZMQ_REQ);

	sock.connect(antix::make_endpoint(master_host, master_port));

	// send message with no content to indicate begin
	antix::send_blank(&sock);
	cout << "Sent begin command to master." << endl;

	// get a blank ACK or else the REP socket will hang
	zmq::message_t resp;
	sock.recv(&resp);

	cout << "Done." << endl;
}
