/*
	This connects to the master node & sends a blank message to trigger
	simulation begin
*/

#include <zmq.hpp>
#include <iostream>

using namespace std;

string master_host = "*";
// operator port
string master_port = "7772";

int main() {
	zmq::context_t context(1);
	zmq::socket_t sock(context, ZMQ_REQ);

	string master_s = "tcp://" + master_host + ":" + master_port;
	sock.connect(master_s.c_str());

	// send message with no content to indicate begin
	zmq::message_t begin(1);
	sock.send(begin);
	cout << "Sent master begin command" << endl;

	// get a blank ACK or else the REP socket will hang
	zmq::message_t resp;
	sock.recv(&resp);

	cout << "Done." << endl;
}
