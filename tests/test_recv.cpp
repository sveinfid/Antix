#include "antix.cpp"

using namespace std;

string port = "12345";

int
main(int argc, char **argv) {
	zmq::context_t context(1);
	zmq::socket_t sock(context, ZMQ_REP);
	sock.bind(antix::make_endpoint("127.0.0.1", port));

	cout << "Waiting for a message on port " << port << "..." << endl;
	zmq::message_t msg;
	sock.recv(&msg);
	cout << "Got a message of size " << msg.size() << endl;
	cout << "Sending ACK response..." << endl;
	antix::send_blank(&sock);
	return 0;
}
