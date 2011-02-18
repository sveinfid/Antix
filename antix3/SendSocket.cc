/*
	Unidirectional sending from one map server to another
*/

#include "../antix2/antix.cpp"

using namespace std;

class SendSocket {
public:
	zmq::socket_t *socket;

	/*
		Open a publishing socket on the given ip and port
	*/
	SendSocket(zmq::context_t *context, string ip, string port) {
		string endpoint = "tcp://" + ip + ":" + port;
		socket = new zmq::socket_t(*context, ZMQ_PUB);
		socket->bind( endpoint.c_str() );
	}

	/*
		Returns 0 if successful
	*/
	int
	send_map(antixtransfer::SendMap *sendmap) {
		return antix::send_pb_envelope(socket, sendmap, "m");
	}

	/*
		Returns 0 is successful
	*/
	int
	send_robot(antixtransfer::MoveRobot *move_robot) {
		return antix::send_pb_envelope(socket, move_robot, "r");
	}
};
