/*
	Unidirectional sending from one map server to another

	To setup:
	SendSocket left_map(&context, our_ip, left_port);

	To send messages:

	antixtransfer::SendMap send_map_msg;
	// .. build send_map_msg
	left_map.send_map(send_map_msg);

	antixtransfer::MoveRobot move_robot_msg;
	// .. build move_robot_msg
	left_map.send_robot(move_robot_msg);
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
