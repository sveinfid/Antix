/*
	Unidirectional receiving of messages from another map server to the current
*/

#include "../antix2/antix.cpp"

using namespace std;

class ReceiveSocket {
public:
	zmq::socket_t *socket;

	/*
		Set up a socket for receiving messages
	*/
	ReceiveSocket(zmq::context_t *context) {
		socket = new zmq::socket_t(*context, ZMQ_SUB);
		socket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	}

	/*
		Connect and receive messages to the given IP and port
		Should be another neighbouring map server's SendSocket
	*/
	void
	add_neighbour(string ip, string port) {
		string endpoint = "tcp://" + ip + ":" + port;
		socket->connect( endpoint.c_str() );
	}

	/*
		After a check_messages() call which returns 0, call this function
		The protobuf message move_robot will then contain a valid MoveRobot object

		If called without check_messages(), will almost certainly block or
		return invalid data

		Returns 0 if successful
	*/
	int
	get_robot(antixtransfer::MoveRobot *move_robot) {
		return antix::recv_pb(socket, move_robot, 0);
	}

	/*
		Similarly to get_map(), this must be called after a check_messages()
		call which returns 1. Otherwise invalid.

		Places a valid SendMap map into the given object

		Returns 0 if successful
	*/
	int
	get_map(antixtransfer::SendMap *map) {
		return antix::recv_pb(socket, map, 0);
	}

	/*
		Check if there are any messages to be read on our socket
		Does not block

		If you call this function, you MUST call the respective
		get_map() or get_robot() depending on the return value

		If you fail to do this, everything will break.

		Returns:
		 -2 if received invalid message / error
		 -1 if no messages
		 0 if a robot message
		 1 if a map message
	*/
	int
	check_messages() {
		string type;
		if ( antix::recv_str(socket, &type, ZMQ_NOBLOCK) == 1 ) {
			if (type == "r")
				return 0;
			else if (type == "m")
				return 1;
			else
				return -2;
		}
		return -1;
	}
};
