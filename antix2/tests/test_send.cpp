#include "antix.cpp"

using namespace std;

int
main(int argc, char **argv) {
	if (argc != 5) {
		cerr << "Usage: " << argv[0] << " <port to send to on localhost> <msg type> <N = number of robots> <M = number of pucks>" << endl;
		cerr << "Message types:" << endl;
		cerr << "\t0 - send map with given robots & pucks to either a node or a client" << endl;
		cerr << "\t1 - Single robot transfer from one node to another" << endl;
		cerr << "\t2 - ADD_BOT control message with N robots" << endl;
		cerr << "\t3 - SENSE control message" << endl;
		cerr << "\t4 - SETSPEED control message for N robots" << endl;
		cerr << "\t5 - PICKUP/DROP control message for N robots" << endl;
		cout << "\t6 - Redesign: sendmap2" << endl;
		return -1;
	}
	int msg_type = atoi(argv[2]);
	int num_robots = atoi(argv[3]);
	int num_pucks = atoi(argv[4]);

	zmq::context_t context(1);
	zmq::socket_t sock(context, ZMQ_REQ);
	sock.connect(antix::make_endpoint("localhost", string(argv[1])));

	cout << "NOTE: This includes initial TCP handshake which would not occur in the actual simulation." << endl;
	cout << endl;

	// SendMap message
	if (msg_type == 0) {
		cout << "Sending SendMap message with " << num_robots << " robots and " << num_pucks << " pucks." << endl;

		antixtransfer::SendMap map;
		for (int i = 0; i < num_robots; i++) {
			antixtransfer::SendMap::Robot *r = map.add_robot();
			r->set_team(1);
			r->set_id(i);
			r->set_x(0.0030345);
			r->set_y(0.0005678);
		}
		for (int i = 0; i < num_pucks; i++) {
			antixtransfer::SendMap::Puck *p = map.add_puck();
			p->set_x(0.9999998);
			p->set_y(0.1123456);
			p->set_held(false);
		}
		antix::send_pb(&sock, &map);
		antix::recv_blank(&sock);
		cout << "Received response." << endl;

	// RequestRobotTransfer message
	} else if (msg_type == 1) {
		// NOTE: May actually be slightly more due to enveloping in simulation...
		antixtransfer::RequestRobotTransfer transfer_msg;
		transfer_msg.set_id(1);
		transfer_msg.set_team(1);
		transfer_msg.set_x(0.5);
		transfer_msg.set_y(0.9);
		transfer_msg.set_a(0.4);
		transfer_msg.set_v(0.3);
		transfer_msg.set_w(0.2);
		transfer_msg.set_has_puck(true);
		cout << "Sending RequestRobotTransfer for ONE robot..." << endl;
		antix::send_pb(&sock, &transfer_msg);
		cout << "No response necessary due to sent over PUB/SUB socket in simulation." << endl;
	
	// ADD_BOT ctl message
	} else if (msg_type == 2) {
		antixtransfer::control_message ctl;
		ctl.set_team(1);
		ctl.set_type(antixtransfer::control_message::ADD_BOT);
		for (int i = 0; i < num_robots; i++) {
			antixtransfer::control_message::Robot *r = ctl.add_robot();
			r->set_id(1);
		}
		antix::send_pb(&sock, &ctl);
		cout << "Sending ADD_BOT msg containing " << num_robots << " robots..." << endl;
		cout << "Waiting for response..." << endl;
		antix::recv_blank(&sock);

	// SENSE ctl message
	} else if (msg_type == 3) {
		antixtransfer::control_message ctl;
		ctl.set_team(1);
		ctl.set_type(antixtransfer::control_message::SENSE);
		antix::send_pb(&sock, &ctl);
		cout << "Sending SENSE msg ..." << endl;
		cout << "Waiting for response..." << endl;
		antix::recv_blank(&sock);

	// SETSPEED ctl message
	} else if (msg_type == 4) {
		antixtransfer::control_message ctl;
		ctl.set_team(1);
		ctl.set_type(antixtransfer::control_message::SETSPEED);
		for (int i = 0; i < num_robots; i++) {
			antixtransfer::control_message::Robot *r = ctl.add_robot();
			r->set_id(1);
			r->set_v(0.5);
			r->set_w(0.004);
		}
		cout << "Sending SETSPEED msg containing " << num_robots << " robots..." << endl;
		antix::send_pb(&sock, &ctl);
		cout << "Waiting for response..." << endl;
		antix::recv_blank(&sock);

	// PICKUP/DROP ctl message
	} else if (msg_type == 5) {
		antixtransfer::control_message ctl;
		ctl.set_team(1);
		ctl.set_type(antixtransfer::control_message::PICKUP);
		for (int i = 0; i < num_robots; i++) {
			antixtransfer::control_message::Robot *r = ctl.add_robot();
			r->set_id(1);
		}
		cout << "Sending PICKUP/DROP msg containing " << num_robots << " robots..." << endl;
		antix::send_pb(&sock, &ctl);
		cout << "Waiting for response..." << endl;
		antix::recv_blank(&sock);

	// SendMap2 message
	} else if (msg_type == 6) {
		cout << "Sending SendMap2 message with " << num_robots << " robots." << endl;

		antixtransfer::SendMap2 map;
		for (int i = 0; i < num_robots; i++) {
			antixtransfer::SendMap2::Robot *r = map.add_robot();
			r->set_x(0.0030345);
			r->set_y(0.0005678);
			r->set_puck_id(999999);
			r->set_puck_action(true);
		}
		/*
		for (int i = 0; i < num_pucks; i++) {
			antixtransfer::SendMap::Puck *p = map.add_puck();
			p->set_x(0.9999998);
			p->set_y(0.1123456);
			p->set_held(false);
		}
		*/
		antix::send_pb(&sock, &map);
		antix::recv_blank(&sock);
		cout << "Received response." << endl;

	// Invalid message given
	} else {
		cerr << "Invalid message type" << endl;
	}

	return 0;
}
