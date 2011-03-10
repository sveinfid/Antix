/*
	Client connects to a node on its local machine & gets simulation parameters
	Then it begins controlling robots
*/

#include <map>
#include "entities.cpp"

using namespace std;

string node_ipc_prefix = "/tmp/node";
string node_ipc_id;

int my_id;
int sleep_time;
int num_robots;
double home_radius;
Home *my_home;

// local socket to control sock of node on our machine
zmq::socket_t *node_req_sock;
// socket to node's rep sync sock (also used for initialization)
zmq::socket_t *node_sync_req_sock;
// socket to sync pub sock of node on our machine (also used for initialization)
zmq::socket_t *node_sub_sock;

// construct some protobuf messages here so we don't call constructor needlessly
antixtransfer::control_message sense_req_msg;

void
shutdown() {
	cout << "Received shutdown message from node. Shutting down..." << endl;
	exit(0);
}

void
synchronize_sub_sock() {
	antixtransfer::node_master_sync sync_msg;
	sync_msg.set_my_id( my_id );

	string s;
	while (s != "cli_sync")
		s = antix::recv_str(node_sub_sock);
	antix::recv_blank(node_sub_sock);

	antix::send_pb(node_sync_req_sock, &sync_msg);
	antix::recv_blank(node_sync_req_sock);
}

/*
	Node has sent us the following sense data with at least one robot
	Decide what to do and send a response

  Decision logic from rtv's Antix

	XXX Right now we can only do setspeed OR pickup OR drop
*/
void
controller(zmq::socket_t *node, antixtransfer::sense_data *sense_msg) {
	// Message that gets sent as a request containing multiple robots
	antixtransfer::control_message control_msg;
	control_msg.set_team( my_id );

	// For each robot in the sense data from this node, build a decision
	for (int i = 0; i < sense_msg->robot_size(); i++) {
		const double x = sense_msg->robot(i).x();
		const double y = sense_msg->robot(i).y();
		const double a = sense_msg->robot(i).a();
		const double id = sense_msg->robot(i).id();
		double last_x = sense_msg->robot(i).last_x();
		double last_y = sense_msg->robot(i).last_y();
#if DEBUG
		cout << "Got last x " << last_x << " and last y " << last_y << " on turn " << antix::turn << endl;
#endif

		// Add the robot to response we will send
		antixtransfer::control_message::Robot *r = control_msg.add_robot();
		r->set_id( id );
		r->set_last_x( last_x );
		r->set_last_y( last_y );

		double heading_error(0.0);
		// distance and angle to home
		const double dx( antix::WrapDistance( my_home->x - x ) );
		const double dy( antix::WrapDistance( my_home->y - y ) );
		const double da( antix::fast_atan2( dy, dx ) );
		const double dist( hypot( dx, dy ) );

		// if this robot is holding a puck
		if (sense_msg->robot(i).has_puck()) {
			// turn towards home
			heading_error = antix::AngleNormalize(da - a);

			// if the robot is some random distance inside home, drop puck
			if (dist < drand48() * my_home->r) {
				r->set_type( antixtransfer::control_message::DROP );
				continue;
			}

		// not holding a puck
		} else {
			bool picking_up = false;
			// if we're away from home and see puck(s)
			if (dist > my_home->r && sense_msg->robot(i).seen_puck_size() > 0) {
				double closest_range(1e9);
				// Look at all the pucks we can see
				for (int j = 0; j < sense_msg->robot(i).seen_puck_size(); j++) {
					const double puck_range = sense_msg->robot(i).seen_puck(j).range();
					const bool puck_held = sense_msg->robot(i).seen_puck(j).held();
					const double puck_bearing = sense_msg->robot(i).seen_puck(j).bearing();

					// If one is within pickup distance, try to pick it up
					if (puck_range < Robot::pickup_range && !puck_held) {
#if DEBUG
						cout << "Trying to pick up a puck" << endl;
#endif
						// remember this location
						r->set_last_x(x);
						r->set_last_y(y);
						r->set_type( antixtransfer::control_message::PICKUP );
						picking_up = true;
#if DEBUG
						cout << "(PICKUP) Sending last x " << r->last_x() << " and last y " << r->last_y() << " on turn " << antix::turn << endl;
#endif
						break;
					}

					// Otherwise see if its the closest we've seen yet
					if (puck_range < closest_range && !puck_held) {
						heading_error = puck_bearing;
						closest_range = puck_range;
					}
				}

				// if we're picking up, message is built. go on to next robot
				if (picking_up)
					continue;

			// we don't see any pucks
			} else {
				const double lx( antix::WrapDistance( last_x - x ) );
				const double ly( antix::WrapDistance( last_y - y ) );

				// go towards last place a puck was picked up (or attempted pick up in
				// the case of this version
				heading_error = antix::AngleNormalize( antix::fast_atan2(ly, lx) - a );

				// if the robot is at the location of last attempted puck, choose random
				if ( hypot( lx, ly ) < 0.05 ) {
					//robots[id].last_x += drand48() * 0.4 - 0.2;
					//robots[id].last_y += drand48() * 0.4 - 0.2;
					last_x += drand48() * 1.0 - 0.5;
					last_y += drand48() * 1.0 - 0.5;
					r->set_last_x( antix::DistanceNormalize( last_x ) );
					r->set_last_y( antix::DistanceNormalize( last_y ) );
				}
			}
		// done not holding puck case
		}

#if DEBUG
		cout << "(SETSPEED) Sending last x " << r->last_x() << " and last y " << r->last_y() << " on turn " << antix::turn << endl;
#endif

		// If we got here, nothing left to try except setspeed
		r->set_type(antixtransfer::control_message::SETSPEED);

		// check if the robot is pointing in correct direction
		if ( fabs(heading_error) < 0.1 ) {
			r->set_v( 0.005 );
			r->set_w( 0.0 );
		} else {
			r->set_v( 0.001 );
			r->set_w( 0.2 * heading_error );
		}
	}

	// send the decision for all of our robots to this node
	antix::send_pb(node, &control_msg);
}

/*
	Request from local node what our robots can see
  Make a decision based on this & send it back
*/
void
sense_and_controller() {
#if DEBUG_SYNC
	cout << "Sync: Requesting sense data from node for my team: " << my_id << "..." << endl;
#endif
	// Ask local node what the robots from our team sees
	antix::send_pb(node_req_sock, &sense_req_msg);

#if DEBUG_SYNC
	cout << "Sync: Awaiting sense data response..." << endl;
#endif
	// Get the sense data back from the node
	// XXX declare once
	antixtransfer::sense_data sense_msg;
	// Get the sense data from every node
	int rc = antix::recv_pb(node_req_sock, &sense_msg, 0);
	assert(rc == 1);

#if DEBUG_SYNC
	cout << "Sync: Got sense data with " << sense_msg.robot_size() << " robots." << endl;
#endif

	// if there's at least one robot in the response, we will be sending a command
	if (sense_msg.robot_size() > 0) {
		controller(node_req_sock, &sense_msg);

#if DEBUG_SYNC
		cout << "Sync: Awaiting responses from node we sent commands to..." << endl;
#endif
		// get response back since REQ sock
		antix::recv_blank(node_req_sock);
	}
#if DEBUG_SYNC
	cout << "Sync: Sensing & controlling done." << endl;
#endif
}

/*
  Look through the list of homes from init response & find our own
*/
Home *
find_our_home(antixtransfer::connect_init_response *init_response) {
	for (int i = 0; i < init_response->home_size(); i++) {
		if (init_response->home(i).team() == my_id)
			return new Home( init_response->home(i).x(), init_response->home(i).y(), home_radius, init_response->home(i).team() );
	}
	return NULL;
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);
	srand( time(NULL) );
	srand48( time(NULL) );

	if (argc != 4) {
		cerr << "Usage: " << argv[0] << " <# of robots> <client id> <node IPC id>" << endl;
		return -1;
	}
	assert(atoi(argv[1]) > 0);
	num_robots = atoi(argv[1]);
	my_id = atoi(argv[2]);
	node_ipc_id = string(argv[3]);

	// initialize some protobufs that do not change
	sense_req_msg.set_team(my_id);

	cout << "Connecting to local node..." << endl;

	// node sync req sock
	while (1) {
		try {
			node_sync_req_sock = new zmq::socket_t(context, ZMQ_REQ);
		} catch (zmq::error_t e) {
			cout << "Error: Node sync req new: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	while (1) {
		try {
			node_sync_req_sock->connect(antix::make_endpoint_ipc(node_ipc_prefix + node_ipc_id + "r"));
		} catch (zmq::error_t e) {
			cout << "Error: Node sync req connect: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// node sync sub sock
	while (1) {
		try {
			node_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
		} catch (zmq::error_t e) {
			cout << "Error: Node sync sub new: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	while (1) {
		try {
			node_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
			node_sub_sock->connect(antix::make_endpoint_ipc(node_ipc_prefix + node_ipc_id + "p"));
		} catch (zmq::error_t e) {
			cout << "Error: Node sync sub connect: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	// node control
	while (1) {
		try {
			node_req_sock = new zmq::socket_t(context, ZMQ_REQ);
		} catch (zmq::error_t e) {
			cout << "Error: Node req new: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	while (1) {
		try {
			node_req_sock->connect(antix::make_endpoint_ipc(node_ipc_prefix + node_ipc_id + "c"));
		} catch (zmq::error_t e) {
			cout << "Error: Node req connect: " << e.what() << endl;
			antix::sleep(1000);
			continue;
		}
		break;
	}

	cout << "Connected to local node. Telling it of our existence..." << endl;

	// Identify ourself & specify num robots we want
	antixtransfer::connect_init_client init_req;
	init_req.set_num_robots( num_robots );
	init_req.set_id( my_id );
	antix::send_pb(node_sync_req_sock, &init_req);

	// Get back blank in response since REQ sock
	antix::recv_blank(node_sync_req_sock);

	// Make sure we are synchronized with node's pub sock
	synchronize_sub_sock();
	
	cout << "Waiting for signal for simulation begin..." << endl;

	// Wait for response containing simulation params / home location
	// This also indicates simulation begin
	antixtransfer::connect_init_response init_response;
	// we may get messages on sub sock from node syncing other clients. ignore
	string s;
	while (s != "cli_begin")
		s = antix::recv_str(node_sub_sock);

	int rc = antix::recv_pb(node_sub_sock, &init_response, 0);
	assert(rc == 1);
	home_radius = init_response.home_radius();
	sleep_time = init_response.sleep_time();
	Robot::pickup_range = init_response.pickup_range();
	antix::world_size = init_response.world_size();

	// set our own home
	my_home = find_our_home(&init_response);
	assert(my_home != NULL);

	cout << "Beginning simulation..." << endl;

	// response from node on sync sock
	string response;

	// enter main loop
	while (1) {
		// sense, then decide & send what commands for each robot
		sense_and_controller();

		response = antix::wait_for_next_turn(node_sync_req_sock, node_sub_sock, my_id, antixtransfer::done::CLIENT);
		if (response == "s")
			shutdown();

#if DEBUG
		antix::turn++;
#endif

#if SLEEP
		antix::sleep(sleep_time);
#endif
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
