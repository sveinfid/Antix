/*
	Client connects to a node on its local machine & gets simulation parameters
	Then it begins controlling robots
*/

#define IS_CLIENT

#include <map>
#include <dlfcn.h>
#include "controller.cpp"

using namespace std;

Controller *ctlr;

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
antixtransfer::control_message control_msg;
antixtransfer::sense_data sense_msg;
// used for wait_for_next_turn()
antixtransfer::done node_done_msg;

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
*/
void
controller(zmq::socket_t *node, antixtransfer::sense_data *sense_msg) {
	// Message that gets sent as a request containing multiple robots
	control_msg.clear_robot();

	// For each robot in the sense data from this node, build a decision
	int robot_size = sense_msg->robot_size();
	for (int i = 0; i < robot_size; i++) {
		// First we create a Controller with this robot's state

		// XXX
		// we copy seen_pucks to a vector to give a nicer interface
		// this costs cpu... perhaps undesirable. but otherwise controller
		// must look at protobuf...
		ctlr->seen_pucks.clear();
		int seen_puck_size = sense_msg->robot(i).seen_puck_size();
		for (int j = 0; j < seen_puck_size; j++) {
			ctlr->seen_pucks.push_back(
				CSeePuck(sense_msg->robot(i).seen_puck(j).held(),
					sense_msg->robot(i).seen_puck(j).range(),
					sense_msg->robot(i).seen_puck(j).bearing())
			);
		}

		ctlr->puck_action = PUCK_ACTION_NONE;
		ctlr->x = sense_msg->robot(i).x();
		ctlr->y = sense_msg->robot(i).y();
		ctlr->a = sense_msg->robot(i).a();
		ctlr->id = sense_msg->robot(i).id();
		ctlr->last_x = sense_msg->robot(i).last_x();
		ctlr->last_y = sense_msg->robot(i).last_y();
		ctlr->home = my_home;
		ctlr->has_puck = sense_msg->robot(i).has_puck();

#if DEBUG
		cout << "Running controller for robot " << ctlr->id << " on turn " << antix::turn << endl;
		cout << " at " << ctlr->x << ", " << ctlr->y << " and with ";
		cout << ctlr->last_x << ", " << ctlr->last_y << " as last_x/y" << endl;
#endif

		ctlr->v = 0.0;
		ctlr->w = 0.0;

		ctlr->doubles.clear();
		ctlr->ints.clear();

		ctlr->collided = sense_msg->robot(i).collided();

		// Update this robot's memory
		int doubles_size = sense_msg->robot(i).doubles_size();
		for (int j = 0; j < doubles_size; j++) {
			ctlr->doubles.push_back( sense_msg->robot(i).doubles(j) );
		}
		int ints_size = sense_msg->robot(i).ints_size();
		for (int j = 0; j < ints_size; j++) {
			ctlr->ints.push_back( sense_msg->robot(i).ints(j) );
		}

#if DEBUG
		cout << "Got last x " << sense_msg->robot(i).last_x();
		cout << " and last y " << sense_msg->robot(i).last_y();
		cout << " on turn " << antix::turn << endl;
#endif

		// then run the controller
		ctlr->controller();

		// then add robot's new data to response protobuf
		antixtransfer::control_message::Robot *r = control_msg.add_robot();
		r->set_id( ctlr->id );
		r->set_last_x( ctlr->last_x );
		r->set_last_y( ctlr->last_y );
		r->set_v( ctlr->v );
		r->set_w( ctlr->w );

		if (ctlr->puck_action == PUCK_ACTION_PICKUP)
			r->set_puck_action(antixtransfer::control_message::PICKUP);
		else if (ctlr->puck_action == PUCK_ACTION_DROP)
			r->set_puck_action(antixtransfer::control_message::DROP);
		else
			r->set_puck_action(antixtransfer::control_message::NONE);

		vector<double>::const_iterator doubles_end = ctlr->doubles.end();
		for (vector<double>::const_iterator it = ctlr->doubles.begin(); it != doubles_end; it++) {
			r->add_doubles( *it );
		}
		vector<int>::const_iterator ints_end = ctlr->ints.end();
		for (vector<int>::const_iterator it = ctlr->ints.begin(); it != ints_end; it++) {
			r->add_ints( *it );
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

	if (argc != 5) {
		cerr << "Usage: " << argv[0] << " <# of robots> <client id> <node IPC id> <AI library.so>" << endl;
		return -1;
	}
	assert(atoi(argv[1]) > 0);
	num_robots = atoi(argv[1]);
	my_id = atoi(argv[2]);
	node_ipc_id = string(argv[3]);
	string ai_library = string(argv[4]);

	// If non absolute path, assume cwd for AI.so path
	if (ai_library[0] != '/')
		ai_library = "./" + ai_library;

	// Load AI dynamically
	// From http://stackoverflow.com/questions/496664/c-dynamic-shared-library-on-linux
	void* handle = dlopen(ai_library.c_str(), RTLD_NOW | RTLD_GLOBAL);
	if (handle == NULL) {
		cerr << "Error: could not load AI library: " << dlerror() << endl;
		exit(-1);
	}
	Controller* (*create)();
	void (*destroy)(Controller*);
	create = (Controller* (*)())dlsym(handle, "create_object");
	destroy = (void (*)(Controller*))dlsym(handle, "destroy_object");
	ctlr = (Controller*) create();

	// initialize some protobufs that do not change
	sense_req_msg.set_team(my_id);
	control_msg.set_team(my_id);
	node_done_msg.set_my_id(my_id);
	node_done_msg.set_type( antixtransfer::done::CLIENT );

	cout << "Connecting to local node..." << endl;

	string node_ipc_prefix = IPC_PREFIX;

	// node sync req sock
	while (1) {
		try {
			node_sync_req_sock = new zmq::socket_t(context, ZMQ_REQ);
		} catch (zmq::error_t e) {
			cout << "Error: Node sync req new: " << e.what() << endl;
			delete node_sync_req_sock;
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
			delete node_sub_sock;
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
			delete node_req_sock;
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
	ctlr->set_static_vars(antix::world_size, Robot::pickup_range);

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

		// XXX it's possible we should use a different function than this
		// as this includes score data definition (done msg) for node which
		// may waste cpu depending on protobuf impl.
		response = antix::wait_for_next_turn(node_sync_req_sock, node_sub_sock, &node_done_msg);
		if (response == "s")
			// leave loop
			break;

#if DEBUG
		antix::turn++;
#endif

#if SLEEP
		antix::sleep(sleep_time);
#endif
	}

	cout << "Received shutdown message from node. Shutting down..." << endl;

	delete my_home;
	delete node_sync_req_sock;
	delete node_sub_sock;
	delete node_req_sock;

	destroy(ctlr);
	dlclose(handle);

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
