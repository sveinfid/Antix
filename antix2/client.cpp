/*
	Client connects to master to get a list of nodes
	and then connects to all nodes
*/

#include <map>
#include "antix.cpp"

using namespace std;

string master_host;
string master_client_port = "7771";
string master_pub_port = "7773";
string client_node_port = "7774";

int my_id;
int sleep_time;
int num_robots;

map<int, zmq::socket_t *> node_map;
vector<CRobot> robots;

// talk to master on this socket
zmq::socket_t *master_req_sock;
// receive list of nodes on this socket
zmq::socket_t *master_sub_sock;

/*

*/
int
choose_random_node() {
	return rand() % node_map.size();
}

/*
	Generate one message with n robots, send to a random node
	XXX
*/
void
generate_robots (int n) {
	int node = choose_random_node();
	antixtransfer::control_message msg;
	msg.set_team(my_id);
	msg.set_type(antixtransfer::control_message::ADD_BOT);
	for (int i = 0; i < n; i++) {
		CRobot r(i, node);
		robots.push_back(r);
		antixtransfer::control_message::Robot *r_pb = msg.add_robot();
		r_pb->set_id(i);
	}

	antix::send_pb(node_map[node], &msg);

	// We must receive a message in response due to REQ socket
	antix::recv_blank(node_map[node]);
	cout << "Created " << n << " robots on " << node << endl;
}

/*
	Get map data from nodes for each of our robots
	TODO
	Right now this just requests the entire map from each node
*/
void
update_map_data() {
	for (map<int, zmq::socket_t *>::iterator it = node_map.begin(); it != node_map.end(); it++) {
		// XXX right now just request whole map from each node
		antixtransfer::control_message msg;
		msg.set_type(antixtransfer::control_message::SENSE);
		msg.set_team(my_id);
		antix::send_pb(it->second, &msg);
	}
	// Separate loops so that all the messages go out at once
	for (map<int, zmq::socket_t *>::iterator it = node_map.begin(); it != node_map.end(); it++) {

		// Receive map back, but do nothing with it right now
		antixtransfer::SendMap map;
		antix::recv_pb(it->second, &map, 0);
		cout << "Got " << map.robot_size() << " robots and " << map.puck_size() << " pucks from node " << it->first << endl;
	}
}

void
update_senses() {
	// XXX cpu work
}

void
controller() {
	// XXX
	// Say for each robot we want to SETSPEED & PICKUP _or_ DROP
}

/*
	Map nodes sockets by id & connect to all nodes
*/
map<int, zmq::socket_t *>
get_node_map(zmq::context_t *context, antixtransfer::Node_list *node_list) {
	map<int, zmq::socket_t*> node_map;
	antixtransfer::Node_list::Node *node;
	for (int i = 0; i < node_list->node_size(); i++) {
		node = node_list->mutable_node(i);
		zmq::socket_t *client_node_sock = new zmq::socket_t(*context, ZMQ_REQ);
		client_node_sock->connect(antix::make_endpoint(node->ip_addr(), node->control_port()));
		node_map.insert(pair<int, zmq::socket_t *> (node->id(), client_node_sock));
	}
	return node_map;
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);

	if (argc != 3) {
		cerr << "Usage: " << argv[0] << " <IP of master> <# of robots>" << endl;
		return -1;
	}
  master_host = string(argv[1]);
	assert(atoi(argv[2]) > 0);
	num_robots = atoi(argv[2]);

	// REQ socket to master_cli port
  cout << "Connecting to master..." << endl;
  master_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	master_req_sock->connect(antix::make_endpoint(master_host, master_client_port));
  antix::send_str(master_req_sock, "client");
	
	// Response from master contains simulation settings & our unique id (team id)
	antixtransfer::MasterServerClientInitialization init_response;
	antix::recv_pb(master_req_sock, &init_response, 0);
	my_id = init_response.id();

	// Subscribe to master's publish socket. A node list will be received
	master_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
	master_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_sub_sock->connect(antix::make_endpoint(master_host, master_pub_port)); 
	// block until receipt of list of nodes indicating simulation beginning
	antixtransfer::Node_list node_list;
	antix::recv_pb(master_sub_sock, &node_list, 0);
	cout << "Received nodes from master" << endl;
	antix::print_nodes(&node_list);

	node_map = get_node_map(&context, &node_list);
	cout << "Connected to all nodes" << endl;
	
	// generate robots
	// message each node to tell it about the robot to be created on it
	generate_robots(num_robots);

	// enter main loop
	while (1) {
		// request map data for our robots
		//update_map_data();

		// update what each robot can see
		//update_senses();

		// decide & send what commands for each robot
		//controller();

		// sleep
		antix::sleep(sleep_time);
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
