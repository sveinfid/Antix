/*
	Client connects to master to get a list of nodes
	and then connects to all nodes
*/

#include <map>
#include "antix.cpp"


using namespace std;

string master_host = "localhost";
string master_client_port = "7771";
string master_publish_port = "7773";
string client_node_port = "7774";

int my_id;
int sleep_time;
int num_robots;

map<int, zmq::socket_t *> node_map;
vector<CRobot> robots;

int
choose_random_node() {
	// TODO. right now selects first node in map always
	for (map<int, zmq::socket_t *>::iterator it = node_map.begin(); it != node_map.end(); it++)
		return it->first;
}

/*
	Right now only creates one robot per turn as we must wait for response from
	the node. This can be avoided if we create all robots at once in one node,
	or create one per node per turn.
*/
void
generate_robot (int id) {
	int node = choose_random_node();
	CRobot r(id, node);
	robots.push_back(r);
	// send an ADD_BOT message to the node the new robot is on
	antixtransfer::control_message msg;
	msg.set_team(my_id);
	msg.set_type(antixtransfer::control_message::ADD_BOT);
	antixtransfer::control_message::Robot *r_pb = msg.add_robot();
	r_pb->set_id(id);

	antix::send_pb(node_map[node], &msg);

	// We must receive a message in response due to REQ socket
	antix::recv_blank(node_map[node]);
	cout << "Created robot with id " << id << " on " << node << endl;
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
		node_map.insert(pair<int, zmq::socket_t*> (node->id(), client_node_sock));
	}
	return node_map;
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	if (argc != 3) {
		cerr << "Usage: " << argv[0] << " <IP to listen on> <# of robots>" << endl;
		return -1;
	}
	// # of robots
	assert(atoi(argv[2]) > 0);
	num_robots = atoi(argv[2]);

	zmq::context_t context(1);

	// REQ socket to master_cli port
	zmq::socket_t client_master_sock(context, ZMQ_REQ);
	// send message giving our IP
	printf("Connecting to master...\n");
	client_master_sock.connect(antix::make_endpoint(master_host, master_client_port));
	antixtransfer::connect_init_client init_connect;
	init_connect.set_ip_addr(string(argv[1]));
	antix::send_pb(&client_master_sock, &init_connect);
	
	// Response from master contains simulation settings & our unique id (team id)
	antixtransfer::MasterServerClientInitialization init_response;
	antix::recv_pb(&client_master_sock, &init_response, 0);
	my_id = init_response.id();

	// Subscribe to master's publish socket. A node list will be received
	zmq::socket_t *master_publish_sock = new zmq::socket_t(context, ZMQ_SUB);
	master_publish_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock->connect(antix::make_endpoint(master_host, master_publish_port));

	// block until receipt of list of nodes indicating simulation beginning
	antixtransfer::Node_list node_list;
	antix::recv_pb(master_publish_sock, &node_list, 0);
	cout << "Received nodes from master" << endl;
	antix::print_nodes(&node_list);

	node_map = get_node_map(&context, &node_list);
	cout << "Connected to all nodes" << endl;
	
	// generate robots
	// message each node to tell it about the robot to be created on it
	for (int i = 0; i < num_robots; i++) {
		generate_robot(i);
	}

	// enter main loop
	while (1) {
		// request map data for our robots
		update_map_data();

		// update what each robot can see
		update_senses();

		// decide & send what commands for each robot
		controller();

		// sleep
		antix::sleep(sleep_time);
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
