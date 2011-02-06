/*
	Node / map piece server

	The majority of the ZMQ stuff is based on examples from the ZMQ guide:
	http://zguide.zeromq.org/chapter:all
*/

#include "antix.cpp"

using namespace std;

string master_host = "localhost";
string master_node_port = "7770";
string master_publish_port = "7773";

int my_id;
double world_size;
double my_min_x;
// where neighbour begins
double my_max_x;
int sleep_time;
int initial_puck_amount;
antixtransfer::Node_list::Node left_node;
antixtransfer::Node_list::Node right_node;
vector<Puck> pucks;
vector<Robot> robots;
vector<Puck> foreign_pucks;
vector<Robot> foreign_robots;

zmq::socket_t *send_to_neighbour_sock;

/*
	Find our offset in node_list and set my_min_x, my_max_x
*/
void
set_dimensions(antixtransfer::Node_list *node_list) {
	antixtransfer::Node_list::Node *node;
	double offset_size = world_size / node_list->node_size();

	for (int i = 0; i < node_list->node_size(); i++) {
		node = node_list->mutable_node(i);
		if (node->id() == my_id) {
			my_min_x = node->x_offset();
			my_max_x = my_min_x + offset_size;
			return;
		}
	}
}

/*
	Place pucks randomly within our region
*/
void
generate_pucks() {
	for (int i = 0; i < initial_puck_amount; i++) {
		pucks.push_back( Puck(my_min_x, my_max_x, world_size) );
	}
}

/*
	Given a map which may or may not contain entities, add any entities therein
	to our internal records of foreign robots & pucks
*/
void
update_foreign_entities(antixtransfer::SendMap *map) {
	// foreign robots
	for (int i = 0; i < map->robot_size(); i++) {
		foreign_robots.push_back( Robot( map->robot(i).x(), map->robot(i).y(), map->robot(i).id(), map->robot(i).team() ) );
	}
	// foreign pucks
	for (int i = 0; i < map->puck_size(); i++) {
		foreign_pucks.push_back( Puck( map->puck(i).x(), map->puck(i).y(), map->puck(i).held() ) );
	}
}

/*
	Send entities which are within sight distance of our left border
	and within sight distance of our right border
*/
void
send_border_entities(zmq::socket_t *send_to_neighbour_sock) {
	// container for robots & pucks
	antixtransfer::SendMap send_map;
	// XXX Right now we only have one big list of robots, so all are sent
	for(vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		antixtransfer::SendMap::Robot *robot = send_map.add_robot();
		robot->set_x( it->x );
		robot->set_y( it->y );
		// not implemented
		robot->set_team(0);
	}

	// XXX only one list of pucks right now
	for(vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		antixtransfer::SendMap::Puck *puck = send_map.add_puck();
		puck->set_x( it->x );
		puck->set_y( it->y );
		puck->set_held( it->held );
	}
	cout << "Sending " << send_map.puck_size() << " pucks and " << send_map.robot_size() << " robots." << endl;

	antix::send_pb(send_to_neighbour_sock, &send_map);
}

/*
	Clear old foreign robots/pucks and attempt to re-fill based on msgs from our
	neighbours
*/
void
recv_border_entities(zmq::socket_t *neighbour_publish_sock) {
	foreign_robots.clear();
	foreign_pucks.clear();

	// We expect two messages: One from left, one from right
	antixtransfer::SendMap map_foreign_1;
	antix::recv_pb(neighbour_publish_sock, &map_foreign_1, ZMQ_NOBLOCK);
	//antix::recv_pb(neighbour_publish_sock, &map_foreign_1, 0);
	cout << "Received " << map_foreign_1.puck_size() << " pucks and " << map_foreign_1.robot_size() << " robots from neighbour 1" << endl;
	update_foreign_entities(&map_foreign_1);

	antixtransfer::SendMap map_foreign_2;
	antix::recv_pb(neighbour_publish_sock, &map_foreign_2, ZMQ_NOBLOCK);
	//antix::recv_pb(neighbour_publish_sock, &map_foreign_2, 0);
	cout << "Received " << map_foreign_2.puck_size() << " pucks and " << map_foreign_2.robot_size() << " robots from neighbour 2" << endl;
	update_foreign_entities(&map_foreign_2);
}

/*
	Remove given robot from local robots
*/
void
remove_robot(Robot *r) {
	for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		if (it->id == r->id && it->team == r->team) {
			robots.erase(it);
		}
	}
}

/*
	Robot has been found to be outside of our map portion
	Transfer the robot (and possibly its puck) to the given node
	Remove robot (and puck) from our local list
*/
void
move_robot(Robot *r, antixtransfer::Node_list::Node *node) {
	// transfer the robot
	antixtransfer::RequestRobotTransfer transfer_msg;
	transfer_msg.set_id(r->id);
	transfer_msg.set_team(r->team);
	transfer_msg.set_x(r->x);
	transfer_msg.set_y(r->y);
	transfer_msg.set_a(r->a);
	transfer_msg.set_v(r->v);
	transfer_msg.set_w(r->w);
	transfer_msg.set_has_puck(r->has_puck);

	// XXX using the PUB socket has potential to lose robots...?
	antix::send_pb(send_to_neighbour_sock, &transfer_msg);

	// remove from our local list
	// XXX do this
}

/*
	update the pose of a single robot
	Taken from rtv's Antix
*/
void
update_pose(Robot r) {
	double dx = r.v * cos(r.a);
	double dy = r.v * sin(r.a);
	double da = r.w;

	r.x = antix::DistanceNormalize(r.x + dx, world_size);
	r.y = antix::DistanceNormalize(r.y + dy, world_size);
	r.a = antix::AngleNormalize(r.a + da);

	// If we're holding a puck, it must move also
	if (r.has_puck) {
		r.puck->x = r.x;
		r.puck->y = r.y;
	}

	// XXX deal with collision

	// if our new location is in another node, move robot
	if (r.x < my_min_x)
		move_robot(&r, &left_node);
	else if (r.x >= my_max_x)
		move_robot(&r, &right_node);
}

/*
	Go through our local robots & update their poses
*/
void
update_poses() {
	// For each robot, update its pose
	for(vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		update_pose(*it);
	}
}

void
add_robot() {

}

int main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	
	if (argc != 4) {
		cerr << "Usage: " << argv[0] << " <IP to listen on> <node-node port> <client-node port>" << endl;
		return -1;
	}

	zmq::context_t context(1);

	// socket to announce ourselves to master on
	zmq::socket_t node_master_sock(context, ZMQ_REQ);
	node_master_sock.connect(antix::make_endpoint(master_host, master_node_port));
	cout << "Connecting to master..." << endl;

	zmq::socket_t master_publish_sock(context, ZMQ_SUB);
	// subscribe to all messages on this socket: should just be a list of nodes
	master_publish_sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock.connect(antix::make_endpoint(master_host, master_publish_port));

	// send message announcing ourself. includes our own IP
	cout << "Sending master our existence notification..." << endl;

	// create & send pb msg
	antixtransfer::connect_init_node pb_init_msg;
	pb_init_msg.set_ip_addr( string(argv[1]) );
	pb_init_msg.set_neighbour_port( string(argv[2]) );
	pb_init_msg.set_client_port( string(argv[3]) );
	antix::send_pb(&node_master_sock, &pb_init_msg);

	// receive message back stating our unique ID
	antixtransfer::connect_init_response init_response;
	antix::recv_pb(&node_master_sock, &init_response, 0);
	my_id = init_response.id();
	world_size = init_response.world_size();
	sleep_time = init_response.sleep_time();
	// XXX need to add this in master
	// initial_puck_amount = init_response.puck_amount();
	initial_puck_amount = 10;
	cout << "We are now node id " << my_id << endl;

	// receive node list
	// blocks until master publishes list of nodes
	antixtransfer::Node_list node_list;
	antix::recv_pb(&master_publish_sock, &node_list, 0);
	cout << "Received list of nodes:" << endl;
	antix::print_nodes(&node_list);

	if (node_list.node_size() < 3) {
		cout << "Error: we need at least 3 nodes. Only received " << node_list.node_size() << " node(s)." << endl;
		return -1;
	}

	// calculate our min / max x from the offset assigned to us in node_list
	set_dimensions(&node_list);

	// find our left/right neighbours
	antix::set_neighbours(&left_node, &right_node, &node_list, my_id);
	cout << "Left neighbour id: " << left_node.id() << " " << left_node.ip_addr() << " neighbour port " << left_node.neighbour_port() << endl;
	cout << "Right neighbour id: " << right_node.id() << " " << right_node.ip_addr() << " neighbour port " << right_node.neighbour_port() << endl;

	// connect & subscribe to both neighbour's PUB sockets
	// this socket will receive foreign entities that are near our border
	zmq::socket_t neighbour_publish_sock(context, ZMQ_SUB);
	// XXX subscribe only to messages intended for us?
	// as the neighbours will be publishing data for both of its neighbours
	// on the same port
	// right now we receive all messages
	neighbour_publish_sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);

	// connect to the neighbours on this socket
	neighbour_publish_sock.connect(antix::make_endpoint(left_node.ip_addr(), left_node.neighbour_port()));
	neighbour_publish_sock.connect(antix::make_endpoint(right_node.ip_addr(), right_node.neighbour_port()));
	cout << "Connected to neighbour on left: " << left_node.ip_addr() << ":" << left_node.neighbour_port() << endl;
	cout << "Connected to neighbour on right: " << right_node.ip_addr() << ":" << right_node.neighbour_port() << endl;

	// open PUB socket neighbours where we publish entities close to the borders
	send_to_neighbour_sock = new zmq::socket_t(context, ZMQ_PUB);
	send_to_neighbour_sock->bind(antix::make_endpoint( argv[1], argv[2] ));

	// create REP socket that receives messages from clients
	// (add_bot, sense, setspeed, pickup, drop)
	zmq::socket_t client_sock(context, ZMQ_REP);
	client_sock.bind(antix::make_endpoint( argv[1], argv[3] ));

	// generate pucks
	generate_pucks();
	cout << "Created " << pucks.size() << " pucks." << endl;
	for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		cout << "Puck at " << it->x << "," << it->y << endl;
	}

	// enter main loop
	while (1) {
		// send entities within sight_range of our borders to our neighbours
		send_border_entities(send_to_neighbour_sock);
		cout << "Sent border entities to neighbours." << endl;

		// read from our neighbour SUB socket & update our foreign entity knowledge
		recv_border_entities(&neighbour_publish_sock);
		cout << "Current foreign entities: " << endl;
		for (vector<Robot>::iterator it = foreign_robots.begin(); it != foreign_robots.end(); it++)
			cout << "Robot at " << it->x << ", " << it->y << endl;
		for (vector<Puck>::iterator it = foreign_pucks.begin(); it != foreign_pucks.end(); it++)
			cout << "Puck at " << it->x << ", " << it->y << endl;

		// update poses for internal robots
		update_poses();
		cout << "Poses updated for all robots." << endl;

		// service client messages on REP socket
		// (add_bot, sense, setspeed, pickup, drop)

		// sleep
		// code from rtv's Antix
		usleep( sleep_time * 1e3);
	}

	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
