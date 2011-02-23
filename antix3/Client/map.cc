/* Dummy Map.cc for testing */

static int recv_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj, int flags) {
	zmq::message_t msg;
	int retval = socket->recv(&msg, flags);

	// If we did a non blocking call, it's possible we don't actually have a msg
	// But this return code doesn't match the ZMQ docs...
	if (retval != 1)
		return retval;

	char raw_pb[msg.size()];
	memcpy(raw_pb, msg.data(), msg.size());

	// make a string out of the raw bytes
	string s;
	s.assign(raw_pb, msg.size() + 1);

	pb_obj->ParseFromString(s);
	return retval;
}

static void	send_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj) {
		string pb_as_str;
		pb_obj->SerializeToString(&pb_as_str);

		zmq::message_t msg( pb_as_str.size() + 1 );
		memcpy( msg.data(), pb_as_str.c_str(), pb_as_str.size() + 1);

		socket->send(msg);
}


static void	send_pb_envelope(zmq::socket_t *sock, google::protobuf::Message *pb_obj, string address) {
	zmq::message_t type(address.size() + 1);
	memcpy(type.data(), address.c_str(), address.size() + 1);
	sock->send(type, ZMQ_SNDMORE);
	send_pb(sock, pb_obj);
}

static string recv_str(zmq::socket_t *sock) {
	zmq::message_t msg;
	sock->recv(&msg);
	return string( (char *) msg.data() );
}

static void handle_map_request(zmq::socket_t *nodes_socket){
	antixtransfer::request_map req_map;
	recv_pb(nodes_socket, &req_map, 0);
		
	//get map();
	//antixtransfer::SendMap2	
}



int main(int argc, char* argv[])
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);

	

	if (argc != 2) {
           cerr << "Usage: " << argv[0] << "<amount of clients>" << endl;
          return -1;
    }

	number_of_clients = argv[1];

	zmq::socket_t database_pub_sock[number_of_clients];	
	zmq::socket_t database_rep_sock[number_of_clients];

	//Connect to all the clients
	for (int i = 0; int i < number_of_clients; i++){
		database_pub_sock[i] = zmq::socket_t(context, ZMQ_PUB);
		database_pub_sock[i] -> bind("ipc:///database/" + client_number);
		
		database_rep_sock[i] = zmq::socket_t(context, ZMQ_REP);
		database_rep_sock[i] -> connect("ipc:///client/" + client_number);
	}
	
	//Main Loop
	while(1){
		for (int i = 0; int i < number_of_clients; i++){
			while (database_rep_sock[i]->recv(&type_msg, ZMQ_NOBLOCK) == 1){ //Not Fair
				string type = recv_str(&database_rep_sock[i]);
				if (type == "map"){
					handle_map_request(&database_rep_sock[i]);
				}
				if (type == "move"){
					antixtransfer::request_map req_map;
					recv_pb(database_rep_sock[i], &req_map, 0);

					antixtransfer::robot_move_ack reply;
	
					reply.set_type = 2;
	
					send_pb(database_rep_sock[i], &reply)
				}
			}			
		}
		
		//sleep()
    	if( sleep_msec > 0 ) usleep( sleep_msec * 1e3 );  
	}

}