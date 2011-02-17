/****
   client.cc
   Each client would be in control of 1 team
   Gordon, Wendy, Jerome
   
****/

#include <vector>

double Robot::pickup_range( Robot::range/5.0 );
double Robot::radius(0.01);
double Robot::range( 0.1 );
zmq::socket_t *database_sub_sock;
zmq::socket_t *database_pub_sock;

string host = "localhost";
string client_port;
string database_port;
//We’ll use 50001~51000 for the clients

Client::Client(Home* home)
:     home(home)
{
}

Robot::Robot(const Pose& pose )
 :    pose(pose),
      speed(),
      see_robots(),
      see_pucks(),
      puck_held(NULL)
  {
    // add myself to the static vector of all robots
         
    if( ! first )
       first = this;
  }

void Client::AddRobot(Robot r){
    robots.push_back(r);
}

void Client::RemoveRobot(Robot* r){
    //first find robot r in the vector list robots
    //erase from vector using index
}

void Client::Update(){
    //for each robot
        int a;
        //updatesensors (get the local map)
		FOR_EACH( r, robots ){
          	a = RequestLocalMap(r);
       		if (a == 2) RemoveRobot(r);
       	}
        
       
        //calculate move
        FOR_EACH( r, robots )
          (r*)->Controller();

        //request movement from database

FOR_EACH( r, robots )
          RequestUpdatePose(r);
       
    //sleep()
    if( sleep_msec > 0 )
       usleep( sleep_msec * 1e3 );   
}

void Client::RequestLocalMap(Robot* r){
    // retrieve x and y position of the robot r

    database.requestlocalmap(x, y);
    // value returned by requestlocalmap must be put into seeRobot and seePucks vector,

// extra calculation is needed if database return grid


}

void Client::RequestUpdatePose(Robot* r){
   
    // first calculate where to move next (new x and y position)

// and sent new pose to database

    database.requestmovement(currentx, currenty, newx, newy, team)
    // depending on the value return from requestmovement():

// 0=NoMovement, 1=SuccessMovement, 2=Success&Transfer

// remove robot if return value is 2

}

static void	send_pb(zmq::socket_t *socket, google::protobuf::Message *pb_obj) {
		string pb_as_str;
		pb_obj->SerializeToString(&pb_as_str);

		zmq::message_t msg( pb_as_str.size() + 1 );
		memcpy( msg.data(), pb_as_str.c_str(), pb_as_str.size() + 1);

		socket->send(msg);
}

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
	
static void send_pb_envelope(zmq::socket_t *sock, google::protobuf::Message *pb_obj, string address) {
		zmq::message_t type(address.size() + 1);
		memcpy(type.data(), address.c_str(), address.size() + 1);
		sock->send(type, ZMQ_SNDMORE);
		send_pb(sock, pb_obj);
}

int main( int argc, char* argv[] )
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    zmq::context_t context(1);   
   
    if (argc != 2) {
           cerr << "Usage: " << argv[0] << "<client number>" << endl;
          return -1;
      }

    client_number = argv[1];
    
    //Connect to database
    database_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
    database_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	database_sub_sock->connect("ipc:///database/" + client_number);

    database_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	database_req_sock->bind("ipc:///client/" + client_number);

	//Initialize client
    intialize();

	antixtransfer::remove_robot rm_bot;
	Robot r;
    
    //Main Loop
    while(1){
        while (database_sub_sock->recv(&type_msg, ZMQ_NOBLOCK) == 1){
            recv_pb(database_sub_sock, &rm_bot, 0);
            Robot::robot();
            AddRobot(r)
           } 
        
        Update();
        Sleep();        

	}
return 0;
}
