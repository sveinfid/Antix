#include <iostream>
#include <fstream>
#include <string>
#include "antix.pb.h"
#include "antix.cpp"
using namespace std;

string master_host = "localhost";
string master_publish_port = "7773";
string master_node_port = "7774";

int main(int argc, char* argv[]) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

/*
  if (argc != 2) {
    cerr << "Usage:  " << argv[0] << "FILE" << endl;
    return -1;
  }
*/
  antixtransfer::RobotRequestMap rrm;

/*
  {
    // Read the existing address book.
    fstream input(argv[1], ios::in | ios::binary);
    if (!input) {
      cout << argv[1] << ": File not found.  Creating a new file." << endl;
    } else if (!rrm.ParseFromIstream(&input)) {
      cerr << "Failed to parse rrm." << endl;
      return -1;
    }
  }
*/

  cout << "robotid: " << rrm.robotid() << endl;
  cout << "robotx: " << rrm.robotx() << endl;
  cout << "roboty: " << rrm.roboty() << endl;
  cout << "robotfacing: " << rrm.robotfacing() << endl;


  
     cout << "Enter robotid: ";
     int robotid;
     cin >> robotid;
   	 rrm.set_robotid(robotid);

     cout << "Enter robotx: ";
     double robotx;
     cin >> robotx;
   	 rrm.set_robotx(robotx);

     cout << "Enter roboty: ";
     double roboty;
     cin >> roboty;
   	 rrm.set_roboty(roboty);

     cout << "Enter robotfacing: ";
     double robotfacing;
     cin >> robotfacing;
   	 rrm.set_robotfacing(robotfacing);

	/*send rrm to master using protobuf*/

	zmq::context_t context(1);
	zmq::socket_t node_master_sock(context, ZMQ_REQ);
	node_master_sock.connect(antix::make_endpoint(master_host, master_node_port));
	cout << "connecting to master..." << endl;
	zmq::socket_t master_publish_sock(context, ZMQ_SUB);
	// subscribe to all messages on this socket: should just be a list of nodes
	master_publish_sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_publish_sock.connect(antix::make_endpoint(master_host, master_publish_port));

	cout << "sending a robot to master..." << endl;	
	antix::send_pb(&node_master_sock, &rrm);
/*

  {
    // Write the new address book back to disk.
    fstream output(argv[1], ios::out | ios::trunc | ios::binary);
    if (!rrm.SerializeToOstream(&output)) {
      cerr << "Failed to write rrm." << endl;
      return -1;
    }
  }
*/
  // Optional:  Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
