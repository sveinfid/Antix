#include <iostream>
#include <fstream>
#include <string>
#include "antix.pb.h"
using namespace std;



int main(int argc, char* argv[]) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc != 2) {
    cerr << "Usage:  " << argv[0] << " ADDRESS_BOOK_FILE" << endl;
    return -1;
  }

  antixtransfer::RobotRequestMap rrm;

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


  {
    // Write the new address book back to disk.
    fstream output(argv[1], ios::out | ios::trunc | ios::binary);
    if (!rrm.SerializeToOstream(&output)) {
      cerr << "Failed to write rrm." << endl;
      return -1;
    }
  }

  // Optional:  Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
