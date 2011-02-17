 
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <sys/types.h>
#include "./hiclient/hiredis/hiredis.h"
#include "./hiclient/hiredis/sds.h"
#include <string>

using namespace std;

class database
{
  public:
    redisContext * db;
    //setup connection
    database();
    //basic command 
    bool setCommand(string key, string value);
    string getCommand(string key);
    bool checkKeyExist(string key);
    void updatePos(string oldkey, string newkey);
    
    //game command
    string getSubMap(double x, double y);
    bool moveRobot(double oldX, double oldY, double newX, double newY, int robotID);
    bool addRobot(double x, double y, int robotID);
    
    //helper function
    string constructKey(double x, double y);
    string construtValue(int rid, int pucks);
    bool checkrobotExist(double x, double y);
    bool checkpuckExist(double x, double y);
    
};
    
