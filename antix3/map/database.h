 
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
    bool setCommand(string key, string value, int checkvalu);
    string getCommand(string key);
    bool checkKeyExist(string key);
    void updatePos(string oldkey, string newkey);
    
    //game command
    
    
    
    
    //helper function
    string constructKey(double x, double y, int pre, int addstar);
    string construtValue(int rid, int pucks);
    bool checkrobotExist(double x, double y);
    bool checkpuckExist(double x, double y);
    bool robotExist(string key);
    int getPucknum(string key);
    bool setPucknum(string key, int num);
    void deleteRobot(string key);
    string keyCommand(string key);
    int getrobotID(string key);
    
    //client can use
    string getSubMap(double x, double y, int precision);
    bool addRobot(double x, double y, int robotID, int robContainPuck);
    bool moveRobot(double oldX, double oldY, double newX, double newY, int robotID,int robotContainPuck );
    bool movePuck( double oldx, double oldy, double newX, double newY, int robotID);
    
};
    