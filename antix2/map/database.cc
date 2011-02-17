#include "database.h" 
#include <assert.h>
// #include <sstream>
#include <iostream>

using namespace std;

//key:   x:0.123456|y:0.123451
//value: r:123456|p:2
database::database()
{
  db = redisConnect("127.0.0.1", 6379);
  if(db->err)
  {
    printf("Unable to make connection\n");
    exit(1);
  }
}

bool database::setCommand(string key, string value)
{
  
    if(checkKeyExist(key))
      return 0;
    
    redisReply *reply = (redisReply*)redisCommand(db,"SET %s %s", key.c_str(), value.c_str());
    printf("SET: %s\ttype: %d\n", reply->str,reply->type);
    
    freeReplyObject(reply);
    return 1;
}


string database::getCommand(string key)
{
    redisReply * reply = (redisReply*)redisCommand(db,"get %s", key.c_str());
    printf("Get: %s\ttype: %d\n", reply->str,reply->type);
    string tmp = reply ->str;
    freeReplyObject(reply);
    return tmp;
}

bool database::checkKeyExist(string key)
{
  redisReply * reply = (redisReply*)redisCommand(db,"exists %s", key.c_str());
  if(reply->integer==1)
  {
    printf("key already exit\n");
    return 1;
  }
    freeReplyObject(reply);
    return 0;
}

//needs to put back the puck back to the old key if there's any.               bug here=====================
void database::updatePos(string oldkey, string newkey)
{
    redisReply * reply = (redisReply*)redisCommand(db,"rename %s %s", oldkey.c_str(), newkey.c_str());
    printf("rename: %s\tto: %s\n", reply->str,oldkey.c_str());
    freeReplyObject(reply);
    
}

//needs to add precision in here..........................
string database::constructKey(double x, double y)
{
  char tmp [255];
  sprintf(tmp, "x:%f|y:%f",x,y);
  cout<<tmp<<endl;
  return tmp;
}

string database::construtValue(int rid, int pucks)
{
  char tmp [255];
  sprintf(tmp, "r:%d|p:%d",rid,pucks);
  cout<<tmp<<endl;
  return tmp;
}

bool database::moveRobot(double oldX, double oldY, double newX, double newY, int robotID )
{
  string newkey = constructKey(newX, newY);
  if(checkKeyExist(newkey))
    return 0;
  
  string oldkey = constructKey(oldX, oldY);
  
  updatePos(oldkey, newkey);
  return 1;
}


//not complete from here on...............
bool database::addRobot(double x, double y, int robotID)
{
  string key = constructKey(x,y);
  if(checkKeyExist(key))
      return 0;
  
  
  
  return 1;
}

string database::getSubMap(double x, double y)
{
return NULL;
}  

bool database::checkrobotExist(double x, double y)
{
  return NULL;
}
bool database:: checkpuckExist(double x, double y)
{
  return NULL;
}