#include "database.h" 
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>

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

bool database::setCommand(string key, string value, int checkvalu)
{
    if(checkvalu)
      if(checkKeyExist(key))
	if(robotExist(key))
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
   
    if(reply->str == NULL)
    {
      freeReplyObject(reply);  
      return "";
    }
    string tmp = reply ->str;
   
    freeReplyObject(reply);
    return tmp;
}

string database::keyCommand(string key)
{
    redisReply * reply = (redisReply*)redisCommand(db,"keys %s", key.c_str());
    printf("Get: %s\ttype: %d\n", reply->str,reply->type);
   
    if(reply->str == NULL)
    {
      freeReplyObject(reply);  
      return "";
    }
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
string database::constructKey(double x, double y, int pre, int addstar)
{
 if (pre == -1) 
   pre = 7;
  
  std::ostringstream cy;
  cy << y;
  std::ostringstream cx;
  cx << x;

  string stry = cy.str();
  string strx = cx.str();
  int getdotX = strx.find(".");
  int getdotY = stry.find(".");
  cout<<stry<<endl<<strx<<endl;
  cout<< "DDDDDDDDDDDDDDDDDDDDDDDDDD\n\n"<<stry.substr(0, getdotY+pre+1)<<endl<<strx.substr(0,getdotX+pre+1)<<endl;
  
  string outk = "x:" + strx.substr(0,getdotX+pre+1);
  
  if(addstar)
    outk+="*";
  
  outk += "|y:" + stry.substr(0, getdotY+pre+1);
  
  if(addstar)
    outk+="*";
  
  return outk;

}

string database::construtValue(int rid, int pucks)
{
  char tmp [255];
  sprintf(tmp, "r:%d|p:%d",rid,pucks);
  cout<<tmp<<endl;
  return tmp;
}

bool database::moveRobot(double oldX, double oldY, double newX, double newY, int robotID, int robotContainPuck )
{
  string newkey = constructKey(newX, newY,-1,0);
  if(robotExist(newkey))
    return 0;
  string oldkey = constructKey(oldX, oldY,-1,0);
  if(!checkKeyExist(newkey))
    updatePos(oldkey, newkey);
  
  deleteRobot(oldkey);
  setCommand(newkey, construtValue(robotID,robotContainPuck+getPucknum(newkey)), 0);
  
  return 1;
}

//===================
bool database::movePuck( double oldx, double oldy, double newX, double newY, int robotID)
{
    string newkey = constructKey(newX, newY,-1,0);
    string oldkey =constructKey(oldx, oldy,-1,0);
    
    int pucknum = getPucknum(oldkey);
    //if robot exist, then remove puck
    if(robotExist(newkey) || pucknum >1)
      setCommand(oldkey, construtValue(getrobotID(oldkey), pucknum-1), 0);
     //if robot does not exit and only one puck, remove key
    else
      deleteRobot(oldkey);
    
    setCommand(newkey, construtValue(getrobotID(newkey), getPucknum(newkey)+1), 0);
      
      
      //if no robot, more than one puck, take off one puck
      
    return 0;
  
}


void database::deleteRobot(string key)
{
  if(getPucknum(key)<1)
  {
    redisReply * reply = (redisReply*)redisCommand(db,"del %s", key.c_str());
    printf("delete: %s\tto: %s\n", reply->str,key.c_str());
    freeReplyObject(reply);
  }
  else
    setCommand(key, "r:|p:"+ getPucknum(key),0);
	       
    
}



bool database::addRobot(double x, double y, int robotID, int robotContainPuck)
{
  string key = constructKey(x,y,-1,0);
  if(checkKeyExist(key))
     if(robotExist(key))
	return 0;
   
   return setCommand(key, construtValue(robotID,robotContainPuck+getPucknum(key)), 0);
  
}

bool database::robotExist(string key)
{
    return (getCommand(key).find("r:") > 0)? true:false;
}     
      
int database::getPucknum(string key)
{
  
  string tmp = getCommand(key);
  int pNum = tmp.find("p:");
  //return /*atoi((tmp.substr(pNum+2)).c_str())*/;
  return (pNum >=0) ? atoi((tmp.substr(pNum+2)).c_str()) : 0 ;
}

int database::getrobotID(string key)
{
  string tmp = getCommand(key);
  int pNum = tmp.find("|");
  //return /*atoi((tmp.substr(pNum+2)).c_str())*/;
  return (pNum >=0) ? atoi((tmp.substr(2, pNum)).c_str()) : 0 ;
  
}
  
bool database::setPucknum(string key, int num)
{
  string tmp = getCommand(key);
  int pNum = tmp.find("p:");
  if( pNum >=0)
  {
    char buf [10];
    sprintf(buf, "%i",num);
    tmp.replace(pNum+2,10, buf);
    return setCommand( key, tmp,1);
  }
  return false;
}
      
string database::getSubMap(double x, double y, int precision)
{
    string tmp = constructKey(x,y,precision,1);
  
    string val = "";
    
    redisReply * reply = (redisReply*)redisCommand(db,"keys %s", tmp.c_str());
    if (reply->type == REDIS_REPLY_ARRAY) 
        for (unsigned int j = 0; j < reply->elements; j++) 
	{
	    val += (reply->element[j]->str);
	    val += " ";
	    val += getCommand(reply->element[j]->str);
	    val += " ";
        }
    
    freeReplyObject(reply);
 

    return val;
}  




bool database::checkrobotExist(double x, double y)
{
  return robotExist(constructKey(x,y,-1,0));
}
bool database:: checkpuckExist(double x, double y)
{
  return getPucknum(constructKey(x,y,-1,0));
}