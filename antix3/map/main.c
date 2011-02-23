#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>

#include "database.h"



#include "./hiclient/hiredis/hiredis.h"
using namespace std;


void RunServer();
void RunClient();


database *db;



int main(void) {

  //fork() == 0? RunClient():  RunServer();
  RunClient();
  //(*db).moveRobot(1.0001,2.0002,3.0003,4.0004,5);
  /*
   (*db).setPucknum("key", 100);
   cout<<(*db).getCommand("key");
   (*db).addRobot(1,2,123456,0);
   cout<<(*db).getCommand((*db).constructKey(1,2))<<endl;
    return 0;*/
  
  for(int i = 5; i>=0; i--)
  {
    
    printf("==================\nchecking for %d digits\n", i);
    cout<<(*db).getSubMap(0.1234567,0.123456, i)<<endl; 
  }
  return 0;
}



void RunServer()
{
  printf("I am server....\n");
 int ab = execlp("./redis-server", NULL, (char*)0);
 printf("printing ab:%d\n",ab);
//execv("./antix2", NULL);
}



void RunClient()
{
  sleep (3);
  printf("I am client\n");
  db = new database();
  printf("client is running add key\n");
//   (*db).setCommand("key", "value", 0);
  //printf("you should see error\n");
(*db).setCommand("x:0.1234567|y:0.123456", "r:1|p:2",1);
(*db).setCommand("x:0.123|y:0.12", "r:2|p:2",1);
(*db).setCommand("x:0.124|y:0.12", "r:3|p:2",1);
(*db).setCommand("x:0.123|y:0.123456", "r:4|p:2",1);
(*db).setCommand("x:0.1237|y:0.156", "r:5|p:2",1);
}