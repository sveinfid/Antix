#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>

#include "database.h"


#include "./hiclient/hiredis/hiredis.h"

void RunServer();
void RunClient();

database *db;


int main(void) {

  //fork() == 0? RunClient():  RunServer();
  RunClient();
  (*db).moveRobot(1.0001,2.0002,3.0003,4.0004,5);

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
  (*db).setCommand("key", "value");
  printf("you should see error\n");
(*db).setCommand("key", "value");

}