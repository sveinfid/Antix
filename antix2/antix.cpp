/*
	Some library functions
*/

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

using namespace std;

class antix {
public:
	static const char *
	make_endpoint(string host, string port) {
		string s = "tcp://" + host + ":" + port;
		return s.c_str();
	}
};
