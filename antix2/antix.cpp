/*
	Some library functions
*/

#include <iostream>

using namespace std;

class antix {
public:
	static const char *
	make_endpoint(string host, string port) {
		string s = "tcp://" + host + ":" + port;
		return s.c_str();
	}
};
