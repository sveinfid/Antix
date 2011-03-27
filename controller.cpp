#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "entities.cpp"

using namespace std;

class Controller {
public:
	static vector<double> doubles_shared;
	static vector<int> ints_shared;

	double x,
		y,
		a;
	int id;
	double last_x,
		last_y;
	Home *home;
	bool has_puck;
	vector<CSeePuck> *seen_pucks;

	antixtransfer::control_message::Puck_Action puck_action;
	double v,
		w;

	bool collided;

	vector<double> doubles;
	vector<int> ints;

	Controller() { }

	virtual void
	controller() {
		assert( 1 == 0 );
	}
};

vector<double> Controller::doubles_shared;
vector<int> Controller::ints_shared;

#endif
