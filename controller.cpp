#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "entities.cpp"

using namespace std;

#define PUCK_ACTION_NONE 0
#define PUCK_ACTION_PICKUP 1
#define PUCK_ACTION_DROP 2

class Controller {
public:
	// same for all robots on team on node
	vector<double> doubles_shared;
	vector<int> ints_shared;

	// differ for each robot
	double x,
		y,
		a;
	int id;
	double last_x,
		last_y;
	Home *home;
	bool has_puck;
	vector<CSeePuck> seen_pucks;

	int puck_action;
	double v,
		w;

	bool collided;

	vector<double> doubles;
	vector<int> ints;

	Controller() { }

	virtual void set_static_vars(double world_size, double pickup_range);

	virtual void controller();
};

#ifndef IS_CLIENT
/*
	The following 2 functions are used for dynamic class loading
*/
extern "C" Controller*
create_object() {
	return new Controller;
}

extern "C" void
destroy_object (Controller* object) {
	delete object;
}

/*
	Needed due to how static class variables are declared
*/
void
Controller::set_static_vars(double world_size, double pickup_range) {
	antix::world_size = world_size;
	Robot::pickup_range = pickup_range;
}
#endif

#endif
