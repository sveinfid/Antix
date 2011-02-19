
#ifndef ENTITIES_H
#define ENTITIES_H

#include "antix.cpp"

/*
	from rtv's Antix
*/
class Colour {
	public:
	double r, g, b;

	Colour() {
		r = drand48();
		g = drand48();
		b = drand48();
	}
	Colour (double r, double g, double b) : r(r), g(g), b(b) { }
};

class Home {
public:
	double x, y;
	double r;
	Colour colour;
	int team;

	Home(double x, double y, double r, int team) : x(x), y(y), r(r), team(team) {
		colour = Colour();
	}

	Home(double x, double y, double r, Colour colour) : x(x), y(y), r(r), colour(colour) { }

	Home(double r) : r(r) {
		x = antix::rand_between(0, antix::world_size);
		y = antix::rand_between(0, antix::world_size);
		colour = Colour();
	}
};

class Robot;

class Puck {
public:
	double x,
		y;
	bool held;
	Robot *robot;

	// random pose stuff is from rtv's Antix
	Puck(double min_x, double max_x) {
		x = antix::rand_between(min_x, max_x);
		y = antix::rand_between(0, antix::world_size);
		held = false;
		robot = NULL;
	}
	Puck(double x, double y, bool held) : x(x), y(y), held(held) {
		robot = NULL;
	}
};

class SeePuck {
public:
	double range;
	Puck *puck;

	SeePuck(Puck *puck, double range) : puck(puck), range(range) {}
};

class Robot {
public:
	static double pickup_range;
	static double fov;
	static double vision_range;

	double x, y;
	// orientation
	double a;
	// forward speed
	double v;
	// turn speed
	double w;

	// together uniquely identifies the robot
	// team = client id, essentially
	int team;
	int id;

	bool has_puck;
	Puck *puck;
	Home *home;

	// store what pucks we can see
	vector<SeePuck> see_pucks;

	Robot(double x, double y, int id, int team) : x(x), y(y), id(id), team(team) {
		a = 0;
		v = 0;
		w = 0;
		puck = NULL;
		has_puck = false;
	}

	Robot(double x, double y, int team, double a) : x(x), y(y), team(team), a(a) {
		id = -1;
		v = 0;
		w = 0;
		puck = NULL;
		has_puck = false;
	}

	/*
		update the pose of a single robot
		Taken from rtv's Antix
	*/
	void
	update_pose() {
#if DEBUG
		cout << "Updating pose of robot " << id << " team " << team << endl;
#endif
		double dx = v * cos(a);
		double dy = v * sin(a);
		double da = w;

		x = antix::DistanceNormalize(x + dx);
		y = antix::DistanceNormalize(y + dy);
		a = antix::AngleNormalize(a + da);

		// If we're holding a puck, it must move also
		if (has_puck) {
			assert(puck != NULL);
			assert(puck->robot == this);
			assert(puck->held == true);
			puck->x = x;
			puck->y = y;
		}
	}

	/*
		Remove the puck that robot is carrying, if it is carrying one
	*/
	void
	remove_puck(vector<Puck *> *pucks) {
#if DEBUG
		cout << "Trying to remove puck of robot " << id << " team " << team << endl;
#endif
		if (!has_puck)
			return;

		assert(puck != NULL);
		assert(puck->robot == this);
		assert(puck->held == true);

		bool deleted = false;
		// remove puck from vector
		for (vector<Puck*>::iterator it = pucks->begin(); it != pucks->end(); it++) {
			if ((*it) == puck) {
				pucks->erase(it);
				deleted = true;
				break;
			}
		}
		assert(deleted == true);
		
		// remove record on robot to deleted puck
		delete puck;
		puck = NULL;
		has_puck = false;
	}

	/*
		Update the speed entry for the robot
	*/
	void
	setspeed(double new_v, double new_w) {
#if DEBUG
		cout << "Trying to set speed of robot " << id << " team " << team << endl;
#endif
		v = new_v;
		w = new_w;
	}

	/*
		Attempt to pick up a puck near the robot
	*/
	void
	pickup(vector<Puck *> *pucks) {
#if DEBUG
		cout << "Trying to pickup puck on robot " << id << " team " << team << endl;
#endif
		// check we aren't already holding a puck
		if (has_puck)
			return;
		
		// see if we can find an available puck to pick up
		for (vector<SeePuck>::iterator it = see_pucks.begin(); it != see_pucks.end(); it++) {
			// XXX ugly. We need to check if the puck we see hasn't already moved out
			// of the node
			bool found = false;
			for (vector<Puck *>::iterator it2 = pucks->begin(); it2 != pucks->end(); it2++) {
				if ( (*it2) == it->puck ) {
					found = true;
					break;
				}
			}
			assert(found == true);

			// If the puck isn't held and it's within range
			if ( !it->puck->held && it->range < pickup_range ) {
				has_puck = true;
				puck = it->puck;
				puck->held = true;
				puck->robot = this;
#if DEBUG
				cout << "Robot " << id << " on team " << team << " picked up a puck." << endl;
#endif
				break;
			}
		}
	}

	/*
		Attempt to drop a puck for the given robot
	*/
	void
	drop(vector<Puck *> *pucks) {
#if DEBUG
		cout << "Trying to drop puck on robot " << id << " team " << team << endl;
#endif

		// if we're not holding a puck, nothing to do
		if (!has_puck)
			return;

		// XXX make sure puck is still in puck vector
		// probably not needed
		bool found = false;
		for (vector<Puck *>::iterator it = pucks->begin(); it != pucks->end(); it++) {
			if ( (*it) == puck ) {
				found = true;
				break;
			}
		}
		assert(found == true);

		assert(puck != NULL);
		assert(puck->robot == this);
		assert(puck->held == true);
		
		// otherwise free it
		has_puck = false;
		puck->held = false;
		puck->robot = NULL;
		puck = NULL;
#if DEBUG
		cout << "Done dropping puck" << endl;
#endif
	}
};

double Robot::pickup_range;
double Robot::fov;
double Robot::vision_range;

// Robot class for clients
class CRobot {
public:
	double last_x;
	double last_y;

	CRobot(double last_x, double last_y) : last_x(last_x), last_y(last_y) {}
	CRobot() {
		// XXX potentially bad
		last_x = 0.0;
		last_y = 0.0;
	}
};
#endif
