
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
	unsigned int index;
	bool held;
	Robot *robot;

	// random pose stuff is from rtv's Antix
	Puck(double min_x, double max_x) {
		x = antix::rand_between(min_x, max_x);
		y = antix::rand_between(0, antix::world_size);
		held = false;
		robot = NULL;
		index = 0;
	}
	Puck(double x, double y, bool held) : x(x), y(y), held(held) {
		robot = NULL;
		index = 0;
	}
};

class SeePuck {
public:
	double range;
	Puck *puck;

	SeePuck(Puck *puck, double range) : puck(puck), range(range) {}
};

/*
	from rtv's Antix
*/
class MatrixCell {
public:
	vector<Robot *> robots;
	vector<Puck *> pucks;
};

class Robot {
public:
	static double pickup_range;
	static double fov;
	static double vision_range;
	static double vision_range_squared;
	static double robot_radius;
	static vector<MatrixCell> matrix;
	static vector<Robot *> cmatrix;

	// index into sensor matrix
	unsigned int index;
	// index into collision matrix
	unsigned int cindex;

	double x, y;
	// orientation
	double a;
	// forward speed
	double v;
	// turn speed
	double w;

	// last point we were heading to. see .proto for why
	double last_x;
	double last_y;

	// together uniquely identifies the robot
	// team = client id, essentially
	int team;
	int id;

	bool has_puck;
	Puck *puck;
	Home *home;

	// store what pucks we can see
	vector<SeePuck> see_pucks;

	// Used in Map
	Robot(double x, double y, int id, int team, double last_x, double last_y) : x(x), y(y), id(id), team(team), last_x(last_x), last_y(last_y) {
		a = 0;
		v = 0;
		w = 0;
		puck = NULL;
		has_puck = false;
		index = 0;
		cindex = 0;
	}

	// Used in GUI & foreign robots
	Robot(double x, double y, int team, double a) : x(x), y(y), team(team), a(a) {
		id = -1;
		v = 0;
		w = 0;
		puck = NULL;
		has_puck = false;
		index = 0;
		cindex = 0;
	}

	void
	random_warp(double min_x, double max_x) {
		x = antix::rand_between(min_x, max_x);
		y = antix::rand_between(0, antix::world_size);
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
		const double dx = v * antix::fast_cos(a);
		const double dy = v * antix::fast_sin(a);
		const double da = w;

		// always update angle even if we don't move
		a = antix::AngleNormalize(a + da);

		const double new_x = antix::DistanceNormalize(x + dx);
		const double new_y = antix::DistanceNormalize(y + dy);

		/*
			Collision matrix stuff
		*/
		unsigned int new_cindex = antix::CCell(new_x, new_y);
		// we try to move to a new collision cell
		if (new_cindex != cindex) {
			// if it's occupied, we can't move there. Disallow move
			if ( cmatrix[new_cindex] != NULL ) {
				// XXX does this make sense?
				a = -a;
				return;
			}
			// otherwise no robot in that cell. move to it and continue
			cmatrix[cindex] = NULL;
			cindex = new_cindex;
			cmatrix[cindex] = this;
		}

		x = new_x;
		y = new_y;

		/*
			Sensor matrix stuff	
		*/
		const unsigned int new_index = antix::Cell( x, y );

		// If we're holding a puck, it must move also
		if (has_puck) {
			assert(puck != NULL);
			assert(puck->robot == this);
			assert(puck->held == true);
			puck->x = x;
			puck->y = y;
		}

		if (new_index != index ) {
			antix::EraseAll( this, matrix[index].robots );
			matrix[new_index].robots.push_back( this );

			if (has_puck) {
				antix::EraseAll( puck, matrix[index].pucks );
				matrix[new_index].pucks.push_back( puck );
				puck->index = new_index;
			}
			index = new_index;
		}
	}

	/*
		Update the speed entry for the robot
	*/
	void
	setspeed(double new_v, double new_w, double new_last_x, double new_last_y) {
#if DEBUG
		cout << "Trying to set speed of robot " << id << " team " << team << endl;
#endif
		v = new_v;
		w = new_w;
		last_x = new_last_x;
		last_y = new_last_y;
	}

	/*
		Attempt to pick up a puck near the robot
	*/
	void
	pickup(vector<Puck *> *pucks, double new_last_x, double new_last_y) {
#if DEBUG
		cout << "Trying to pickup puck on robot " << id << " team " << team << endl;
#endif
		last_x = new_last_x;
		last_y = new_last_y;

		// check we aren't already holding a puck
		if (has_puck)
			return;
		assert(puck == NULL);
		
		// see if we can find an available puck to pick up
		for (vector<SeePuck>::iterator it = see_pucks.begin(); it != see_pucks.end(); it++) {
			// Check if the puck we see hasn't already moved out of the node
#ifndef NDEBUG
			bool found = false;
			for (vector<Puck *>::iterator it2 = pucks->begin(); it2 != pucks->end(); it2++) {
				if ( (*it2) == it->puck ) {
					found = true;
					break;
				}
			}
			assert(found == true);
#endif

			// If the puck isn't held and it's within range
			if ( !it->puck->held && it->range < pickup_range ) {
				assert(it->puck->robot == NULL);
				assert(it->puck->held == false);
				has_puck = true;
				puck = it->puck;
				puck->held = true;
				puck->robot = this;

				// ensure puck is in our same cell
				if (puck->index != index) {
					antix::EraseAll( puck, matrix[puck->index].pucks );
					matrix[index].pucks.push_back( puck );
					puck->index = index;
				}
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

		// make sure puck is still in the puck vector
#ifndef NDEBUG
		bool found = false;
		for (vector<Puck *>::iterator it = pucks->begin(); it != pucks->end(); it++) {
			if ( (*it) == puck ) {
				found = true;
				break;
			}
		}
		assert(found == true);
#endif

		assert(puck != NULL);
		assert(puck->robot == this);
		assert(puck->held == true);
		
		// free it
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
double Robot::vision_range_squared;
double Robot::robot_radius;
vector<MatrixCell> Robot::matrix;
vector<Robot *> Robot::cmatrix;

#endif
