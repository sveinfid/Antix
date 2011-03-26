/*
	Some object definitions

	Most of the objects are essentially copied from rtv's Antix
*/

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

class Puck;

class Home {
public:
	double x, y;
	double r;
	Colour colour;
	int team;
	vector<Puck *> pucks;
	int score;

	Home(double x, double y, double r, int team) : x(x), y(y), r(r), team(team) {
		colour = Colour();
		score = 0;
	}

	Home(double x, double y, double r, Colour colour) : x(x), y(y), r(r), colour(colour) {
		score = 0;
	}

	Home(double r) : r(r) {
		x = antix::rand_between(0, antix::world_size);
		y = antix::rand_between(0, antix::world_size);
		colour = Colour();
		score = 0;
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
	Home *home;
	int lifetime;

	// random pose stuff is from rtv's Antix
	Puck(double min_x, double max_x) {
		x = antix::rand_between(min_x, max_x);
		y = antix::rand_between(0, antix::world_size);
		held = false;
		robot = NULL;
		index = 0;
		lifetime = 0;
		home = NULL;
	}
	Puck(double x, double y, bool held) : x(x), y(y), held(held) {
		robot = NULL;
		index = 0;
		lifetime = 0;
		home = NULL;
	}
};

class SeePuck {
public:
	double range;
	Puck *puck;

	SeePuck(Puck *puck, double range) : puck(puck), range(range) {}
};

/*
	Used to tell Controller what pucks it can see
*/
class CSeePuck {
public:
	bool held;
	double range,
		bearing;

	CSeePuck(const bool held, const double range, const double bearing)
		: held(held), range(range), bearing(bearing) { }
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

	bool collided;

	// Critical section vector we're in, or NULL
	vector<Robot *> *critical_section;
	bool in_critical;

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

	// robot's individual memory
	vector<int> ints;
	vector<double> doubles;

	bbox_t sensor_bbox;

	// Used in Map
	Robot(double x, double y, int id, int team, double last_x, double last_y) : x(x), y(y), id(id), team(team), last_x(last_x), last_y(last_y) {
		a = 0;
		v = 0;
		w = 0;
		puck = NULL;
		has_puck = false;
		index = 0;
		cindex = 0;
		home = NULL;
		collided = false;
		critical_section = NULL;
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
		home = NULL;
		collided = false;
		critical_section = NULL;
	}

	void
	random_warp(double min_x, double max_x) {
		x = antix::rand_between(min_x, max_x);
		y = antix::rand_between(0, antix::world_size);
	}

	static Robot *
	did_geom_collide(Robot *r, unsigned int cindex, double x, double y) {
		if (cmatrix[cindex] == NULL || cmatrix[cindex] == r)
			return NULL;

		Robot *r2 = cmatrix[cindex];

		const double max_range = robot_radius + robot_radius;

		// can perhaps avoid subsequent checks by looking at dx, dy individually,
		// similar to that in testrobots
		const double dx( antix::WrapDistance( r2->x - x ) );
		const double dy( antix::WrapDistance( r2->y - y ) );

		const double range( hypot( dx, dy ) );
		if (range <= max_range)
			return r2;

		// XXX put this back in
		/*
		const double squared_max_range = max_range * max_range;
		const double dsq = dx*dx + dy*dy;
		if ( dsq <= squared_max_range )
			return r2;
		*/

		return NULL;
	}

	/*
		If the target cindex is taken, definitely collided
		Otherwise perform geometric collision checks on those cells around
		the target cindex

		Returns the robot we collided with, or NULL
	*/
	static Robot *
	did_collide(Robot *r, unsigned int cindex, double x, double y) {
		if (cmatrix[cindex] != NULL && cmatrix[cindex] != r)
			return cmatrix[cindex];

		const unsigned int c_x = antix::CCell_x(x);
		const unsigned int c_y = antix::CCell_y(y);

		const unsigned int top_left = (c_x - 1) + (c_y - 1) * antix::cmatrix_width;
		const unsigned int top_centre = (c_x) + (c_y - 1) * antix::cmatrix_width;
		const unsigned int top_right = (c_x + 1) + (c_y - 1) * antix::cmatrix_width;
		const unsigned int bottom_left = (c_x - 1) + (c_y + 1) * antix::cmatrix_width;
		const unsigned int bottom_centre = (c_x) + (c_y + 1) * antix::cmatrix_width;
		const unsigned int bottom_right = (c_x + 1) + (c_y + 1) * antix::cmatrix_width;
		const unsigned int left = (c_x - 1) + (c_y) * antix::cmatrix_width;
		const unsigned int right = (c_x + 1) + (c_y) * antix::cmatrix_width;

		const unsigned int cmatrix_size = antix::cmatrix_width * antix::cmatrix_width;

		Robot *r2;

		if (top_left < cmatrix_size) {
			r2 = did_geom_collide(r, top_left, x, y);
			if (r2 != NULL) return r2;
		}

		if (top_centre < cmatrix_size) {
			r2 = did_geom_collide(r, top_centre, x, y);
			if (r2 != NULL) return r2;
		}

		if (top_right < cmatrix_size) {
			r2 = did_geom_collide(r, top_right, x, y);
			if (r2 != NULL) return r2;
		}

		if (bottom_left < cmatrix_size) {
			r2 = did_geom_collide(r, bottom_left, x, y);
			if (r2 != NULL) return r2;
		}

		if (bottom_centre < cmatrix_size) {
			r2 = did_geom_collide(r, bottom_centre, x, y);
			if (r2 != NULL) return r2;
		}

		if (bottom_right < cmatrix_size) {
			r2 = did_geom_collide(r, bottom_right, x, y);
			if (r2 != NULL) return r2;
		}

		if (left < cmatrix_size) {
			r2 = did_geom_collide(r, left, x, y);
			if (r2 != NULL) return r2;
		}

		if (right < cmatrix_size) {
			r2 = did_geom_collide(r, right, x, y);
			if (r2 != NULL) return r2;
		}

		return NULL;
	}

	/*
		When this robot collides, this is called
	*/
	void
	collide() {
		// 180 our direction?
		a = antix::AngleNormalize(a + M_PI);

		// both speeds to 0
		v = 0;
		w = 0;

		collided = true;
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

		double new_x = antix::DistanceNormalize(x + dx);
		double new_y = antix::DistanceNormalize(y + dy);

		/*
			Collision matrix stuff
		*/
#if COLLISIONS
		unsigned int new_cindex = antix::CCell(new_x, new_y);

		collided = false;

		// we try to move to a new collision cell
		if (new_cindex != cindex) {
			// if it's occupied, we can't move there. Disallow move
			if ( cmatrix[new_cindex] != NULL ) {
				// we collide
				collide();
				// other robot also collides
				cmatrix[new_cindex]->collide();

				return;
			}
		}
		// Now do checks in surrounding cells at the location we want for
		// whether we collide
		Robot *other = did_collide(this, new_cindex, new_x, new_y);
		if (other != NULL) {
			//cout << "Found collided from check_collided()." << endl;
			collide();
			// XXX maybe should find all robots we collided with and collide()
			// them rather than only being able to collide() the first returned
			return;
		}

		// remove from old cindex
		if (cindex != new_cindex) {
			cmatrix[cindex] = NULL;
		}

		cindex = new_cindex;
		cmatrix[cindex] = this;
#endif

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
#if DEBUG_ERASE_PUCK
				cout << "EraseAll puck #1 in update_pose()" << endl;
#endif
				antix::EraseAll( puck, matrix[index].pucks );
				matrix[new_index].pucks.push_back( puck );
				puck->index = new_index;
			}
			index = new_index;
		}

		FovBBox( sensor_bbox );
	}

	// from rtv's Antix
	// find the axis-aligned bounding box of our field of view
	void
	FovBBox( bbox_t &box ) {
		box.x.min = x;
		box.x.max = x;
		box.y.min = y;
		box.y.max = y;
		const double halffov = fov/2.0;
		const double lefta = a + halffov;
		const double righta = a - halffov;

		// extreme left of fov
		antix::grow_bounds( box.x, x + vision_range * antix::fast_cos( lefta ) );
		antix::grow_bounds( box.y, y + vision_range * antix::fast_sin( lefta ) );

		// extreme right of fov
		antix::grow_bounds( box.x, x + vision_range * antix::fast_cos( righta ) );
		antix::grow_bounds( box.y, y + vision_range * antix::fast_sin( righta ) );

		// points where the fov crosses an axis
		if (lefta > 0 && righta < 0)
			antix::grow_bounds( box.x, x + vision_range );
		if (lefta > M_PI/2.0 && righta < M_PI/2.0)
			antix::grow_bounds( box.y, y + vision_range );
		if (lefta > M_PI && righta < M_PI)
			antix::grow_bounds( box.x, x - vision_range );
		if (lefta > -M_PI && righta < -M_PI)
			antix::grow_bounds( box.x, x - vision_range );
		if (lefta > -M_PI/2.0 && righta < -M_PI/2.0 )
			antix::grow_bounds( box.y, y - vision_range );
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
	pickup(vector<Puck *> *pucks) {
#if DEBUG
		cout << "Trying to pickup puck on robot " << id << " team " << team << endl;
#endif

		// check we aren't already holding a puck
		if (has_puck)
			return;
		assert(puck == NULL);
		
		// see if we can find an available puck to pick up
		vector<SeePuck>::const_iterator see_pucks_end = see_pucks.end();
		for (vector<SeePuck>::const_iterator it = see_pucks.begin(); it != see_pucks_end; it++) {
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
#if DEBUG_ERASE_PUCK
					cout << "EraseAll puck #1 in pickup()" << endl;
#endif
					antix::EraseAll( puck, matrix[puck->index].pucks );
					matrix[index].pucks.push_back( puck );
					puck->index = index;
				}

				// if puck is in a home, disassociate it from that home
				if (puck->home != NULL) {
#if DEBUG_ERASE_PUCK
					cout << "EraseAll puck #2 in pickup()" << endl;
#endif
					antix::EraseAll( puck, puck->home->pucks );
					puck->home = NULL;
				}
#if DEBUG
				cout << "Robot " << id << " on team " << team << " picked up a puck." << endl;
#endif
				break;
			}
		}
	}

	/*
		Return pointer to the Home puck p is in, if it's in a home
		Otherwise NULL
	*/
	static Home *
	is_puck_in_home(Puck *p, vector<Home *> *homes) {
		vector<Home *>::iterator homes_end = homes->end();
		for (vector<Home *>::const_iterator it = homes->begin(); it != homes_end; it++) {
			Home *h = *it;
			const double dx( antix::WrapDistance( h->x - p->x ) );
			const double dy( antix::WrapDistance( h->y - p->y ) );
			const double range( hypot( dx, dy ) );

			if (range < antix::home_radius) {
#ifndef NDEBUG
				// make sure puck isn't already in the vector
				//for (vector<Puck *>::const_iterator it2 = h->pucks.begin(); it2 != h->pucks.end(); it2++) {
				//	assert(*it2 != p);
				//}
#endif
				return h;
			}
		}
		return NULL;
	}

	/*
		Attempt to drop a puck for the given robot
	*/
	void
	drop(vector<Puck *> *pucks, vector<Home *> *homes) {
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
		// store reference so as to check puck location further down
		Puck *p = puck;
		puck = NULL;

		assert(p->home == NULL);

		// if it's in a home, add it to that home's vector of pucks
		Home *h = is_puck_in_home(p, homes);
		if (h != NULL) {
			p->home = h;
			h->pucks.push_back( p );
			// set lifetime to initial
			p->lifetime = PUCK_LIFETIME;
		}
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
