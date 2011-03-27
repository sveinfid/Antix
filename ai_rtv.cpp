#ifndef AI_RTV_H
#define AI_RTV_H

#include "controller.cpp"

using namespace std;

class ai_rtv : public Controller {
public:

	void
	controller() {
		double heading_error(0.0);
		// distance and angle to home
		const double dx( antix::WrapDistance( home->x - x ) );
		const double dy( antix::WrapDistance( home->y - y ) );
		const double da( antix::fast_atan2( dy, dx ) );
		const double dist( hypot( dx, dy ) );

		if (collided) {
			//cout << "I collided!" << endl;
		}

		// if this robot is holding a puck
		if (has_puck) {
			// turn towards home
			heading_error = antix::AngleNormalize(da - a);

			// if the robot is some random distance inside home, drop puck
			if (dist < drand48() * home->r) {
				puck_action = antixtransfer::control_message::DROP;
			}

		// not holding a puck
		} else {
			// if we're away from home and see puck(s)
			if (dist > home->r && seen_pucks->size() > 0) {
				double closest_range(1e9);
				// Look at all the pucks we can see
				vector<CSeePuck>::iterator end = seen_pucks->end();
				for (vector<CSeePuck>::const_iterator it = seen_pucks->begin(); it != end; it++) {
					// If one is within pickup distance, try to pick it up
					if (it->range < Robot::pickup_range && !has_puck) {
						// remember this location
						last_x = x;
						last_y = y;
						puck_action = antixtransfer::control_message::PICKUP;
					}

					// Otherwise see if its the closest we've seen yet
					if (it->range < closest_range && !it->held) {
						heading_error = it->bearing;
						closest_range = it->range;
					}
				}

			// we don't see any pucks
			} else {
				const double lx( antix::WrapDistance( last_x - x ) );
				const double ly( antix::WrapDistance( last_y - y ) );

				// go towards last place a puck was picked up (or attempted pick up in
				// the case of this version
				heading_error = antix::AngleNormalize( antix::fast_atan2(ly, lx) - a );

				// if the robot is at the location of last attempted puck, choose random
				if ( hypot( lx, ly ) < 0.05 ) {
					last_x += drand48() * 1.0 - 0.5;
					last_y += drand48() * 1.0 - 0.5;
					last_x = antix::DistanceNormalize( last_x );
					last_y = antix::DistanceNormalize( last_y );
				}
			}
		// done not holding puck case
		}

		// check if the robot is pointing in correct direction
		if ( fabs(heading_error) < 0.1 ) {
			v = 0.005;
			w = 0.0;
		} else {
			v = 0.001;
			w = 0.2 * heading_error;
		}
	}
};

#endif
