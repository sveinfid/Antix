/*
	Hold robots, pucks, and their location in the world
	Calculate movements
	Find those on boundary & build boundary protobuf messages
	Find those outside range & build movement protobuf messages
*/

#ifndef MAP_H
#define MAP_H

#include "entities.cpp"

using namespace std;

class Map {
public:
	// where we begin
	double my_min_x;
	// where neighbour begins
	double my_max_x;

	// the robots & pucks we control
	vector<Puck *> pucks;
	vector<Robot *> robots;
	// sent to us by neighbours
	vector<Puck> foreign_pucks;
	vector<Robot> foreign_robots;

	// We need to know homes to set robot's first last_x, last_y
	vector<Home *> homes;

	// what each robot can see by team
	map<int, antixtransfer::sense_data *> sense_map;

	Map(double my_min_x,
		antixtransfer::Node_list *node_list,
		int initial_puck_amount,
		int my_id) : my_min_x(my_min_x) {

		my_max_x = my_min_x + antix::offset_size;
		cout << "Set dimensions of this map. Min x: " << my_min_x << " Max x: " << my_max_x << endl;
		populate_homes(node_list);
		create_robots(node_list, my_id);
		generate_pucks(initial_puck_amount);
	}

	/*
		There is a list of homes & their locations in node list.
		Use it to populate our home list
	*/
	void
	populate_homes(antixtransfer::Node_list *node_list) {
		for (int i = 0; i < node_list->home_size(); i++) {
			Home *h = new Home(node_list->home(i).x(),
				node_list->home(i).y(),
				// XXX radius doesn't really matter right now (here), but may at some point
				0,
				node_list->home(i).team()
			);
			homes.push_back(h);
		}
	}

	Home *
	find_robot_home(int team) {
		for (vector<Home *>::iterator it = homes.begin(); it != homes.end(); it++) {
			if ( (*it)->team == team )
				return *it;
		}
		return NULL;
	}

	/*
		Node_list contains a list of Robots to be created on each node
		Look for our ID in this list & create the robots assigned to us
	*/
	void
	create_robots(antixtransfer::Node_list *node_list, int my_id) {
		antixtransfer::Node_list::Robots_on_Node *rn;
		for (int i = 0; i < node_list->robots_on_node_size(); i++) {
			rn = node_list->mutable_robots_on_node(i);
			// Create only those robots assigned to this node
			if (rn->node() == my_id) {
				for (int j = 0; j < rn->num_robots(); j++) {
					// We need the robot's home for its initial last_x/last_y
					Home *h = find_robot_home( rn->team() );
					assert(h != NULL);

					Robot *r = new Robot(antix::rand_between(my_min_x, my_max_x), antix::rand_between(0, antix::world_size), j, rn->team(), h->x, h->y);
					robots.push_back(r);
	#if DEBUG
					cout << "Created a bot: Team: " << r->team << " id: " << r->id << " at (" << r->x << ", " << r->y << ")" << endl;
	#endif
				}
			}
		}
	}

	/*
		Place pucks randomly within our region
	*/
	void
	generate_pucks(int initial_puck_amount) {
		for (int i = 0; i < initial_puck_amount; i++) {
			// XXX is this -0.01 needed?
			pucks.push_back( new Puck(my_min_x, my_max_x - 0.01) );
		}
		cout << "Created " << pucks.size() << " pucks." << endl;
	#if DEBUG
		for (vector<Puck *>::iterator it = pucks.begin(); it != pucks.end(); it++) {
			cout << "Puck at " << (*it)->x << "," << (*it)->y << endl;
		}
	#endif
	}

	/*
		Print out what foreign robots & pucks we know of
	*/
	void
	print_foreign_entities() {
		cout << "Current foreign entities: " << endl;
		for (vector<Robot>::iterator it = foreign_robots.begin(); it != foreign_robots.end(); it++)
			cout << "\tRobot at " << it->x << ", " << it->y << endl;
		for (vector<Puck>::iterator it = foreign_pucks.begin(); it != foreign_pucks.end(); it++)
			cout << "\tPuck at " << it->x << ", " << it->y << endl;
	}

	/*
		Output the information on our robots
	*/
	void
	print_local_robots() {
		cout << "Current local robots:" << endl;
		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			cout << "Robot " << (*it)->id << " on team " << (*it)->team << " at (" << (*it)->x << ", " << (*it)->y << ") a: " << (*it)->a << " v: " << (*it)->v << endl;
		}
	}

	/*
		XXX unacceptable cost to find each robot this way!
	*/
	Robot *
	find_robot(int team, int id) {
#if DEBUG
		cout << "Trying to find robot with team " << team << " and id " << id << endl;
#endif
		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			if ((*it)->team == team && (*it)->id == id)
				return *it;
		}
		return NULL;
	}

	void
	add_foreign_robot(double x,
		double y,
		int id,
		int team) {

		foreign_robots.push_back( Robot(x, y, id, team) );
	}

	void
	add_foreign_puck(double x,
		double y,
		bool held) {

		foreign_pucks.push_back( Puck(x, y, held) );
	}

	void
	add_robot(double x,
		double y,
		int id,
		int team,
		double a,
		double v,
		double w,
		bool has_puck,
		double last_x,
		double last_y) {

#if DEBUG
		cout << "Moving: added new robot with a " << a << " w " << w << "(Turn " << antix::turn << ")" << endl;
#endif
		Robot *r = new Robot(x, y, id, team, last_x, last_y);
		r->a = a;
		r->v = v;
		r->w = w;
		r->has_puck = has_puck;
		robots.push_back(r);
		// If the robot is carrying a puck, we have to add a puck to our records
		if (r->has_puck) {
			Puck *p = new Puck(r->x, r->y, true);
			p->robot = r;
			pucks.push_back(p);

			r->puck = p;

			assert(r->has_puck == true);
			assert(r->puck->robot == r);
			assert(p->robot == r);
			assert(r->puck == p);
		}
	}

	/*
		Robot has been found to be outside of our map portion
		Add the relevant data to a new Robot entry in the given move_bot message
	*/
	void
	add_move_robot(Robot *r, antixtransfer::move_bot *move_bot_msg) {
		antixtransfer::move_bot::Robot *r_move = move_bot_msg->add_robot();
		r_move->set_id(r->id);
		r_move->set_team(r->team);
		r_move->set_x(r->x);
		r_move->set_y(r->y);
		r_move->set_a(r->a);
		r_move->set_v(r->v);
		r_move->set_w(r->w);
		r_move->set_has_puck(r->has_puck);
		r_move->set_last_x(r->last_x);
		r_move->set_last_y(r->last_y);
#if DEBUG
		cout << "Moving robot with a " << r->a << " w " << r->w << " (Turn " << antix::turn << ")" << endl;
#endif
	}

	/*
		Look through our list of robots to see if any are outside of our range
		If found, add the relevant data to a move_bot message
		and remove relevant data from our records
	*/
	void
	build_move_message(antixtransfer::move_bot *move_left_msg, antixtransfer::move_bot *move_right_msg) {

		move_left_msg->clear_robot();
		move_right_msg->clear_robot();

		vector<Robot *>::iterator it = robots.begin();
		// while loop as iterator may be updated other due to deletion
		while (it != robots.end()) {
			// We do these 4 cases as it's possible the robot has looped around the world
			// where the robot will be less than our x but actually travels to the right
			// node (one case)

			// If robot's x is less than ours and bigger than our left neighbours, send
			// to our left neighbour
			if ((*it)->x < my_min_x && (*it)->x > my_min_x - antix::offset_size) {
				add_move_robot(*it, move_left_msg);
				(*it)->remove_puck(&pucks);
				delete *it;
				it = robots.erase(it);
	#if DEBUG
				cout << "Moving robot " << (*it)->id << " on team " << (*it)->team << " to left node (1)" << endl;
	#endif

			// Otherwise if it's less than ours and smaller than our left neighbour's,
			// assume that we are the far right node: send it to our right neighbour
			} else if ((*it)->x < my_min_x) {
				add_move_robot(*it, move_right_msg);
				(*it)->remove_puck(&pucks);
				delete *it;
				it = robots.erase(it);
	#if DEBUG
				cout << "Moving robot " << (*it)->id << " on team " << (*it)->team << " to right node (2)" << endl;
	#endif

			// If robot's x is bigger than ours and smaller than our right neighbour's, we
			// send it to our right neighbour
			} else if ((*it)->x >= my_max_x && (*it)->x < my_max_x + antix::offset_size) {
				add_move_robot(*it, move_right_msg);
				(*it)->remove_puck(&pucks);
				delete *it;
				it = robots.erase(it);
	#if DEBUG
				cout << "Moving robot " << (*it)->id << " on team " << (*it)->team << " to right node (3)" << endl;
	#endif

			// Otherwise it's bigger than ours and bigger than our right neighbour's,
			// assume we are the far left node: send it to our left neighbour
			} else if ((*it)->x >= my_max_x) {
				add_move_robot(*it, move_left_msg);
				(*it)->remove_puck(&pucks);
				delete *it;
				it = robots.erase(it);
	#if DEBUG
				cout << "Moving robot " << (*it)->id << " on team " << (*it)->team << " to left node (4)" << endl;
	#endif

			} else {
				it++;
			}
		}
	}

	/*
		Add the given puck to the given map
	*/
	void
	add_border_puck(antixtransfer::SendMap *map, Puck *p) {
		antixtransfer::SendMap::Puck *p_pb = map->add_puck();
		p_pb->set_x( p->x );
		p_pb->set_y( p->y );
		p_pb->set_held( p->held );
	}

	/*
		Add given robot to the given map
	*/
	void
	add_border_robot(antixtransfer::SendMap *map, Robot *r) {
		antixtransfer::SendMap::Robot *r_pb = map->add_robot();
		r_pb->set_x( r->x );
		r_pb->set_y( r->y );
		r_pb->set_team( r->team );
		r_pb->set_id( r->id );
	}

	/*
		Clear the last iteration's border entities, and place any of our robots &
		pucks where they belong
	*/
	void
	rebuild_border_entities(antixtransfer::SendMap *border_map_left, antixtransfer::SendMap *border_map_right) {

		border_map_left->clear_robot();
		border_map_left->clear_puck();
		border_map_right->clear_robot();
		border_map_right->clear_puck();

		foreign_pucks.clear();
		foreign_robots.clear();

		for (vector<Puck *>::iterator it = pucks.begin(); it != pucks.end(); it++) {
			if ((*it)->x > my_max_x - Robot::vision_range)
				add_border_puck(border_map_right, *it);
			else if ((*it)->x < my_min_x + Robot::vision_range)
				add_border_puck(border_map_left, *it);
		}

		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			if ((*it)->x > my_max_x - Robot::vision_range)
				add_border_robot(border_map_right, *it);
			else if ((*it)->x < my_min_x + Robot::vision_range)
				add_border_robot(border_map_left, *it);
		}

		/*
		// for debugging
		cout << "Left border entities: " << endl;
		for (int i = 0; i < border_map_left->robot_size(); i++) {
			cout << "Robot at " << border_map_left->robot(i).x() << ", " << border_map_left->robot(i).y() << endl;
		}
		for (int i = 0; i < border_map_left->puck_size(); i++) {
			cout << "Puck at " << border_map_left->puck(i).x() << ", " << border_map_left->puck(i).y() << endl;
		}

		cout << "Right border entities: " << endl;
		for (int i = 0; i < border_map_right->robot_size(); i++) {
			cout << "Robot at " << border_map_right->robot(i).x() << ", " << border_map_right->robot(i).y() << endl;
		}
		for (int i = 0; i < border_map_right->puck_size(); i++) {
			cout << "Puck at " << border_map_right->puck(i).x() << ", " << border_map_right->puck(i).y() << endl;
		}
		*/
	}

	/*
		Build a map with one entry per team, each entry being a protobuf message stating
		what the robots in that team see

		The actual sense logic is from rtv's Antix

		XXX Really ugly repeating code
	*/
	void
	build_sense_messages() {
		// clear old sense data
		for (map<int, antixtransfer::sense_data *>::iterator it = sense_map.begin(); it != sense_map.end(); it++) {
			delete it->second;
		}
		sense_map.clear();

		// for every robot we have, build a message for it containing what it sees
		for (vector<Robot *>::iterator r = robots.begin(); r != robots.end(); r++) {
			antixtransfer::sense_data *team_msg;

			// if we already have an in progress sense msg for this team, use that
			if (sense_map.count( (*r)->team ) > 0) {
				team_msg = sense_map[(*r)->team];
			// otherwise make a new one and use it
			} else {
				team_msg = new antixtransfer::sense_data;
				sense_map.insert( pair<int, antixtransfer::sense_data *>((*r)->team, team_msg) );
			}

			// create entry for this robot since it's first time we're looking at it
			antixtransfer::sense_data::Robot *robot_pb = team_msg->add_robot();
			robot_pb->set_a( (*r)->a );
			robot_pb->set_x( (*r)->x );
			robot_pb->set_y( (*r)->y );
			robot_pb->set_has_puck( (*r)->has_puck );
			robot_pb->set_id( (*r)->id );
			robot_pb->set_last_x( (*r)->last_x );
			robot_pb->set_last_y( (*r)->last_y );

			// look at all other robots to see if we can see them
			for (vector<Robot *>::iterator other = robots.begin(); other != robots.end(); other++) {
				// we don't look at ourself
				if (*r == *other)
					continue;
				
				double dx( antix::WrapDistance( (*other)->x - (*r)->x ) );
				if ( fabs(dx) > Robot::vision_range )
					continue;

				double dy( antix::WrapDistance( (*other)->y - (*r)->y ) );
				if ( fabs(dy) > Robot::vision_range )
					continue;

				double range = hypot( dx, dy );
				if (range > Robot::vision_range )
					continue;

				// check that it's in fov
				double absolute_heading = atan2( dy, dx );
				double relative_heading = antix::AngleNormalize(absolute_heading - (*r)->a);
				if ( fabs(relative_heading) > Robot::fov/2.0 )
					continue;

				// we can see the robot
				antixtransfer::sense_data::Robot::Seen_Robot *seen_robot = robot_pb->add_seen_robot();
				seen_robot->set_range( range );
				seen_robot->set_bearing( relative_heading );
			}

			// we will now find what pucks we can see, but before that, clear our see_pucks
			(*r)->see_pucks.clear();

			// now look at all the pucks
			for (vector<Puck *>::iterator puck = pucks.begin(); puck != pucks.end(); puck++) {
				double dx( antix::WrapDistance( (*puck)->x - (*r)->x ) );
				if ( fabs(dx) > Robot::vision_range )
					continue;

				double dy( antix::WrapDistance( (*puck)->y - (*r)->y ) );
				if ( fabs(dy) > Robot::vision_range )
					continue;

				double range = hypot( dx, dy );
				if (range > Robot::vision_range)
					continue;

				// fov check
				double absolute_heading = atan2( dy, dx );
				double relative_heading = antix::AngleNormalize( absolute_heading - (*r)->a );
				if ( fabs(relative_heading) > Robot::fov/2.0 )
					continue;

				// we can see the puck
				antixtransfer::sense_data::Robot::Seen_Puck *seen_puck = robot_pb->add_seen_puck();
				seen_puck->set_range( range );
				seen_puck->set_bearing ( relative_heading );
				seen_puck->set_held( (*puck)->held );

				(*r)->see_pucks.push_back(SeePuck(*puck, range));
			}

			// now look at foreign robots
			for (vector<Robot>::iterator other = foreign_robots.begin(); other != foreign_robots.end(); other++) {
				// we don't look at ourself
				if (*r == &*other)
					continue;
				
				double dx( antix::WrapDistance( other->x - (*r)->x ) );
				if ( fabs(dx) > Robot::vision_range )
					continue;

				double dy( antix::WrapDistance( other->y - (*r)->y ) );
				if ( fabs(dy) > Robot::vision_range )
					continue;

				double range = hypot( dx, dy );
				if (range > Robot::vision_range )
					continue;

				// check that it's in fov
				double absolute_heading = atan2( dy, dx );
				double relative_heading = antix::AngleNormalize(absolute_heading - (*r)->a);
				if ( fabs(relative_heading) > Robot::fov/2.0 )
					continue;

				// we can see the robot
				antixtransfer::sense_data::Robot::Seen_Robot *seen_robot = robot_pb->add_seen_robot();
				seen_robot->set_range( range );
				seen_robot->set_bearing( relative_heading );
			}

			// and foreign pucks
			/* XXX we don't deal with picking up foreign pucks right now
			for (vector<Puck>::iterator puck = foreign_pucks.begin(); puck != foreign_pucks.end(); puck++) {
				double dx( antix::WrapDistance( puck->x - (*r)->x ) );
				if ( fabs(dx) > Robot::vision_range )
					continue;

				double dy( antix::WrapDistance( puck->y - (*r)->y ) );
				if ( fabs(dy) > Robot::vision_range )
					continue;

				double range = hypot( dx, dy );
				if (range > Robot::vision_range)
					continue;

				// fov check
				double absolute_heading = atan2( dy, dx );
				double relative_heading = antix::AngleNormalize( absolute_heading - (*r)->a );
				if ( fabs(relative_heading) > Robot::fov/2.0 )
					continue;

				// we can see the puck
				antixtransfer::sense_data::Robot::Seen_Puck *seen_puck = robot_pb->add_seen_puck();
				seen_puck->set_range( range );
				seen_puck->set_bearing ( relative_heading );
				seen_puck->set_held( puck->held );
			}
			*/
		}
	#if DEBUG
		cout << "Sensors re-calculated." << endl;
	#endif
	}

	/*
		Go through our local robots & update their poses
	*/
	void
	update_poses() {
	#if DEBUG
		cout << "Updating poses for all robots..." << endl;
	#endif
		for(vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			(*it)->update_pose();
		}
	#if DEBUG
		cout << "Poses updated for all robots." << endl;
	#endif
	}

	/*
		Put all of our puck locations and all of our robots into a protobuf message
	*/
	void
	build_gui_map(antixtransfer::SendMap_GUI *gui_map) {
		for (vector<Puck *>::iterator it = pucks.begin(); it != pucks.end(); it++) {
			antixtransfer::SendMap_GUI::Puck *puck = gui_map->add_puck();
			puck->set_x( (*it)->x );
			puck->set_y( (*it)->y );
			puck->set_held( (*it)->held );
		}
		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			antixtransfer::SendMap_GUI::Robot *robot = gui_map->add_robot();
			robot->set_team( (*it)->team );
			robot->set_id( (*it)->id );
			robot->set_x( (*it)->x );
			robot->set_y( (*it)->y );
			robot->set_a( (*it)->a );
		}
	}
};
#endif