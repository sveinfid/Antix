/*
	Hold robots, pucks, and their location in the world
	Calculate movements
	Find those on boundary & build boundary protobuf messages
	Find those outside range & build movement protobuf messages
*/

#ifndef MAP_H
#define MAP_H

#include "entities.cpp"

// for examine_border_cell()
#define LEFT_CELLS 0
#define RIGHT_CELLS 1

// for bots[][]
#define BOTS_TEAM_SIZE 1000
#define BOTS_ROBOT_SIZE 60000

using namespace std;

class Map {
public:
	// where we begin
	double my_min_x;
	// where neighbour begins
	double my_max_x;

	// the robots & pucks we control
	vector<Puck *> pucks; //TODO: might be able to remove this
	vector<Robot *> robots; //TODO: remove this
	// sent to us by neighbours
	vector<Puck> foreign_pucks;
	vector<Robot> foreign_robots;

	// Robots in the critical sections
	vector<Robot *> right_crit;
	vector<Robot *> left_crit;
	vector<Robot *> right_crit_new;
	vector<Robot *> left_crit_new;

	// indices into cindex that have foreign robots
	// we keep this for a turn until after update_poses()
	vector<Robot *> foreign_critical_robots;

	// We need to know homes to set robot's first last_x, last_y
	vector<Home *> all_homes;
	// And homes that we check for scoring
	vector<Home *> local_homes;

	Robot* bots[BOTS_TEAM_SIZE][BOTS_ROBOT_SIZE]; //bots[teamsize][amount of robots per team]

	// what each robot can see by team
	map<int, antixtransfer::sense_data *> sense_map;

	~Map() {
		for (vector<Puck *>::iterator it = pucks.begin(); it != pucks.end(); it++) {
			delete *it;
		}
		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			delete *it;
		}
		for (vector<Home *>::iterator it = all_homes.begin(); it != all_homes.end(); it++) {
			delete *it;
		}
		for (map<int, antixtransfer::sense_data *>::iterator it = sense_map.begin(); it != sense_map.end(); it++) {
			delete it->second;
		}
#if DEBUG
		cout << "Map deleted." << endl;
#endif
	}

	Map(double my_min_x,
		antixtransfer::Node_list *node_list,
		int initial_puck_amount,
		int my_id) : my_min_x(my_min_x) {

		my_max_x = my_min_x + antix::offset_size;
		antix::my_min_x = my_min_x;

		for (int i = 0; i < BOTS_TEAM_SIZE; i++) {
			for (int j = 0; j < BOTS_ROBOT_SIZE; j++) {
				bots[i][j] = NULL;
			}
		}

		// + 1000 as our calculations not exact in some places. Rounding error or?
		//Robot::matrix.resize(antix::matrix_width * antix::matrix_height + 1000);
		Robot::matrix.resize(antix::matrix_height * antix::matrix_height + 1000);
		cout << "Vision matrix has " << Robot::matrix.size() << " cells" << endl;

#if COLLISIONS
		// size of cell in one dimension
		double collision_cell_size = 2 * Robot::robot_radius;
		antix::cmatrix_width = ceil(antix::world_size / collision_cell_size);
		Robot::cmatrix.resize(antix::cmatrix_width * antix::cmatrix_width);
		for (vector<Robot *>::iterator it = Robot::cmatrix.begin(); it != Robot::cmatrix.end(); it++) {
			*it = NULL;
		}
		cout << "Collision matrix has " << Robot::cmatrix.size() << " cells." << endl;
#endif

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
			all_homes.push_back(h);
#if DEBUG
			cout << "Added home " << h->team << endl;
#endif
		}

		// Take the subset of those homes that have area within our section
		// of the map and place in another vector
		for (vector<Home *>::iterator it = all_homes.begin(); it != all_homes.end(); it++) {
			Home *h = *it;
			// Within our x range
			if (h->x + antix::home_radius >= my_min_x && h->x - antix::home_radius < my_max_x) {
				local_homes.push_back(h);

			// On first node, the homes may come from wrapping around the world
			} else if (my_min_x == 0) {
				if (h->x + antix::home_radius > antix::world_size) {
					local_homes.push_back(h);
				}

			// On last node, homes may come from wrapping around the world
			} else if (my_max_x == antix::world_size) {
				if (h->x - antix::home_radius < 0) {
					local_homes.push_back(h);
				}
			}
		}
	}

	Home *
	find_robot_home(int team) {
		for (vector<Home *>::iterator it = all_homes.begin(); it != all_homes.end(); it++) {
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

#if COLLISIONS
					// collision matrix
					// make sure we appear at an unused location
					unsigned int cindex = antix::CCell(r->x, r->y);
					//while ( Robot::did_collide( r, cindex, r->x, r->y ) != NULL ) {
					// do not collide, do not spawn in a critical section
					while ( Robot::did_collide( r, cindex, r->x, r->y ) != NULL || in_critical_section(r->x) != 0 ) {
						r->random_warp(my_min_x, my_max_x);
						cindex = antix::CCell(r->x, r->y);
					}
					r->cindex = cindex;
					Robot::cmatrix[r->cindex] = r;
#endif

					// bots[][] array
					assert(r->team < BOTS_TEAM_SIZE);
					assert(r->id < BOTS_ROBOT_SIZE);
					bots[r -> team][r -> id] = r; //XXX Gordon's Test

					// vector of all robots
					robots.push_back(r); //TODO, remove robots
					
					// sensor matrix
					unsigned int index = antix::Cell(r->x, r->y);
					r->index = index;
					Robot::matrix[index].robots.push_back( r );

#if DEBUG
					cout << "Created a bot: Team: " << r->team << " id: " << r->id << " at (" << r->x << ", " << r->y << ")" << endl;
#endif
				}
			}
		}
	}

	/*
		Place puck randomly within our section of the map
		Make sure it doesn't fall within a home
	*/
	void
	respawn_puck(Puck *p) {
		p->x = antix::rand_between( my_min_x, my_max_x );
		p->y = antix::rand_between( 0, antix::world_size );

		Home *h = Robot::is_puck_in_home(p, &local_homes);
		if (h != NULL) {
			return respawn_puck(p);
		}

		// only add in cell when we find one we're staying in
		unsigned int index = antix::Cell(p->x, p->y);
		p->index = index;
		Robot::matrix[index].pucks.push_back( p );
	}

	/*
		Place pucks randomly within our region
	*/
	void
	generate_pucks(int initial_puck_amount) {
		for (int i = 0; i < initial_puck_amount; i++) {
			// XXX is this -0.01 needed?
			//pucks.push_back( new Puck(my_min_x, my_max_x - 0.01) );
			Puck *p = new Puck(my_min_x, my_max_x - 0.01);
			respawn_puck(p);
			pucks.push_back( p );
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

	Robot *
	find_robot(int team, int id) {
#if DEBUG
		cout << "Trying to find robot with team " << team << " and id " << id << endl;
#endif
		assert(team < BOTS_TEAM_SIZE);
		assert(id < BOTS_ROBOT_SIZE);

		return bots[team][id];
		//Matrix(cell(x,y))
		
		/*
		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			if ((*it)->team == team && (*it)->id == id)
				return *it;
		}
		return NULL;*/
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

	/*
		Add robot to local records
		If collision cell it enters is occupied, return NULL

		If force is true, force addition of robot despite collision
	*/
	Robot *
	add_robot(double x,
		double y,
		int id,
		int team,
		double a,
		double v,
		double w,
		bool has_puck,
		double last_x,
		double last_y)
		{

#if DEBUG
		cout << "Moving: added new robot team " << team << " id " << id << " with a " << a << " w " << w << " (" << x << ", " << y << ") (Turn " << antix::turn << ")" << endl;
#endif
		Robot *r = new Robot(x, y, id, team, last_x, last_y);
		r->a = a;
		r->v = v;
		r->w = w;
		r->has_puck = has_puck;

#if COLLISIONS
		// collision matrix
		unsigned int new_cindex = antix::CCell( x, y );
		// If cell occupied (or collision occur), reject robot
		Robot *r2 = Robot::did_collide( r, new_cindex, x, y );
		if (r2 != NULL) {
			// Collide our local robot
			// XXX ?
			//Robot::cmatrix[new_cindex]->collide();
			delete r;
			return NULL;
		}

		// Cell is free
		r->cindex = new_cindex;
		Robot::cmatrix[new_cindex] = r;
#endif

		// Place in critical section if necessary

		// should always be in one when adding a moved bot?
		assert( in_critical_section( r-> x ) != 0 );

		// left crit section
		if (in_critical_section( r-> x ) == 1) {
			r->critical_section = &left_crit_new;
			left_crit_new.push_back( r );

		// right crit section
		} else if ( in_critical_section( r->x ) == 2 ) {
			r->critical_section = &right_crit_new;
			right_crit_new.push_back( r );
		}
		
		// bots[][] array
		assert(r->team < BOTS_TEAM_SIZE);
		assert(r->id < BOTS_ROBOT_SIZE);
		bots[r->team][r->id] = r;

		// vector of all robots
		robots.push_back(r);

		// sensor matrix
		unsigned int new_index = antix::Cell( x, y );
		r->index = new_index;
		Robot::matrix[new_index].robots.push_back( r );

		// If the robot is carrying a puck, we have to add a puck to our records
		if (r->has_puck) {
			Puck *p = new Puck(r->x, r->y, true);
			p->robot = r;
			pucks.push_back(p);

			r->puck = p;

			p->index = new_index;
			Robot::matrix[new_index].pucks.push_back( p );

			assert(r->has_puck == true);
			assert(r->puck->robot == r);
			assert(p->robot == r);
			assert(r->puck == p);
		}

		return r;
	}

	/*
		Robot has been found to be outside of our map portion
		Add the relevant data to a new Robot entry in the given move_bot message
	*/
	void
	add_robot_to_move_msg(Robot *r, antixtransfer::move_bot *move_bot_msg) {
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
		r_move->set_bbox_x_min(r->sensor_bbox.x.min);
		r_move->set_bbox_x_max(r->sensor_bbox.x.max);
		r_move->set_bbox_y_min(r->sensor_bbox.y.min);
		r_move->set_bbox_y_max(r->sensor_bbox.y.max);

		vector<int>::const_iterator ints_end = r->ints.end();
		for (vector<int>::const_iterator it = r->ints.begin(); it != ints_end; it++)
			r_move->add_ints( *it );
		vector<double>::const_iterator doubles_end = r->doubles.end();
		for (vector<double>::const_iterator it = r->doubles.begin(); it != doubles_end; it++)
			r_move->add_doubles( *it );
#if DEBUG
		cout << "Moving robot with a " << r->a << " w " << r->w << " (Turn " << antix::turn << ")" << endl;
#endif
	}

	/*
		Remove the puck that robot is carrying, if it is carrying one
	*/
	void
	remove_puck(Robot *r) {
#if DEBUG
		cout << "Trying to remove puck of robot " << r->id << " team " << r->team << endl;
#endif
		if (!r->has_puck)
			return;

		assert(r->puck != NULL);
		assert(r->puck->robot == r);
		assert(r->puck->held == true);

		assert(r->puck->home == NULL);

		// remove puck from cell
#if DEBUG_ERASE_PUCK
		cout << "EraseAll puck #1 in remove_puck()" << endl;
#endif
		antix::EraseAll( r->puck, Robot::matrix[ r->puck->index ].pucks );

		// remove puck from vector
		// XXX expensive
#if DEBUG_ERASE_PUCK
		cout << "EraseAll puck #2 in remove_puck()" << endl;
#endif
		antix::EraseAll( r->puck, pucks );
		
		// remove record on robot to deleted puck
		delete r->puck;
		r->puck = NULL;
		r->has_puck = false;
	}

	/*
		Remove robot from our records, and any associated puck
	*/
	void
	remove_robot(Robot *r) {
		// delete the robot's puck (if holding) from vector & memory
		remove_puck(r);

		// remove from vector of all robots
		// XXX expensive
		antix::EraseAll( r, robots );

		// from bots[][]
		assert(r->team < BOTS_TEAM_SIZE);
		assert(r->id < BOTS_ROBOT_SIZE);
		bots[r->team][r->id] = NULL;

#if COLLISIONS
		// from collision matrix
		assert(Robot::cmatrix[r->cindex] == r);
		if (Robot::cmatrix[r->cindex] == r) {
			Robot::cmatrix[r->cindex] = NULL;
		}
#endif

		// from sense matrix
		antix::EraseAll( r, Robot::matrix[r->index].robots );

		// delete robot from memory
		delete r;

		return;
	}

	/*
		Build a map with one entry per team, each entry being a protobuf message stating
		what the robots in that team see

		The actual sense logic is from rtv's Antix
	*/
	void
	build_sense_messages() {
		// clear old sense data
		map<int, antixtransfer::sense_data *>::iterator sense_map_end = sense_map.end();
#ifndef NDEBUG
		int sense_map_count = 0;
#endif
		for (map<int, antixtransfer::sense_data *>::iterator it = sense_map.begin(); it != sense_map_end; it++) {
			delete it->second;
#ifndef NDEBUG
			sense_map_count++;
#endif
		}
		assert(sense_map_count == sense_map.size());
		sense_map.clear();

		// for every robot we have, build a message for it containing what it sees
		vector<Robot *>::const_iterator robots_end = robots.end();
#ifndef NDEBUG
		int robot_count = 0;
#endif
		antixtransfer::sense_data *team_msg;
		for (vector<Robot *>::const_iterator r = robots.begin(); r != robots_end; r++) {
#ifndef NDEBUG
			robot_count++;
#endif

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
			robot_pb->set_collided( (*r)->collided );

			vector<int>::const_iterator ints_end = (*r)->ints.end();
			for (vector<int>::const_iterator it = (*r)->ints.begin(); it != ints_end; it++)
				robot_pb->add_ints( *it );
			vector<double>::const_iterator doubles_end = (*r)->doubles.end();
			for (vector<double>::const_iterator it = (*r)->doubles.begin(); it != doubles_end; it++)
				robot_pb->add_doubles( *it );

			// we will now find what robots & pucks we can see, but before that,
			// clear our see_pucks (and see_robots when we care...)
			(*r)->see_pucks.clear();

			const int lastx( antix::CellNoWrap_x( (*r)->sensor_bbox.x.max) );
			const int lasty( antix::CellNoWrap_y( (*r)->sensor_bbox.y.max) );

			for (int x = antix::CellNoWrap_x( (*r)->sensor_bbox.x.min); x <= lastx; x++)
				for (int y = antix::CellNoWrap_y( (*r)->sensor_bbox.y.min); y <= lasty; y++)
					UpdateSensorsCell(x, y, *r, robot_pb);

			// now look at foreign robots
			/* XXX right now we don't care about foreign robots
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
				double absolute_heading = antix::fast_atan2( dy, dx );
				double relative_heading = antix::AngleNormalize(absolute_heading - (*r)->a);
				if ( fabs(relative_heading) > Robot::fov/2.0 )
					continue;

				// we can see the robot
				antixtransfer::sense_data::Robot::Seen_Robot *seen_robot = robot_pb->add_seen_robot();
				seen_robot->set_range( range );
				seen_robot->set_bearing( relative_heading );
			}
			*/

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
				double absolute_heading = antix::fast_atan2( dy, dx );
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
		assert(robot_count == robots.size());
#if DEBUG
		cout << "Sensors re-calculated." << endl;
#endif
	}
	
	/*
		- Look at each home that has area within our section of the map
		- Go through the pucks in that home
		- If lifetime for a puck is 0, increment the score of that team
		  and respawn the puck randomly within our section
		- Otherwise decrement puck's lifetime
	*/
	void
	update_scores() {
		vector<Home *>::const_iterator homes_end = local_homes.end();
		for (vector<Home *>::const_iterator it = local_homes.begin(); it != homes_end; it++) {
			Home *h = *it;
			// Must while loop since possibly altering the home->pucks vector
			vector<Puck *>::iterator it2 = h->pucks.begin();
			while (it2 != h->pucks.end()) {
				if ( (*it2)->lifetime <= 0 ) {
					// update score
					(*it)->score++;

					// disassociate puck from home
					(*it2)->home = NULL;

					// remove puck from sense matrix
#if DEBUG_ERASE_PUCK
		cout << "EraseAll puck #1 in update_scores()" << endl;
#endif
					antix::EraseAll( *it2, Robot::matrix[ (*it2)->index ].pucks );

					// respawn puck
					respawn_puck( *it2 );

					// remove from home->pucks, continue
					it2 = h->pucks.erase(it2);

				} else {
					(*it2)->lifetime--;
					it2++;
				}
			}
#if DEBUG
			cout << "After updating scores, home " << (*it)->team << " has score ";
			cout << (*it)->score << " turn " << antix::turn << endl;
#endif
		}
	}

	/*
		Return 1 if in left crit section, 2 if in right, 0 otherwise
	*/
	int
	in_critical_section(double x) {
		// check that the x is actually in our node
		assert(x >= my_min_x || x < my_max_x);

		if (x - Robot::robot_radius < my_min_x + Robot::vision_range)
			return 1;
		if (x + Robot::robot_radius > my_max_x - Robot::vision_range)
			return 2;

		return 0;
	}

	/*
		Add a robot at x, y which only exists for collision checking
	*/
	Robot *
	add_foreign_crit_robot(double x, double y) {
		Robot *r = new Robot(x, y, -1, 0.0);

		// put into cmatrix
		const int cindex = antix::CCell( r->x, r->y );

#ifndef NDEBUG
		// If the cell is occupied, see if that robot is listed in a critical section
		// This means that we told the other node about it (hopefully)
		if (Robot::cmatrix[cindex] != NULL) {
			bool found = false;
			for (vector<Robot *>::iterator it = left_crit.begin(); it != left_crit.end(); it++) {
				if (*it == Robot::cmatrix[cindex]) {
					found = true;
					break;
				}
			}
			for (vector<Robot *>::iterator it = left_crit_new.begin(); it != left_crit_new.end(); it++) {
				if (*it == Robot::cmatrix[cindex]) {
					found = true;
					break;
				}
			}
			for (vector<Robot *>::iterator it = right_crit.begin(); it != right_crit.end(); it++) {
				if (*it == Robot::cmatrix[cindex]) {
					found = true;
					break;
				}
			}
			for (vector<Robot *>::iterator it = right_crit_new.begin(); it != right_crit_new.end(); it++) {
				if (*it == Robot::cmatrix[cindex]) {
					found = true;
					break;
				}
			}
			assert( found == true );
		}
#endif

		assert( Robot::cmatrix[cindex] == NULL );

#ifndef NDEBUG
		Robot *collided = Robot::did_collide(r, cindex, r->x, r->y);
		if (collided != NULL) {
			cout << "Collision with robot at " << collided->x << ", " << collided->y << endl;
			cout << " and one we were adding at " << x << ", " << y << endl;
		}
		assert(collided == NULL);
#endif

		r->cindex = cindex;
		Robot::cmatrix[cindex] = r;

		return r;
	}

	/*
		Update poses for robots in our right critical section
		Add any moved to move_msg
		Add all critical region bots to crit_map
	*/
	void
	update_right_crit_region(antixtransfer::move_bot *move_msg, antixtransfer::SendMap *crit_map_ours) {

		move_msg->clear_robot();
		crit_map_ours->clear_robot();

		// Update poses for our local crit region bots
		// while loop since we may remove robots
		vector<Robot *>::iterator it = right_crit.begin();
		while ( it != right_crit.end() ) {
			Robot *r = *it;
			r->update_pose();
			it++;
		}

		// Robot may move out of node or leave crit section
		// while loop since we may remove robots
		it = right_crit.begin();
		while ( it != right_crit.end() ) {
			Robot *r = *it;

			// If robot's x is bigger than ours and smaller than our right neighbour's,
			// we send it to our right neighbour
			if (r->x >= my_max_x && r->x < my_max_x + antix::offset_size) {
				add_robot_to_move_msg(r, move_msg);
				remove_robot(r);
				it = right_crit.erase( it );
				continue;
			}

			// Otherwise if it's less than ours and smaller than our left neighbour's,
			// assume that we are the far right node: send it to our right neighbour
			else if (r->x < my_min_x) {
				add_robot_to_move_msg(r, move_msg);
				remove_robot(r);
				it = right_crit.erase( it );
				continue;
			}

			// Or out of crit section but remain in the node
			if ( in_critical_section( r->x ) == 0 ) {
				// and set its crit region vector to NULL
				r->critical_section = NULL;
				it = right_crit.erase( it );
				continue;
			}

			it++;
		}

#ifndef DEBUG
		// Make sure robot is not in both right_crit_new and right_crit
		for(vector<Robot *>::iterator it = right_crit.begin(); it != right_crit.end(); it++) {
			for (vector<Robot *>::iterator it2 = right_crit_new.begin(); it2 != right_crit_new.end(); it2++) {
				assert( *it != *it2 );
			}
		}
#endif

		// Add all bots in the right crit region to pb message
		for (vector<Robot *>::iterator it = right_crit.begin(); it != right_crit.end(); it++) {
			antixtransfer::SendMap::Robot *r = crit_map_ours->add_robot();
			r->set_x( (*it)->x );
			r->set_y( (*it)->y );
			r->set_team( -1 );
			r->set_id( -1 );
		}
		for (vector<Robot *>::iterator it = right_crit_new.begin(); it != right_crit_new.end(); it++) {
			antixtransfer::SendMap::Robot *r = crit_map_ours->add_robot();
			r->set_x( (*it)->x );
			r->set_y( (*it)->y );
			r->set_team( -1 );
			r->set_id( -1 );
		}
	}

	/*
		crit_map contains a list of robots in the right critical section of the left
		node

		- Add those robots to our cmatrix
		- Update the poses for our left critical region bots
		- Build move message
		- Remove the foreign robots again
	*/
	void
	update_left_crit_region(antixtransfer::SendMap *crit_map, antixtransfer::move_bot *move_msg, antixtransfer::SendMap *crit_map_ours) {

		move_msg->clear_robot();
		crit_map_ours->clear_robot();

		// Right now we only track foreign robots for collisions

		// Indices into cmatrix to remove when movement is complete
		vector<Robot *> temp_foreign_robots;
		const int robot_size = crit_map->robot_size();
		for (int i = 0; i < robot_size; i++) {
			//cout << "Add foreign crit in update_left_crit_region()" << endl;
			Robot *r = add_foreign_crit_robot(crit_map->robot(i).x(), crit_map->robot(i).y() );
			// track for later deletion
			temp_foreign_robots.push_back(r);
		}

		// Update poses of the robots in our left critical region
		// We also check if the robots move to another node or outside of critical
		// region

		// while loop since we may remove robots
		vector<Robot *>::iterator it = left_crit.begin();
		while ( it != left_crit.end() ) {
			Robot *r = *it;
			r->update_pose();
			it++;
		}

		// Then check if they moved to another node or left crit region

		// while loop since we may remove robots
		it = left_crit.begin();
		while ( it != left_crit.end() ) {
			Robot *r = *it;

			// if the robot moves to left node, remove from both crit region & records
			// add to move left message

			// If robot's x is less than ours and bigger than our left neighbours,
			// send to our left neighbour
			if (r->x < my_min_x && r->x > my_min_x - antix::offset_size) {
				add_robot_to_move_msg(r, move_msg);
				remove_robot(r);
				it = left_crit.erase( it );
				continue;

			// if it's bigger than ours and bigger than our right neighbour's,
			// assume we are the far left node: send it to our left neighbour
			} else if ( r->x >= my_max_x ) {
				add_robot_to_move_msg(r, move_msg);
				remove_robot(r);
				it = left_crit.erase( it );
				continue;
			}

			// if it moves right out of critical region, remove from crit region
			if ( in_critical_section( r->x ) == 0 ) {
				// and set its crit region vector to NULL
				r->critical_section = NULL;
				it = left_crit.erase( it );
				continue;
			}

			it++;
		}

		// Remove all the foreign critical section robots
		vector<Robot *>::const_iterator temp_foreign_robots_end = temp_foreign_robots.end();
		for (vector<Robot *>::const_iterator it = temp_foreign_robots.begin(); it != temp_foreign_robots_end; it++) {
			Robot *r = *it;
			assert( Robot::cmatrix[ r->cindex ] == r );
			Robot::cmatrix[ r->cindex ] = NULL;
			delete r;
		}
		temp_foreign_robots.clear();

		// Add all robots in our left critical section to crit_map_ours
		// XXX optimize
		for (vector<Robot *>::iterator it = left_crit.begin(); it != left_crit.end(); it++) {
			antixtransfer::SendMap::Robot *r_s = crit_map_ours->add_robot();
			r_s->set_x( (*it)->x );
			r_s->set_y( (*it)->y );
			r_s->set_team( -1 );
			r_s->set_id( -1 );
		}
		for (vector<Robot *>::iterator it = left_crit_new.begin(); it != left_crit_new.end(); it++) {
			antixtransfer::SendMap::Robot *r_s = crit_map_ours->add_robot();
			r_s->set_x( (*it)->x );
			r_s->set_y( (*it)->y );
			r_s->set_team( -1 );
			r_s->set_id( -1 );
		}
	}

	/*
		Take the final positions of the foreign robots in the crit section for this turn
	*/
	void
	add_critical_region_robots(antixtransfer::SendMap *crit_map) {
		for (int i = 0; i < crit_map->robot_size(); i++) {
			//cout << "Add foreign crit in add_critical_region_robots()" << endl;
			Robot *r = add_foreign_crit_robot(crit_map->robot(i).x(), crit_map->robot(i).y() );
			foreign_critical_robots.push_back(r);
		}
	}

	/*
		Go through our local robots & update their poses
	*/
	void
	update_poses() {
#if DEBUG
		cout << "Updating poses for all robots..." << endl;
#endif

		vector<Robot *>::const_iterator crit_end;
		// Move those robots that were newly added to critical section in the previous
		// turn into the main critical section vectors
		crit_end = right_crit_new.end();
		for (vector<Robot *>::iterator it = right_crit_new.begin(); it != crit_end; it++) {
			Robot *r = *it;
			r->critical_section = &right_crit;
			right_crit.push_back( r );
		}
		right_crit_new.clear();

		crit_end = left_crit_new.end();
		for (vector<Robot *>::iterator it = left_crit_new.begin(); it != crit_end; it++) {
			Robot *r = *it;
			r->critical_section = &left_crit;
			left_crit.push_back( r );
		}
		left_crit_new.clear();

		// Now move all robots that we can
		vector<Robot *>::const_iterator robots_end = robots.end();
#ifndef NDEBUG
		int robot_count = 0;
#endif
		Robot *r;
		for(vector<Robot *>::const_iterator it = robots.begin(); it != robots_end; it++) {
			r = *it;
			// Only update pose for those not in a critical region
			if (r->critical_section == NULL) {
				r->update_pose();

				// Check if the robot moved into a critical section
				// XXX These checks assume a robot cannot move out of node without first
				// being in critical section. Currently unenforced.

				// Left critical section
				if ( in_critical_section( r->x ) == 1 ) {
					r->critical_section = &left_crit_new;
					left_crit_new.push_back( r );

				// Right critical section
				} else if ( in_critical_section( r->x) == 2 ) {
					r->critical_section = &right_crit_new;
					right_crit_new.push_back( r );
				}
			}

#ifndef NDEBUG
			robot_count++;
#endif
		}
		assert(robot_count == robots.size());

#if COLLISIONS
#ifndef NDEBUG
		// n^2 check for missed collisions (overlapped)
		for (vector<Robot *>::const_iterator it = robots.begin(); it != robots_end; it++) {
			// Against all local robots
			for (vector<Robot *>::const_iterator it2 = robots.begin(); it2 != robots_end; it2++) {
				const Robot *r1 = *it;
				const Robot *r2 = *it2;
				if (r1 == r2)
					continue;

				const double dx( antix::WrapDistance( r2->x - r1->x ) );
				const double dy( antix::WrapDistance( r2->y - r1->y ) );
				const double range( hypot( dx, dy ) );

				if (range <= Robot::robot_radius + Robot::robot_radius) {
					cout << "Found a collision that was allowed." << endl;
					cout << "\tRobot " << r1->id << " team " << r1->team << " at (";
					cout << r1->x << ", " << r1->y << ") cell " << r1->cindex << endl;
					cout << "\tRobot " << r2->id << " team " << r2->team << " at (";
					cout << r2->x << ", " << r2->y << ") cell " << r2->cindex << endl;
				}
			}
			// Against foreign critical robots
			for (vector<Robot *>::iterator it2 = foreign_critical_robots.begin(); it2 != foreign_critical_robots.end(); it2++) {
				const Robot *r1 = *it;
				const Robot *r2 = *it2;
				if (r1 == r2)
					continue;

				const double dx( antix::WrapDistance( r2->x - r1->x ) );
				const double dy( antix::WrapDistance( r2->y - r1->y ) );
				const double range( hypot( dx, dy ) );

				if (range <= Robot::robot_radius + Robot::robot_radius) {
					cout << "Found a collision that was allowed. (FOREIGN)" << endl;
					cout << "\tRobot " << r1->id << " team " << r1->team << " at (";
					cout << r1->x << ", " << r1->y << ") cell " << r1->cindex << endl;
					cout << "\tRobot " << r2->id << " team " << r2->team << " at (";
					cout << r2->x << ", " << r2->y << ") cell " << r2->cindex << endl;
				}
			}
		}
#endif
#endif

		// Remove previous turn's foreign critical region robots
		for (vector<Robot *>::iterator it = foreign_critical_robots.begin(); it != foreign_critical_robots.end(); it++) {
			Robot *r = *it;

			assert( Robot::cmatrix[ r->cindex ] == r );
			Robot::cmatrix[ r->cindex ] = NULL;
			delete r;
		}
		foreign_critical_robots.clear();

#if DEBUG
		cout << "Poses updated for all robots." << endl;
#endif
	}

	/*
		Add all robots in the right critical section to the SendMap message
	*/
	void
	build_right_crit_map(antixtransfer::SendMap *crit_map) {
		crit_map->clear_robot();
		for (vector<Robot *>::iterator it = right_crit.begin(); it != right_crit.end(); it++) {
			antixtransfer::SendMap::Robot *r = crit_map->add_robot();
			r->set_x( (*it)->x );
			r->set_y( (*it)->y );
			r->set_team( -1 );
			r->set_id( -1 );
		}
		for (vector<Robot *>::iterator it = right_crit_new.begin(); it != right_crit_new.end(); it++) {
			antixtransfer::SendMap::Robot *r = crit_map->add_robot();
			r->set_x( (*it)->x );
			r->set_y( (*it)->y );
			r->set_team( -1 );
			r->set_id( -1 );
		}
	}

	/*
		Put all of our puck locations and all of our robots into a protobuf message
	*/
	void
	build_gui_map(antixtransfer::SendMap_GUI *gui_map) {
		gui_map->clear_puck();
		gui_map->clear_robot();

		vector<Puck *>::const_iterator pucks_end = pucks.end();
#ifndef NDEBUG
		int pucks_count = 0;
#endif
		for (vector<Puck *>::const_iterator it = pucks.begin(); it != pucks_end; it++) {
#ifndef NDEBUG
			pucks_count++;
#endif
			antixtransfer::SendMap_GUI::Puck *puck = gui_map->add_puck();
			puck->set_x( (*it)->x );
			puck->set_y( (*it)->y );
			puck->set_held( (*it)->held );
		}
		assert(pucks_count == pucks.size());

		vector<Robot *>::const_iterator robots_end = robots.end();
#ifndef NDEBUG
		int robots_count = 0;
#endif
		for (vector<Robot *>::const_iterator it = robots.begin(); it != robots_end; it++) {
#ifndef NDEBUG
			robots_count++;
#endif
			antixtransfer::SendMap_GUI::Robot *robot = gui_map->add_robot();
			robot->set_team( (*it)->team );
			robot->set_id( (*it)->id );
			robot->set_x( (*it)->x );
			robot->set_y( (*it)->y );
			robot->set_a( (*it)->a );
		}
		assert(robots_count == robots.size());
	}

	/*
		from rtv's Antix
	*/
	inline void
	UpdateSensorsCell(unsigned int x, unsigned int y, Robot *r, antixtransfer::sense_data::Robot *robot_pb) {
		//unsigned int index( antix::CellWrap(x) + ( antix::CellWrap(y) * antix::matrix_width ) );
		//if (x < 0 || x > antix::matrix_width || y < 0 || y > antix::matrix_height)

		// we don't wrap along x, so don't care about these cases
		// XXX something like this may make sense
		// x < our min coord, x > our max coord
		//if (x < 0 || x > antix::matrix_width)
		//	return;

		// We don't wrap x, just y (since map split along x axis)
		unsigned int index( x + (antix::CellWrap(y) * antix::matrix_height) );
		//cout << "Index in updatesensors cell " << index << " x " << x << " y " << antix::CellWrap(y) << endl;
		assert( index < Robot::matrix.size());

		TestRobotsInCell( Robot::matrix[index], r, robot_pb );
		TestPucksInCell( Robot::matrix[index], r, robot_pb );
	}

	void
	TestRobotsInCell(const MatrixCell& cell, Robot *r, antixtransfer::sense_data::Robot *robot_pb) {
		// look at robots in this cell and see if we can see them
		vector<Robot *>::const_iterator robots_end = cell.robots.end();
#ifndef NDEBUG
		int robots_count = 0;
#endif
		for (vector<Robot *>::const_iterator other = cell.robots.begin(); other != robots_end; other++) {
#ifndef NDEBUG
			robots_count++;
#endif
			// we don't look at ourself
			if (r == *other)
				continue;
			
			const double dx( antix::WrapDistance( (*other)->x - r->x ) );
			if ( fabs(dx) > Robot::vision_range )
				continue;

			const double dy( antix::WrapDistance( (*other)->y - r->y ) );
			if ( fabs(dy) > Robot::vision_range )
				continue;

			//double range = hypot( dx, dy );
			//if (range > Robot::vision_range )
			const double dsq = dx*dx + dy*dy;
			if ( dsq > Robot::vision_range_squared )
				continue;

			// check that it's in fov
			const double absolute_heading = antix::fast_atan2( dy, dx );
			const double relative_heading = antix::AngleNormalize(absolute_heading - r->a);
			if ( fabs(relative_heading) > Robot::fov/2.0 )
				continue;

			// we can see the robot
			antixtransfer::sense_data::Robot::Seen_Robot *seen_robot = robot_pb->add_seen_robot();
			//seen_robot->set_range( range );
			seen_robot->set_range( sqrt(dsq) );
			seen_robot->set_bearing( relative_heading );
		}
		assert(robots_count == cell.robots.size());
	}

	void
	TestPucksInCell(const MatrixCell& cell, Robot *r, antixtransfer::sense_data::Robot *robot_pb) {
		// check which pucks in this cell we can see
		vector<Puck *>::const_iterator pucks_end = cell.pucks.end();
#ifndef NDEBUG
		int pucks_count = 0;
#endif
		for (vector<Puck *>::const_iterator puck = cell.pucks.begin(); puck != pucks_end; puck++) {
#ifndef NDEBUG
			pucks_count++;
#endif
			const double dx( antix::WrapDistance( (*puck)->x - r->x ) );
			if ( fabs(dx) > Robot::vision_range )
				continue;

			const double dy( antix::WrapDistance( (*puck)->y - r->y ) );
			if ( fabs(dy) > Robot::vision_range )
				continue;

			//double range = hypot( dx, dy );
			//if (range > Robot::vision_range)
			const double dsq = dx*dx + dy*dy;
			if ( dsq > Robot::vision_range_squared )
				continue;

			// fov check
			const double absolute_heading = antix::fast_atan2( dy, dx );
			const double relative_heading = antix::AngleNormalize( absolute_heading - r->a );
			if ( fabs(relative_heading) > Robot::fov/2.0 )
				continue;

			// we can see the puck
			antixtransfer::sense_data::Robot::Seen_Puck *seen_puck = robot_pb->add_seen_puck();
			const double range_approx = sqrt(dsq);
			//seen_puck->set_range( range );
			seen_puck->set_range( range_approx );
			seen_puck->set_bearing ( relative_heading );
			seen_puck->set_held( (*puck)->held );

			//r->see_pucks.push_back(SeePuck(*puck, range));
			r->see_pucks.push_back(SeePuck(*puck, range_approx));

			// make sure this puck is in vector as well
#ifndef NDEBUG
			bool found = false;
			for (vector<Puck *>::iterator it2 = pucks.begin(); it2 != pucks.end(); it2++) {
				if ( (*it2) == *puck ) {
					found = true;
					break;
				}
			}
			assert(found == true);
#endif
		}
		assert(pucks_count == cell.pucks.size());
	}
};
#endif
