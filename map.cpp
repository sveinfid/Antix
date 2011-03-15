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
#define BOTS_ROBOT_SIZE 6000

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

	// We need to know homes to set robot's first last_x, last_y
	vector<Home *> homes;
	
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
		for (vector<Home *>::iterator it = homes.begin(); it != homes.end(); it++) {
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
			homes.push_back(h);
#if DEBUG
			cout << "Added home " << h->team << endl;
#endif
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

#if COLLISIONS
					// collision matrix
					// make sure we appear at an unused location
					unsigned int cindex = antix::CCell(r->x, r->y);
					while ( Robot::cmatrix[cindex] != NULL ) {
						r->random_warp(my_min_x, my_max_x);
						cindex = antix::CCell(r->x, r->y);
					}
					r->cindex = cindex;
					Robot::cmatrix[cindex] = r;
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

		Home *h = Robot::is_puck_in_home(p, &homes);
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
		double last_y) {

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
		// XXX handle this better...
		if (Robot::cmatrix[new_cindex] != NULL) {
			//cerr << "Error: Moved to an already occupied collision cell!" << endl;
			r->cindex = -1;
			r->collide(Robot::cmatrix[new_cindex]);
		} else {
			r->cindex = new_cindex;
			Robot::cmatrix[new_cindex] = r;
		}
		// may not make sense since cell may be taken
		//r->cindex = new_cindex;
#endif
		
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
		antix::EraseAll( r->puck, Robot::matrix[ r->puck->index ].pucks );

		// remove puck from vector
		// XXX expensive
		antix::EraseAll( r->puck, pucks );
		
		// remove record on robot to deleted puck
		delete r->puck;
		r->puck = NULL;
		r->has_puck = false;
	}

	/*
		Remove robot from our records, and any associated puck
	*/
	vector<Robot *>::iterator
	remove_robot(vector<Robot *>::iterator & it) {
		Robot *r = *it;

		// delete the robot's puck (if holding) from vector & memory
		remove_puck(r);

		// remove from vector of all robots
		// XXX expensive
		antix::EraseAll( r, robots );

		// from bots[][]
		assert(r->team < BOTS_TEAM_SIZE);
		assert(r->id < BOTS_ROBOT_SIZE);
		bots[r->team][r->id] = NULL;

		// from collision matrix
#if COLLISIONS
		//assert(Robot::cmatrix[r->cindex] == r);
		if (Robot::cmatrix[r->cindex] == r) {
			Robot::cmatrix[r->cindex] = NULL;
		}
#endif

		// delete robot from memory
		int index = r->index;
		delete r;

		// from matrix
		return Robot::matrix[index].robots.erase( it );
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
		Debug function. Make sure the given robot is actually in the given map
		True if it is, false otherwise
	*/
	bool
	check_in_border_robot(antixtransfer::SendMap *map, Robot *r) {
		for (int i = 0; i < map->robot_size(); i++) {
			if (map->robot(i).id() == r->id && map->robot(i).team() == r->team)
				return true;
		}
#if DEBUG
		cout << "Error: robot not found in border map! In cell " << r->index << " id " << r->id << " team " << r->team << " at (" << r->x << ", " << r->y << ")" << endl;
#endif
		return false;
	}

	/*
		Debug function: Look through robot vector and find those that need to be
		in border map. Make sure they are actually in the given border maps

		We don't check puck since it's hard to identify exactly due to no id
	*/
	void
	check_correct_robots_in_border(antixtransfer::SendMap *border_map_left, antixtransfer::SendMap *border_map_right) {
		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			if ((*it)->x > my_max_x - Robot::vision_range) {
				//cout << "Robot at " << (*it)->x << ", " << (*it)->y << " cell " << antix::Cell( (*it)->x, (*it)->y ) << endl;
				assert(check_in_border_robot(border_map_right, *it));
			}
			else if ((*it)->x < my_min_x + Robot::vision_range) {
				//cout << "Robot at " << (*it)->x << ", " << (*it)->y << " cell " << antix::Cell( (*it)->x, (*it)->y ) << endl;
				assert(check_in_border_robot(border_map_left, *it));
			}
		}
	}

	/*
		Debug function. Make sure the given Robot is in the given move_bot message
		True if it is, false otherwise
	*/
	bool
	check_in_move_msg(Robot *r, antixtransfer::move_bot *move_bot) {
		for (int i = 0; i < move_bot->robot_size(); i++) {
			if (move_bot->robot(i).id() == r->id && move_bot->robot(i).team() == r->team)
				return true;
		}
#if DEBUG
		cout << "Error: robot not found in move msg! In cell " << r->index << endl;
#endif
		return false;
	}

	/*
		Debug function: Look through robot vector & find those that need to be moved
		Make sure they are actually in the given move messages
	*/
	void
	check_correct_robots_in_move(antixtransfer::move_bot *move_left_msg,
		antixtransfer::move_bot *move_right_msg) {

		for (vector<Robot *>::iterator it = robots.begin(); it != robots.end(); it++) {
			// If robot's x is less than ours and bigger than our left neighbours, send
			// to our left neighbour
			if ((*it)->x < my_min_x && (*it)->x > my_min_x - antix::offset_size) {
				assert(check_in_move_msg(*it, move_left_msg));

			// Otherwise if it's less than ours and smaller than our left neighbour's,
			// assume that we are the far right node: send it to our right neighbour
			} else if ((*it)->x < my_min_x) {
				assert(check_in_move_msg(*it, move_right_msg));

			// If robot's x is bigger than ours and smaller than our right neighbour's,
			// we send it to our right neighbour
			} else if ((*it)->x >= my_max_x && (*it)->x < my_max_x + antix::offset_size) {
				assert(check_in_move_msg(*it, move_right_msg));

			// Otherwise it's bigger than ours and bigger than our right neighbour's,
			// assume we are the far left node: send it to our left neighbour
			} else if ((*it)->x >= my_max_x) {
				assert(check_in_move_msg(*it, move_left_msg));

			}
		}
	}

	/*
		Look at the pucks and robots in the given cell
		Add them to the respective protobuf messages if necessary
	*/
	void
	examine_border_cell(int index,
		antixtransfer::move_bot *move_left_msg,
		antixtransfer::move_bot *move_right_msg,
		antixtransfer::SendMap *border_map_left,
		antixtransfer::SendMap *border_map_right,
		int side) {

		// Robots
		// while loop as iterator may be updated due to deletion
		// NOTE: Can't remove function call to end it seems or fails asserts when
		// removing robots. Possibly due to removal of final robot sometimes?
		vector<Robot *>::iterator it_robot = Robot::matrix[index].robots.begin();
		while (it_robot != Robot::matrix[index].robots.end()) {
			// First we look whether robot needs to move to another node

			if (side == LEFT_CELLS) {
				// If robot's x is less than ours and bigger than our left neighbours,
				// send to our left neighbour
				if ((*it_robot)->x < my_min_x && (*it_robot)->x > my_min_x - antix::offset_size) {
#if DEBUG
					cout << "Moving robot " << (*it_robot)->id << " on team " << (*it_robot)->team << " to left node (1)" << endl;
#endif
					add_robot_to_move_msg(*it_robot, move_left_msg);
					it_robot = remove_robot(it_robot);
					continue;

				// Otherwise if it's less than ours and smaller than our left neighbour's,
				// assume that we are the far right node: send it to our right neighbour
				} else if ((*it_robot)->x < my_min_x) {
#if DEBUG
					cout << "Moving robot " << (*it_robot)->id << " on team " << (*it_robot)->team << " to right node (2)" << endl;
#endif
					add_robot_to_move_msg(*it_robot, move_right_msg);
					it_robot = remove_robot(it_robot);
					continue;
				}

			// Otherwise right side cell
			} else {
				// If robot's x is bigger than ours and smaller than our right neighbour's,
				// we send it to our right neighbour
				if ((*it_robot)->x >= my_max_x && (*it_robot)->x < my_max_x + antix::offset_size) {
#if DEBUG
					cout << "Moving robot " << (*it_robot)->id << " on team " << (*it_robot)->team << " to right node (3)" << endl;
#endif
					add_robot_to_move_msg(*it_robot, move_right_msg);
					it_robot = remove_robot(it_robot);
					continue;

				// Otherwise it's bigger than ours and bigger than our right neighbour's,
				// assume we are the far left node: send it to our left neighbour
				} else if ((*it_robot)->x >= my_max_x) {
#if DEBUG
					cout << "Moving robot " << (*it_robot)->id << " on team " << (*it_robot)->team << " to left node (4)" << endl;
#endif
					add_robot_to_move_msg(*it_robot, move_left_msg);
					it_robot = remove_robot(it_robot);
					continue;
				}
			}

			// Check whether on the border
			if (side == LEFT_CELLS) {
				if ((*it_robot)->x < my_min_x + Robot::vision_range)
					add_border_robot(border_map_left, *it_robot);

			// Right side
			} else {
				// Then if it doesn't move, check whether it's on the border
				if ((*it_robot)->x > my_max_x - Robot::vision_range)
					add_border_robot(border_map_right, *it_robot);
			}

			it_robot++;
		}

		vector<Puck *>::iterator it_puck = Robot::matrix[index].pucks.begin();
		vector<Puck *>::const_iterator end_puck = Robot::matrix[index].pucks.end();

		// Pucks
		for ( ; it_puck != end_puck; it_puck++) {
			if (side == LEFT_CELLS) {
				if ((*it_puck)->x < my_min_x + Robot::vision_range)
					add_border_puck(border_map_left, *it_puck);
			} else {
				if ((*it_puck)->x > my_max_x - Robot::vision_range)
					add_border_puck(border_map_right, *it_puck);
			}
		}
	}

	/*
		Look at the robots on the far left and far right of our matrix
		Find those which need to be moved to another node, add them to move messages,
		and remove.
		Find those which are within sight distance to tell our neighbours and add them
		to border messages.

		We do this at same time as we only want to iterate over the cells once
	*/
	void
	build_moves_and_border_entities(antixtransfer::move_bot *move_left_msg,
		antixtransfer::move_bot *move_right_msg,
		antixtransfer::SendMap *border_map_left,
		antixtransfer::SendMap *border_map_right) {

		// old move messages no longer valid
		move_left_msg->clear_robot();
		move_right_msg->clear_robot();

		// old maps are no longer valid
		border_map_left->clear_robot();
		border_map_left->clear_puck();
		border_map_right->clear_robot();
		border_map_right->clear_puck();

		// we are rebuilding those to send to our neighbours, they are doing the same
		// so clear our old data on neighbour's border entities
		foreign_pucks.clear();
		foreign_robots.clear();

		// Now we look at robots and pucks on the sides of the matrix

		// XXX all of these loops can probably be tightened to include less cols

		// look down the cells on far left
		// check 3 furthest left cols
		int left_limit = antix::matrix_left_x_col + 10;
		for (int x = antix::matrix_left_x_col - 10; x < left_limit; x++) {
			for (int y = 0; y < antix::matrix_height; y++) {
				// 2d array into 1d matrix: x + y*width
				int index = x + y * antix::matrix_height;
				//cout << "Index left " << index << " x " << x << " y " << y << endl;
				if (index >= Robot::matrix.size())
					continue;
				assert(index < Robot::matrix.size());
				// Look at the robots & pucks
				examine_border_cell(index, move_left_msg, move_right_msg, border_map_left, border_map_right, LEFT_CELLS);
			}
		}

		// and on far right
		// check 3 farthest right columns. start from column on furthest right
		// and go back
		int right_limit = antix::matrix_right_x_col - 10;
		for (int x = antix::matrix_right_x_col + 10; x > right_limit; x--) {
			for (int y = 0; y < antix::matrix_height; y++) {
				if (x < 0)
					continue;
				// 2d array into 1d matrix: x + y*width
				int index = x + y * antix::matrix_height;
				if (index >= Robot::matrix.size())
					continue;
				//cout << "Index right " << index << " x " << x << " y " << y << endl;
				assert(index < Robot::matrix.size());
				examine_border_cell(index, move_left_msg, move_right_msg, border_map_left, border_map_right, RIGHT_CELLS);
			}
		}

		// far right node can have index for bot move into far left
		for (int y = 0; y < antix::matrix_height; y++) {
			int index = 0 + y * antix::matrix_height;
			//cout << "Index far left " << index << " x " << 0 << " y " << y << endl;
			assert(index < Robot::matrix.size());
			examine_border_cell(index, move_left_msg, move_right_msg, border_map_left, border_map_right, LEFT_CELLS);
		}

		// far left node can have index for bot move into far right
		int far_right_limit = antix::matrix_right_world_x_col - 10;
		for (int x = antix::matrix_right_world_x_col + 10; x > far_right_limit; x--) {
			for (int y = 0; y < antix::matrix_height; y++) {
				if (x < 0)
					continue;
				int index = x + y * antix::matrix_height;
				if (index >= Robot::matrix.size())
					continue;
				assert(index < Robot::matrix.size());
				//cout << "Index far right " << index << " x " << x << " y " << y << endl;
				examine_border_cell(index, move_left_msg, move_right_msg, border_map_left, border_map_right, RIGHT_CELLS);
			}
		}

		// It's also possible for robots to wrap around to cells on far side of world
		// this may only be needed for far left node?
		/*
		for (int x = antix::matrix_right_world_x_col; x >= antix::matrix_right_world_x_col - 2; x--) {
			for (int y = 0; y < antix::matrix_height; y++) {
				// 2d array into 1d matrix: x + y*width
				int index = x + y * antix::matrix_height;
				assert(index < Robot::matrix.size());
				//assert(index < antix::matrix_height * antix::matrix_width);
				//cout << "Index FAR right " << index << " x " << x << " y " << y << endl;
				examine_border_cell(index, move_left_msg, move_right_msg, border_map_left, border_map_right, RIGHT_CELLS);
				//examine_border_cell(index, move_left_msg, move_right_msg, border_map_left, border_map_right, LEFT_CELLS);
			}
		}
		*/
		/*
		assert( antix::matrix_left_x_col < Robot::matrix.size() );
		examine_border_cell(antix::matrix_left_x_col, move_left_msg, move_right_msg, border_map_left, border_map_right, LEFT_CELLS);

		assert( antix::matrix_right_x_col < Robot::matrix.size() );
		examine_border_cell(antix::matrix_right_x_col, move_left_msg, move_right_msg, border_map_left, border_map_right, RIGHT_CELLS);
		*/

#ifndef NDEBUG
		check_correct_robots_in_border(border_map_left, border_map_right);
		check_correct_robots_in_move(move_left_msg, move_right_msg);
#endif
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
		// XXX only look at homes that are necessary to look at!
		vector<Home *>::const_iterator homes_end = homes.end();
		for (vector<Home *>::const_iterator it = homes.begin(); it != homes_end; it++) {
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
		Go through our local robots & update their poses
	*/
	void
	update_poses() {
#if DEBUG
		cout << "Updating poses for all robots..." << endl;
#endif
		vector<Robot *>::const_iterator robots_end = robots.end();
#ifndef NDEBUG
		int robot_count = 0;
#endif
		for(vector<Robot *>::const_iterator it = robots.begin(); it != robots_end; it++) {
			(*it)->update_pose();
#ifndef NDEBUG
			robot_count++;
#endif
		}
		assert(robot_count == robots.size());
#if DEBUG
		cout << "Poses updated for all robots." << endl;
#endif
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
