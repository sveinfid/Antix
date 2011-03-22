/*
  Taken with little editing from rtv's Antix
*/

#include "entities.cpp"

// OS X users need <glut/glut.h> instead
#if defined(__APPLE__)&& defined(__MACH__)
#include <glut/glut.h>
#else
#include <GL/glut.h>
#endif
#include "zpr.h"
#include <utility>
#include <string>
//#include "glfont.h"
#include <sstream>

using namespace std;

int winsize = 600;

string master_host;
// we use client port when connecting to the master
string master_req_port = "7771";
string master_sub_port = "7773";
string master_gui_req_port = "7774";

zmq::socket_t *master_req_sock;
zmq::socket_t *master_sub_sock;
zmq::socket_t *master_gui_req_sock;
vector<zmq::socket_t *> req_sockets;

// from master
double world_size;
double robot_radius;
double robot_fov;
double home_radius;
// controls visualization of pixel data
bool show_data = false;
bool paused = false;

vector<Home> homes;
// these get cleared and rebuilt every turn
vector<Puck> pucks;
vector<Robot> robots;

antixtransfer::Node_list node_list;
antixtransfer::GUI_Request req;
antixtransfer::SendMap_GUI gui_map;

//keep track of which node to listen to
int current_node = 0;
// how we are viewing the world
#define VIEW_NO_NODE 0
#define VIEW_SINGLE_NODE 1
#define VIEW_ALL_NODE 2
int view_mode = VIEW_SINGLE_NODE;

//gui scoring stuff
int jscore;
string stringScore;
stringstream ss;
char tempCharToDisplayScore;

void
associate_robot_with_home(Robot *r) {
	vector<Home>::const_iterator homes_end = homes.end();
	for (vector<Home>::iterator it = homes.begin(); it != homes_end; it++) {
		if (it->team == r->team) {
			r->home = &*it;
			return;
		}
	}

	cerr << "Error: failed to associate robot with home" << endl;
	cerr << "\tRobot has team id " << r->team << endl;
	cerr << "Homes:" << endl;

	for (vector<Home>::iterator it = homes.begin(); it != homes.end(); it++) {
		cerr << "\tHome with team id " << it->team << endl;
	}
}

/*
  Send a message to all nodes requesting updated map data
  Receive N map responses, N = number of nodes
*/
void
rebuild_entity_db() {
	pucks.clear();
	robots.clear();
	/*
	int node_iterator = 0;

	// send request to every node
	for (vector<zmq::socket_t *>::iterator it = req_sockets.begin(); it != req_sockets.end(); it++) {
		//do not sent to every node, just current node 
		//antix::send_blank(*it);
		cout << "CURRENT NODE: " << node_iterator << endl;
		if(node_iterator == abs(current_node%node_list.node_size()))
		{
			cout << "SEND MESSAGE: " << node_iterator << endl;
			antix::send_blank(*it);
		}
		node_iterator++;
			
	}*/

	// Request scores from master
	antix::send_blank(master_gui_req_sock);
	
#if DEBUG
	cout << "Sync: Sent entity requests to nodes." << endl;
#endif

	int node_iterator = 0;
	
	// wait on response from each node
	vector<zmq::socket_t *>::const_iterator socks_end = req_sockets.end();
#ifndef NDEBUG
	int sockets_count = 0;
#endif
	for (vector<zmq::socket_t *>::const_iterator it = req_sockets.begin(); it != socks_end; it++) {
#ifndef NDEBUG
		sockets_count++;
#endif
		
		// do not wait for every node, just current node
		if(view_mode != VIEW_NO_NODE &&
			(view_mode == VIEW_ALL_NODE || (node_iterator == abs(current_node%node_list.node_size()))))
		{
			req.set_r(true);
			antix::send_pb(*it, &req);
			antix::recv_pb(*it, &gui_map, 0);
			
			// add received pucks
			int puck_size = gui_map.puck_size();
			for (int l = 0; l < puck_size; l++) {
				pucks.push_back( Puck(gui_map.puck(l).x(), gui_map.puck(l).y(), false ) );
			}

			// and robots
			int robot_size = gui_map.robot_size();
			for (int l = 0; l < robot_size; l++) {
				Robot r(gui_map.robot(l).x(), gui_map.robot(l).y(),
					gui_map.robot(l).team(), gui_map.robot(l).a());
				associate_robot_with_home(&r);
				assert(r.home != NULL);
				robots.push_back(r);
			}
		}
		else {
			req.set_r(false);
			antix::send_pb(*it, &req);
			antix::recv_blank(*it);
		}
		node_iterator++;
	}
	assert(sockets_count == req_sockets.size());
#ifndef NDEBUG
	cout << "Sync: After rebuilding db, know about " << robots.size() << " robots and " << pucks.size() << " pucks." << endl;
#endif

	// Read response from master: contains update of scores
	antixtransfer::Scores scores_msg;
	antix::recv_pb(master_gui_req_sock, &scores_msg, 0);

	for (int i = 0; i < scores_msg.score_size(); i++) {
		Home *h = NULL;
		// find the home associated with the score we're on
		for (vector<Home>::iterator it2 = homes.begin(); it2 != homes.end(); it2++) {
			if (it2->team == scores_msg.score(i).id()) {
				h = &*it2;
				break;
			}
		}
		assert(h != NULL);
		h->score = scores_msg.score(i).score();
	}
}

void
UpdateAll() {
	rebuild_entity_db();
}

// GLUT callback functions ---------------------------------------------------

// update the world - this is called whenever GLUT runs out of events
// to process
static void
idle_func( void ) {
	UpdateAll();
}

static void
timer_func( int dummy ) {
	glutPostRedisplay(); // force redraw
}

// draw a robot
void
Draw(const Robot *r) {
	glPushMatrix();

	// shift into this robot's local coordinate frame
	glTranslatef( r->x, r->y, 0 );
	glRotatef( antix::rtod(r->a), 0,0,1 );

	glColor3f( r->home->colour.r, r->home->colour.g, r->home->colour.b ); 
	
	double radius = robot_radius;
	
	// if robots are smaller than 4 pixels across, draw them as points
	if( (radius * (double)winsize/(double)world_size) < 2.0 ) {
		glBegin( GL_POINTS );
		glVertex2f( 0,0 );
		glEnd();
	} else {
		// draw a circular body
		glBegin(GL_LINE_LOOP);
		for( float a=0; a<(M_PI*2.0); a+=M_PI/16 )
			glVertex2f( sin(a) * radius, 
							cos(a) * radius );
		glEnd();

		// draw a nose indicating forward direction
		glBegin(GL_LINES);
		glVertex2f( 0, 0 );
		glVertex2f( robot_radius, 0 );
		glEnd();
	}

/* XXX not doing this for now
  if( show_data ) {
		glColor3f( 1,0,0 ); // red
		
		for (vector<SeeRobot>::iterator it = r->see_robots.begin(); it != r->see_robots.end(); it++) {
				float dx = it->range * cos(it->bearing);
				float dy = it->range * sin(it->bearing);
				
				glBegin( GL_LINES );
				glVertex2f( 0,0 );
				glVertex2f( dx, dy );
				glEnd();
		}
		
		glColor3f( 0.3,0.8,0.3 ); // light green
		
		for (vector<SeePuck>::iterator it = r->see_pucks.begin(); it != r->see_pucks.end(); it++) {
				float dx = it->range * cos(it->bearing);
				float dy = it->range * sin(it->bearing);
				
				glBegin( GL_LINES );
				glVertex2f( 0,0 );
				glVertex2f( dx, dy );
				glEnd();
		}
		
		glColor3f( 0.4,0.4,0.4 ); // grey

		// draw the sensor FOV
		glBegin(GL_LINE_LOOP);
		
		glVertex2f( 0, 0 );
		
		double right = -fov/2.0;
		double left = +fov/2.0;// + M_PI;
		double incr = fov/32.0;
		for( float a=right; a<left; a+=incr)
		  glVertex2f( cos(a) * range, 
						  sin(a) * range );

		glVertex2f( cos(left) * range, 
						sin(left) * range );
		
		glEnd();		
	}
*/
	
	// shift out of local coordinate frame
	glPopMatrix();
}

// utility
void
GlDrawCircle( double x, double y, double r, double count ) {
	glBegin(GL_LINE_LOOP);
	for( float a=0; a<(M_PI*2.0); a+=M_PI/count )
		glVertex2f( x + sin(a) * r, y + cos(a) * r );
	glEnd();
}

// render all robots in OpenGL
void
DrawAll() {
	vector<Robot>::const_iterator robots_end = robots.end();
#ifndef NDEBUG
	int robots_count = 0;
#endif
	for (vector<Robot>::const_iterator it = robots.begin(); it != robots_end; it++) {
#ifndef NDEBUG
		robots_count++;
#endif
		Draw(&*it);
	}
	assert(robots_count == robots.size());
	
	vector<Home>::const_iterator homes_end = homes.end();
#ifndef NDEBUG
	int homes_count = 0;
#endif
	for (vector<Home>::const_iterator it = homes.begin(); it != homes_end; it++) {
#ifndef NDEBUG
		homes_count++;
#endif
		glColor3f( it->colour.r, it->colour.g, it->colour.b );

		GlDrawCircle( it->x, it->y, home_radius, 16 );
		/*
		GlDrawCircle( it->x+world_size, it->y, home_radius, 16 );
		GlDrawCircle( it->x-world_size, it->y, home_radius, 16 );
		GlDrawCircle( it->x, it->y+world_size, home_radius, 16 );
		GlDrawCircle( it->x, it->y-world_size, home_radius, 16 );
		*/

		glColor3f(0.0, 1.0, 0.0); // Green

		glRasterPos2d(it->x, it->y);
		ss.str("");
		ss << it->score;
		stringScore = "Score: " + ss.str();
		cout << "String score " << stringScore << endl;
		for (string::iterator i = stringScore.begin(); i != stringScore.end(); ++i){
			tempCharToDisplayScore = *i;
			//glutBitmapCharacter(GLUT_BITMAP_9_BY_15, tempCharToDisplayScore);
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, tempCharToDisplayScore);
		}
	}
	assert(homes_count == homes.size());
	
	glColor3f( 1,1,1 ); // green
	glBegin( GL_POINTS );

	vector<Puck>::const_iterator pucks_end = pucks.end();
#ifndef NDEBUG
	int pucks_count = 0;
#endif
	for (vector<Puck>::const_iterator it = pucks.begin(); it != pucks_end; it++) {
#ifndef NDEBUG
		pucks_count++;
#endif
		glVertex2f( it->x, it->y );
	}
	assert(pucks_count == pucks.size());
	glEnd();
}

// draw the world - this is called whenever the window needs redrawn
static void
display_func( void ) {
	winsize = glutGet( GLUT_WINDOW_WIDTH );
	glClear( GL_COLOR_BUFFER_BIT );  
	DrawAll();
	glutSwapBuffers();

	// run this function again in about 50 msec
	glutTimerFunc( 20, timer_func, 0 );
}

static void
mouse_func(int button, int state, int x, int y) {
	if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN ) ) {
		paused = !paused;
	}
}

static void
processSpecialKeys(int key, int x, int y) {
	
	switch(key) {
		case GLUT_KEY_LEFT : 
			current_node--;
			//cout << "PRESSED LEFT KEY" << abs(current_node%node_list.node_size()) << endl;
			break;
		case GLUT_KEY_RIGHT : 
			current_node++; 
			//cout << "PRESSED RIGHT KEY" << abs(current_node%node_list.node_size()) << endl;
			break;
		case GLUT_KEY_UP : 
			if (view_mode == VIEW_NO_NODE)
				view_mode = VIEW_SINGLE_NODE;
			else if (view_mode == VIEW_SINGLE_NODE)
				view_mode = VIEW_ALL_NODE;
			//cout << "PRESSED UP KEY" << all_node << endl;
			break;
		case GLUT_KEY_DOWN : 
			if (view_mode == VIEW_ALL_NODE)
				view_mode = VIEW_SINGLE_NODE;
			else if (view_mode == VIEW_SINGLE_NODE)
				view_mode = VIEW_NO_NODE;
			//cout << "PRESSED DOWN KEY" << all_node << endl;
			break;
			
	}
}


//
// Robot static member methods ---------------------------------------------

void
InitGraphics( int argc, char* argv[] ) {
	// initialize opengl graphics
	glutInit( &argc, argv );
	glutInitWindowSize( winsize, winsize );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA );
	glutCreateWindow( argv[0] ); // program name
	glClearColor( 0.1,0.1,0.1,1 ); // dark grey
	glutDisplayFunc( display_func );
	glutTimerFunc( 50, timer_func, 0 );
	glutMouseFunc( mouse_func );
	glutSpecialFunc( processSpecialKeys ); //handle user key press (special key: LEFT/RIGHT)
	glutIdleFunc( idle_func );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glEnable( GL_BLEND );
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluOrtho2D( 0,1,0,1 );
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glScalef( 1.0/world_size, 1.0/world_size, 1 ); 
	glPointSize( 4.0 );
	zprInit();
}

void
UpdateGui() {
	glutMainLoop();
}

int
main(int argc, char **argv) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	zmq::context_t context(1);
	srand( time(NULL) );
	srand48( time(NULL) );
	
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <IP of master>" << endl;
		return -1;
	}
	master_host = string(argv[1]);

	// connect to master
	cout << "Connecting to master..." << endl;
	master_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	master_req_sock->connect(antix::make_endpoint(master_host, master_req_port));
	antix::send_str(master_req_sock, "init_gui_client");

	// Response from master contains simulation settings & our unique id (team id)
	antixtransfer::MasterServerClientInitialization init_response;
	antix::recv_pb(master_req_sock, &init_response, 0);
	robot_fov = init_response.fov();
	world_size = init_response.world_size();
	robot_radius = init_response.robot_radius();
	home_radius = init_response.home_radius();

	cout << "Connected." << endl;

	// Subscribe to master's publish socket. A node list will be received
	master_sub_sock = new zmq::socket_t(context, ZMQ_SUB);
	master_sub_sock->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	master_sub_sock->connect(antix::make_endpoint(master_host, master_sub_port));

	// GUI sock on master is where we receive scores for teams
	master_gui_req_sock = new zmq::socket_t(context, ZMQ_REQ);
	master_gui_req_sock->connect(antix::make_endpoint(master_host, master_gui_req_port));

	string s;
	while (s != "start") {
		s = antix::recv_str(master_sub_sock);
	}
	// block until receipt of list of nodes indicating simulation beginning
	antix::recv_pb(master_sub_sock, &node_list, 0);
	cout << "Received nodes from master" << endl;
	antix::print_nodes(&node_list);

	// connect to every node: we get robot/puck positions from each
	for (int i = 0; i < node_list.node_size(); i++) {
		zmq::socket_t *req_sock = new zmq::socket_t(context, ZMQ_REQ);
		req_sock->connect( antix::make_endpoint( node_list.node(i).ip_addr(), node_list.node(i).gui_port() ));
		req_sockets.push_back(req_sock);
	}
	cout << "Connected to all nodes" << endl;

	// populate home vector
	for (int i = 0; i < node_list.home_size(); i++) {
		Home h( node_list.home(i).x(), node_list.home(i).y(), home_radius, node_list.home(i).team() );
		homes.push_back(h);
		cout << "Added home: " << h.team << " at (" << h.x << ", " << h.y << ")" << endl;
	}

	// connect to all nodes & wait for updates
	rebuild_entity_db();

	InitGraphics(argc, argv);
	UpdateGui();

	return 0;
}
