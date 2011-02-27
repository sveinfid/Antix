/*
  Taken with little editing from rtv's Antix
*/

#include "entities.cpp"

// OS X users need <glut/glut.h> instead
#include <GL/glut.h>

using namespace std;

int winsize = 600;

string master_host;
// we use client port when connecting to the master
string master_req_port = "7771";
string master_sub_port = "7773";

zmq::socket_t *master_req_sock;
zmq::socket_t *master_sub_sock;
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

void
associate_robot_with_home(Robot *r) {
  for (vector<Home>::iterator it = homes.begin(); it != homes.end(); it++) {
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

  // send request to every node
  for (vector<zmq::socket_t *>::iterator it = req_sockets.begin(); it != req_sockets.end(); it++) {
    antix::send_blank(*it);
  }
#if DEBUG
  cout << "Sent entity requests to nodes." << endl;
#endif

  // wait on response from each node
  for (vector<zmq::socket_t *>::iterator it = req_sockets.begin(); it != req_sockets.end(); it++) {
    antixtransfer::SendMap_GUI map;
    antix::recv_pb(*it, &map, 0);
    
    // add received pucks
    for (int l = 0; l < map.puck_size(); l++) {
      pucks.push_back( Puck(map.puck(l).x(), map.puck(l).y(), false ) );
    }

    // and robots
    for (int l = 0; l < map.robot_size(); l++) {
      Robot r(map.robot(l).x(), map.robot(l).y(), map.robot(l).team(), map.robot(l).a());
      associate_robot_with_home(&r);
      robots.push_back(r);
    }
  }
//#if DEBUG
  cout << "After rebuilding db, know about " << robots.size() << " robots and " << pucks.size() << " pucks." << endl;
//#endif
}

void
UpdateAll() {
  rebuild_entity_db();
}

// GLUT callback functions ---------------------------------------------------

// update the world - this is called whenever GLUT runs out of events
// to process
static void
idle_func( void )
{
  UpdateAll();
}

static void
timer_func( int dummy )
{
  glutPostRedisplay(); // force redraw
}

// draw a robot
void
Draw(Robot *r)
{
  glPushMatrix();

	// shift into this robot's local coordinate frame
  glTranslatef( r->x, r->y, 0 );
  glRotatef( antix::rtod(r->a), 0,0,1 );
  
	glColor3f( r->home->colour.r, r->home->colour.g, r->home->colour.b ); 
	
	double radius = robot_radius;
	
	// if robots are smaller than 4 pixels across, draw them as points
	if( (radius * (double)winsize/(double)world_size) < 2.0 )
	  {
		 glBegin( GL_POINTS );
		 glVertex2f( 0,0 );
		 glEnd();
	  }
	else
	  {
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
  if( show_data )
	 {
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
GlDrawCircle( double x, double y, double r, double count )
{
	glBegin(GL_LINE_LOOP);
	for( float a=0; a<(M_PI*2.0); a+=M_PI/count )
		glVertex2f( x + sin(a) * r, y + cos(a) * r );
	glEnd();
}

// render all robots in OpenGL
void
DrawAll()
{		
  for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		Draw(&*it);
  }
	
  for (vector<Home>::iterator it = homes.begin(); it != homes.end(); it++) {
    glColor3f( it->colour.r, it->colour.g, it->colour.b );

    GlDrawCircle( it->x, it->y, home_radius, 16 );
    GlDrawCircle( it->x+world_size, it->y, home_radius, 16 );
    GlDrawCircle( it->x-world_size, it->y, home_radius, 16 );
    GlDrawCircle( it->x, it->y+world_size, home_radius, 16 );
    GlDrawCircle( it->x, it->y-world_size, home_radius, 16 );
  }
	
	glColor3f( 1,1,1 ); // green
	glBegin( GL_POINTS );

  for (vector<Puck>::iterator it = pucks.begin(); it != pucks.end(); it++) {
		glVertex2f( it->x, it->y );
  }
	glEnd();
}

// draw the world - this is called whenever the window needs redrawn
static void
display_func( void ) 
{  
  winsize = glutGet( GLUT_WINDOW_WIDTH );
  glClear( GL_COLOR_BUFFER_BIT );  
  DrawAll();
  glutSwapBuffers();
	
  // run this function again in about 50 msec
  glutTimerFunc( 20, timer_func, 0 );
}

static void
mouse_func(int button, int state, int x, int y) 
{  
  if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN ) )
	 {
		paused = !paused;
	 }
}

//
// Robot static member methods ---------------------------------------------

void
InitGraphics( int argc, char* argv[] )
{
  // initialize opengl graphics
  glutInit( &argc, argv );
  glutInitWindowSize( winsize, winsize );
  glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA );
  glutCreateWindow( argv[0] ); // program name
  glClearColor( 0.1,0.1,0.1,1 ); // dark grey
  glutDisplayFunc( display_func );
  glutTimerFunc( 50, timer_func, 0 );
  glutMouseFunc( mouse_func );
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
}

void
UpdateGui()
{
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
