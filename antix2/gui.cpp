/*
  Taken with little editing from rtv's Antix
*/

#include "antix.cpp"

// OS X users need <glut/glut.h> instead
#include <GL/glut.h>

using namespace std;

int winsize = 600;

// from master
double world_size;
double robot_radius;
double robot_fov;
// controls visualization of pixel data
bool show_data = false;
bool paused = false;

vector<Puck> pucks;
vector<Robot> population;
vector<Home> homes;

/*
  This should rebuild our internal database of robots/pucks/etc
  XXX
*/
void
UpdateAll() {

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
  
	glColor3f( r->home->color.r, r->home->color.g, r->home->color.b ); 
	
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
	
	// shift out of local coordinate frame
  glPopMatrix();
}

// render all robots in OpenGL
void
DrawAll()
{		
  for (vector<Robot>::iterator it = robots.begin(); it != robots.end(); it++) {
		Draw(&*it);
  }
	
  for (vector<Home>::iterator it = homes.begin(); it != homes.end(); it++) {
    glColor3f( it->color.r, it->color.g, it->color.b );

    GlDrawCircle( it->x, it->y, it->r, 16 );
    GlDrawCircle( it->x+world_size, it->y, it->r, 16 );
    GlDrawCircle( it->x-world_size, it->y, it->r, 16 );
    GlDrawCircle( it->x, it->y+world_size, it->r, 16 );
    GlDrawCircle( it->x, it->y-world_size, it->r, 16 );
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

// utility
void
GlDrawCircle( double x, double y, double r, double count )
{
	glBegin(GL_LINE_LOOP);
	for( float a=0; a<(M_PI*2.0); a+=M_PI/count )
		glVertex2f( x + sin(a) * r, y + cos(a) * r );
	glEnd();
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
  InitGraphics(argc, argv);
  UpdateGui();

  return 0;
}
