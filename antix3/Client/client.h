/****
	client.cc
	Each client would be in control of 1 team
	Gordon, Wendy, Jerome
	
****/

#include <vector>

class Home
{
	public:
		double x, y, r;	
		Home(	double x, double y, double r );
};


class Robot
{
	public:
			 Home* home;
	 
		class Pose
		{
		public:
			double x,y,a; // 2d position and orientation
			
		Pose( double x, double y, double a ) : x(x), y(y), a(a) {}
		Pose() : x(0.0), y(0.0), a(0.0) {}

			//Pose( const Pose &p ) : x(p.x), y(p.y), a(p.a) {}	
			
			// get a random pose 
			static Pose Random()
			{
			  return Pose( drand48() * Robot::worldsize, 
								drand48() * Robot::worldsize, 
								Robot::AngleNormalize( drand48() * (M_PI*2.0)));
			}
		} pose; // instance: robot is located at this pose
		
		class Speed
		{		
		public:
			double v; // forward speed
			double w; // turn speed
	  	
			// constructor sets speeds to zero
		Speed() : v(0.0), w(0.0) {}		
		} speed; // instance: robot is moving this fast
		
		class SeeRobot
		{
		public:
		  const Home* home;
		  Pose pose;
		  Speed speed;
		  double range;
		  double bearing;
		  bool haspuck;
			
		SeeRobot( const Home* home, const Pose& p, const Speed& s, const double range, const double bearing, const bool haspuck )
		  : home(home), pose(p), speed(s), range(range), bearing(bearing), haspuck(haspuck)
			{ /* empty */}
	 };

	 vector<SeeRobot> see_robots;

	public: class Puck
	 {
	 public:
		 bool held;		 
		 unsigned int index;
 		 double x,y;
		 
		 /** constructor places a puck at specified pose */
		 //Puck( double x, double y ) : x(x), y(y), held(false) {}
		 
	 Puck() 
		:  held(false), index(0),x(0.0), y(0.0) 
			{  /* do nothing */ }		 
	 };		 
	 
	 static std::vector<Puck> pucks;

	 class SeePuck
	 {
	 public:
		 Puck* puck;
		 bool held;
		 double bearing;		 
		 double range;
		 
	 SeePuck( Puck* puck,  const double range, const double bearing, const bool held )
		: puck(puck), held(held), bearing(bearing), range(range) 
		 { /* empty */}
	 };
	 
	 /** A sense vector containing information about all the pucks
			 detected in my field of view */
	 std::vector<SeePuck> see_pucks;	 	 
#if DEBUGVIS
	 std::vector<Robot*> neighbors;
	 std::vector<Puck*> neighbor_pucks;
	 std::set<unsigned int> neighbor_cells;
#endif

	 // constructor
	 Robot( Home* home, const Pose& pose );
	 
	 // destructor
	 virtual ~Robot() {}
	 
	 /** Attempt to pick up a puck. Returns true if one was picked up,
			 else false. */
	 bool Pickup(); 
	 
	 /** Attempt to drop a puck. Returns true if one was dropped, else
			 false. */
	 bool Drop();
	 
	 /** Returns true if we are currently holding a puck. */
	 bool Holding();
	  
	 /** pure virtual - subclasses must implement this method  */
	 virtual void Controller() = 0;

	private:
	 Puck* puck_held;
	 
	 // move the robot
	 void UpdatePose();
	 
	 // update
	 void UpdateSensors();
  };	


} 






class client {

	Home* home;
	vector<Robot*>	robots;
	
	Robot::Robot(const Pose& pose )
	   : 	pose(pose),
		speed(),
		see_robots(),
		see_pucks(),
		puck_held(NULL)
	{
	  // add myself to the static vector of all robots
	  population.push_back( this );
	  
	  if( ! first )
		 first = this;
	}

	Robot::removeRobot();
	Robot::addRobot()

}
