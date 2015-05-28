#include "open_abb_driver/matvec/matVec.h"
#include "open_abb_driver/ABBControlInterface.h"
#include "open_abb_driver/ABBFeedbackInterface.h"

//ROS specific
#include <ros/ros.h>

#include <open_abb_driver/robot_Ping.h>
#include <open_abb_driver/robot_SetCartesian.h>
#include <open_abb_driver/robot_GetCartesian.h>
#include <open_abb_driver/robot_SetWorkObject.h>
#include <open_abb_driver/robot_SetZone.h>
#include <open_abb_driver/robot_SetTool.h>
#include <open_abb_driver/robot_SetComm.h>
#include <open_abb_driver/robot_SetJoints.h>
#include <open_abb_driver/robot_GetJoints.h>
#include <open_abb_driver/robot_SetSpeed.h>
#include <open_abb_driver/robot_SetDIO.h>
#include <open_abb_driver/robot_SpecialCommand.h>
#include <open_abb_driver/robot_Stop.h>
#include <open_abb_driver/robot_SetTrackDist.h>

//ROS specific, these are redundant with abb_node
//standard libary messages instead of custom messages
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#define ID_CODE_MAX 999

#define SERVER_BAD_MSG 0
#define SERVER_OK 1
#define SERVER_COLLISION 2

#define MAX_TRANS_STEP 2.0
#define MAX_ROT_STEP (0.5 * DEG2RAD)
#define MAX_J_STEP 0.5

#define NB_FREQ 200.0
#define STOP_CHECK_FREQ 25.0
#define DIST_CHECK_FREQ 100.0

#define SAFETY_FACTOR 0.90
#define MINIMUM_TRACK_DIST_TRANS 1.0 //mm
#define MAXIMUM_TRACK_DIST_TRANS 20.0 //mm
#define MINIMUM_TRACK_DIST_ORI 0.333  //deg
#define MAXIMUM_TRACK_DIST_ORI 6.66 //deg
#define INFINITY_TRACK_DIST_TRANS 1000.0 ///mm
#define INFINITY_TRACK_DIST_ORI 333.0 //deg

#define MINIMUM_NB_SPEED_TCP 1.0 //mm/s
#define MINIMUM_NB_SPEED_ORI 0.333 //deg/s

#define NUM_JOINTS 6
#define NUM_FORCES 6

#define BLOCKING 1
#define NON_BLOCKING 0

#include <boost/thread/thread.hpp>

namespace open_abb_driver
{
	
	class RobotController
	{
	public:
		RobotController( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~RobotController();
		
		// Service Callbacks
		bool robot_Ping(
			open_abb_driver::robot_Ping::Request& req, 
			open_abb_driver::robot_Ping::Response& res);
		bool robot_SetCartesian(
			open_abb_driver::robot_SetCartesian::Request& req, 
			open_abb_driver::robot_SetCartesian::Response& res);
		bool robot_GetCartesian(
			open_abb_driver::robot_GetCartesian::Request& req, 
			open_abb_driver::robot_GetCartesian::Response& res);
		bool robot_SetJoints(
			open_abb_driver::robot_SetJoints::Request& req, 
			open_abb_driver::robot_SetJoints::Response& res);
		bool robot_GetJoints(
			open_abb_driver::robot_GetJoints::Request& req, 
			open_abb_driver::robot_GetJoints::Response& res);
		bool robot_SetTool(
			open_abb_driver::robot_SetTool::Request& req, 
			open_abb_driver::robot_SetTool::Response& res);
		bool robot_SetWorkObject(
			open_abb_driver::robot_SetWorkObject::Request& req, 
			open_abb_driver::robot_SetWorkObject::Response& res);
		bool robot_SetDIO(
			open_abb_driver::robot_SetDIO::Request& req, 
			open_abb_driver::robot_SetDIO::Response& res);
		bool robot_SetSpeed(
			open_abb_driver::robot_SetSpeed::Request& req, 
			open_abb_driver::robot_SetSpeed::Response& res);
		bool robot_SetZone(
			open_abb_driver::robot_SetZone::Request& req, 
			open_abb_driver::robot_SetZone::Response& res);
		
		// Call back function for the logging which will be called by a timer event
		void logCallback(const ros::TimerEvent&);
		
		// Non-Blocking move variables
		bool stopRequest;   // Set to true when we are trying to stop the robot
		bool stopConfirm;   // Set to true when the thread is sure it's stopped
		bool cart_move;     // True if we're doing a cartesian move, false if joint
		
		// Variables dealing with changing non-blocking speed and step sizes
		double curCartStep;     // Largest cartesian stepsize during non-blocking
		double curOrientStep;   // Largest orientation step size during non-blocking
		double curJointStep;    // Largest joint step size during non-blocking
		double curDist[3];      // Max allowable tracking error (pos, ang, joint)
		
		// Most recent goal position, and the final target position
		matvec::Vec curGoalP;
		matvec::Quaternion curGoalQ;
		matvec::Vec curTargP;
		matvec::Quaternion curTargQ;
		double curGoalJ[6];
		double curTargJ[6];
		
		// Functions that compute our distance from the current position to the goal
		double posDistFromGoal();
		double orientDistFromGoal();
		double jointDistFromGoal();
		
	private:
		
		typedef boost::shared_mutex Mutex;
		typedef boost::unique_lock< Mutex > WriteLock;
		typedef boost::shared_lock< Mutex > ReadLock;
		
		Mutex nonBlockMutex;
		Mutex jointUpdateMutex;
		Mutex cartUpdateMutex;
		Mutex wobjUpdateMutex;
		Mutex forceUpdateMutex;
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		ABBControlInterface::Ptr controlInterface;
		ABBFeedbackInterface::Ptr feedbackInterface;
		
		// Initialize the robot
		bool Initialize();
		
		// Sets up the default robot configuration
		bool ConfigureRobot();
		
		void FeedbackSpin();
		
		//handles to ROS stuff
		tf::TransformBroadcaster handle_tf;
		//Duplicates with standard messages
		ros::Publisher handle_JointsLog;
		ros::Publisher handle_CartesianLog;
		
		ros::ServiceServer handle_Ping;
		ros::ServiceServer handle_SetCartesian;
		ros::ServiceServer handle_GetCartesian;
		ros::ServiceServer handle_SetJoints;
		ros::ServiceServer handle_GetJoints;
		ros::ServiceServer handle_SetTool;
		ros::ServiceServer handle_SetWorkObject;
		ros::ServiceServer handle_SetSpeed;
		ros::ServiceServer handle_SetZone;
		ros::ServiceServer handle_SpecialCommand;
		ros::ServiceServer handle_SetDIO;
		
		// Functions to handle setting up non-blocking step sizes
		bool setTrackDist(double pos_dist, double ang_dist);
		bool setNonBlockSpeed(double tcp, double ori);
		
		// Robot State
		double curSpd[2];
		int curZone;
		matvec::Vec curToolP;
		matvec::Quaternion curToolQ;
		matvec::Vec curWorkP;
		matvec::Quaternion curWorkQ;
		tf::Transform curWobjTransform;
		
		// Robot Position and Force Information
		matvec::Vec curP;
		matvec::Quaternion curQ;
		double curJ[6];
		double curForce[6];
		
		boost::thread feedbackWorker;
		
		bool SetWorkObject( double x, double y, double z, double q0, double q1,
							double q2, double q3 );
		
	};
	
	/*! \brief Class to process the Feedback variant types */
	class FeedbackVisitor
		: public boost::static_visitor<>
	{
	public:
		
		FeedbackVisitor( ros::Publisher& handle_Joints, 
						 ros::Publisher& handle_Cartesian );
		
		void operator()( const JointFeedback& fb );
		void operator()( const CartesianFeedback& fb );
		
	private:
		
		ros::Publisher& jointPub;
		ros::Publisher& cartesianPub;
		
	};
	
}
