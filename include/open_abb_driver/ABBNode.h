#include "open_abb_driver/ABBControlInterface.h"
#include "open_abb_driver/ABBFeedbackInterface.h"
#include "open_abb_driver/PoseSE3.h"
#include "open_abb_driver/ABBKinematics.h"

//ROS specific
#include <ros/ros.h>

#include <open_abb_driver/Ping.h>
#include <open_abb_driver/SetCartesian.h>
#include <open_abb_driver/GetCartesian.h>
#include <open_abb_driver/SetWorkObject.h>
#include <open_abb_driver/SetZone.h>
#include <open_abb_driver/SetSoftness.h>
#include <open_abb_driver/SetTool.h>
#include <open_abb_driver/SetJoints.h>
#include <open_abb_driver/GetJoints.h>
#include <open_abb_driver/SetSpeed.h>

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

#include <Eigen/Geometry>

namespace open_abb_driver
{
	
	class RobotController
	{
	public:
		RobotController( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~RobotController();
		
		// Service Callbacks
		bool PingCallback( Ping::Request& req, Ping::Response& res );
		bool SetCartesianCallback( SetCartesian::Request& req, SetCartesian::Response& res );
		bool GetCartesianCallback( GetCartesian::Request& req, GetCartesian::Response& res );
		bool SetJointsCallback( SetJoints::Request& req, SetJoints::Response& res );
		bool GetJointsCallback( GetJoints::Request& req, GetJoints::Response& res );
		bool SetToolCallback( SetTool::Request& req, SetTool::Response& res );
		bool SetWorkObjectCallback( SetWorkObject::Request& req, SetWorkObject::Response& res );
		bool SetSpeedCallback( SetSpeed::Request& req, SetSpeed::Response& res );
		bool SetZoneCallback( SetZone::Request& req, SetZone::Response& res );
		bool SetSoftnessCallback( SetSoftness::Request& req, SetSoftness::Response& res );
		
		bool Ping();
		bool SetCartesian( const PoseSE3& pose );
		bool GetCartesian( PoseSE3& pose );
		bool SetJoints( const JointAngles& angles );
		bool GetJoints( JointAngles& angles );
		bool SetTool( const PoseSE3& pose );
		bool SetWorkObject( const PoseSE3& pose );
		bool SetSpeed( double linear, double orientation );
		bool SetZone( unsigned int zone );
		bool SetSoftness( const std::array<double,6>& softness );
		
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
		ABBKinematics ikSolver;
		
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
		ros::ServiceServer handle_SetSoftness;
		ros::ServiceServer handle_SpecialCommand;
		
		// Functions to handle setting up non-blocking step sizes
		bool setTrackDist(double pos_dist, double ang_dist);
		bool setNonBlockSpeed(double tcp, double ori);
		
		// Robot State
		PoseSE3 currToolTrans;
		PoseSE3 currWorkTrans;
		
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
