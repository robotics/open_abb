#include "open_abb_driver/ABBNode.h"

namespace open_abb_driver
{
	
	RobotController::RobotController( const ros::NodeHandle& nh, const ros::NodeHandle& ph ) 
	: nodeHandle( nh ), privHandle( ph ) 
	{
		Initialize();
		
		handle_CartesianLog = privHandle.advertise<geometry_msgs::PoseStamped>("pose", 100);
		handle_JointsLog = privHandle.advertise<sensor_msgs::JointState>("joint_state", 100);
	
		handle_Ping = privHandle.advertiseService("ping", &RobotController::Ping, this);
		handle_SetCartesian = privHandle.advertiseService("set_cartesian", &RobotController::SetCartesian, this);
		handle_GetCartesian = privHandle.advertiseService("get_cartesian", &RobotController::GetCartesian, this);
		handle_SetJoints = privHandle.advertiseService("set_joints", &RobotController::SetJoints, this);
		handle_GetJoints = privHandle.advertiseService("get_joints", &RobotController::GetJoints, this);
		handle_SetTool = privHandle.advertiseService("set_tool", &RobotController::SetTool, this);
		handle_SetWorkObject = privHandle.advertiseService("set_work_object", &RobotController::SetWorkObject, this);
		handle_SetSpeed = privHandle.advertiseService("set_speed", &RobotController::SetSpeed, this);
		handle_SetZone = privHandle.advertiseService("set_zone", &RobotController::SetZone, this);
		handle_SetDIO = privHandle.advertiseService("set_DIO", &RobotController::SetDIO, this);
		handle_UnwindAxes = privHandle.advertiseService("unwind_axes", &RobotController::UnwindAxes, this);

		
		feedbackWorker = boost::thread( boost::bind( &RobotController::FeedbackSpin, this ) );
	}
	
	RobotController::~RobotController() 
	{
		feedbackWorker.join();
	}
	
	bool RobotController::Initialize()
	{
		std::string robotIp;
		int robotMotionPort;
		int robotLoggerPort;
		
		ROS_INFO( "Connecting to the ABB motion server..." );
		privHandle.param<std::string>( "ip", robotIp, "192.168.125.1" );
		privHandle.param( "motion_port", robotMotionPort, 5000 );
		controlInterface = std::make_shared<ABBControlInterface>( robotIp, robotMotionPort );
		
		ROS_INFO( "Connecting to the ABB logger server..." );
		privHandle.param( "logger_port", robotLoggerPort, 5001 );
		feedbackInterface = std::make_shared<ABBFeedbackInterface>( robotIp, robotLoggerPort );
		
		// Allocate space for all of our vectors
		curToolP = matvec::Vec(3);
		curWorkP = matvec::Vec(3);
		curP = matvec::Vec(3);
		curGoalP = matvec::Vec(3);
		curTargP = matvec::Vec(3);
		
		ROS_INFO("Setting robot default configuration...");
		if( !ConfigureRobot() )
		{
			ROS_WARN("Not able to set the robot to default configuration.");
			return false;
		}
		
		return true;
	}
	
	void RobotController::FeedbackSpin()
	{
		FeedbackVisitor visitor( handle_JointsLog, handle_CartesianLog );
		while( ros::ok() )
		{
			feedbackInterface->Spin();
			while( feedbackInterface->HasFeedback() )
			{
				Feedback fb = feedbackInterface->GetFeedback();
				boost::apply_visitor( visitor, fb );
			}
		}
	}

	bool RobotController::SetWorkObject( double x, double y, double z, double qw, 
										 double qx, double qy, double qz )
	{
		if( !controlInterface->SetWorkObject( x, y, z, qw, qx, qy, qz ) ) { return false; }
		
		curWobjTransform.setOrigin(tf::Vector3(x, y, z));
		curWobjTransform.setRotation(tf::Quaternion(qx, qy, qz, qw));
		return true;
	}
	
	bool RobotController::ConfigureRobot()
	{
		double defWOx,defWOy,defWOz,defWOqw,defWOqx,defWOqy,defWOqz;
		double defTx,defTy,defTz,defTqw,defTqx,defTqy,defTqz;
		int zone;
		double speedTCP, speedORI;
		
		//WorkObject
		privHandle.param("workobject_x",defWOx,0.0);
		privHandle.param("workobject_y",defWOy,0.0);
		privHandle.param("workobject_z",defWOz,0.0);
		privHandle.param("workobject_qw",defWOqw,1.0);
		privHandle.param("workobject_qx",defWOqx,0.0);
		privHandle.param("workobject_qy",defWOqy,0.0);
		privHandle.param("workobject_qz",defWOqz,0.0);
		
		if( !SetWorkObject(defWOx,defWOy,defWOz,defWOqw,defWOqx,defWOqy,defWOqz))
		{
			ROS_WARN( "Unable to set the work object." );
			return false;
		}
		// TODO

		
		//Tool
		privHandle.param("tool_x",defTx,0.0);
		privHandle.param("tool_y",defTy,0.0);
		privHandle.param("tool_z",defTz,0.0);
		privHandle.param("tool_qw",defTqw,1.0);
		privHandle.param("tool_qx",defTqx,0.0);
		privHandle.param("tool_qy",defTqy,0.0);
		privHandle.param("tool_qz",defTqz,0.0);
		
		if( !controlInterface->SetTool(defTx,defTy,defTz,defTqw,defTqx,defTqy,defTqz))
		{
			ROS_WARN( "Unable to set the tool." );
			return false;
		}
		
		//Zone
		privHandle.param("zone",zone,1);
		if( !controlInterface->SetZone(zone) )
		{
			ROS_WARN( "Unable to set the tracking zone." );
			return false;
		}
		
		//Speed
		privHandle.param("speed_tcp",speedTCP,0.250);
		privHandle.param("speed_ori",speedORI,0.250);
		if( !controlInterface->SetSpeed(speedTCP, speedORI) )
		{
			ROS_WARN( "Unable to set the speed." );
			return false;
		}
		
		return true;
	}
	
	bool RobotController::Ping( Ping::Request& req, Ping::Response& res )
	{
		return controlInterface->Ping();
	}
	
	bool RobotController::SetCartesian( SetCartesian::Request& req, SetCartesian::Response& res )
	{
		return controlInterface->SetCartesian(req.x, req.y, req.z,
			req.qw, req.qx, req.qy, req.qz);
	}
	
	bool RobotController::GetCartesian( GetCartesian::Request& req, GetCartesian::Response& res )
	{
		return controlInterface->GetCartesian(res.x, res.y, res.z, res.qw, res.qx, res.qy, res.qz);
	}
	
	bool RobotController::SetJoints( SetJoints::Request& req, SetJoints::Response& res )
	{	
		// ROS currently uses boost::array, so we have to copy it to maintain compatibility
		std::array<double,6> position;
		std::copy( req.position.begin(), req.position.end(), position.begin() );
		return controlInterface->SetJoints( position );
	}
	
	bool RobotController::GetJoints( GetJoints::Request& req, GetJoints::Response& res )
	{
		std::array<double,6> position;
		if( controlInterface->GetJoints(position) )
		{
			std::copy( position.begin(), position.end(), res.joints.begin() );
			return true;
		}
		else { return false; }
	}
	
	bool RobotController::SetTool( SetTool::Request& req, SetTool::Response& res )
	{
		return controlInterface->SetTool(req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz);
	}
	
	bool RobotController::SetWorkObject( SetWorkObject::Request& req, SetWorkObject::Response& res )
	{
		return controlInterface->SetWorkObject(req.x, req.y, req.z, req.qw, req.qx, req.qy, req.qz);
	}
	
	bool RobotController::SetSpeed( SetSpeed::Request& req, SetSpeed::Response& res )
	{
		return controlInterface->SetSpeed(req.tcp, req.ori);
	}
	
	bool RobotController::SetZone( SetZone::Request& req, SetZone::Response& res )
	{
		return controlInterface->SetZone(req.mode);
	}
	
	bool RobotController::SetDIO( SetDIO::Request& req, SetDIO::Response& res )
	{
		return controlInterface->SetDIO(req.DIO_num, req.state);
	}
	
	bool RobotController::UnwindAxes( UnwindAxes::Request& req, UnwindAxes::Response& res )
	{
		std::array<double,6> thresholds;
		std::copy( req.thresholds.begin(), req.thresholds.end(), thresholds.begin() );
		return controlInterface->UnwindAxes( thresholds );
	}

	
	FeedbackVisitor::FeedbackVisitor( ros::Publisher& handle_Joints,
									  ros::Publisher& handle_Cartesian )
		: jointPub( handle_Joints ), cartesianPub( handle_Cartesian )
	{}
	
	void FeedbackVisitor::operator()( const JointFeedback& fb )
	{
		sensor_msgs::JointState jointMsg;
		jointMsg.header.stamp = ros::Time::now(); // TODO Use abb timestamp
		for( unsigned int i = 0; i < 6; i++ )
		{
			jointMsg.name.push_back( std::to_string( i+1 ) );
			jointMsg.position.push_back( fb.joints[i] );
		}
		jointPub.publish( jointMsg );
	}
	
	void FeedbackVisitor::operator()( const CartesianFeedback& fb )
	{
		geometry_msgs::PoseStamped poseMsg;
		poseMsg.header.stamp = ros::Time::now();
		poseMsg.pose.position.x = fb.x;
		poseMsg.pose.position.y = fb.y;
		poseMsg.pose.position.z = fb.z;
		poseMsg.pose.orientation.w = fb.qw;
		poseMsg.pose.orientation.x = fb.qx;
		poseMsg.pose.orientation.y = fb.qy;
		poseMsg.pose.orientation.z = fb.qz;
		cartesianPub.publish( poseMsg );
	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	open_abb_driver::RobotController ABBrobot( nh, ph );
	
	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();
	
	return 0;
}
