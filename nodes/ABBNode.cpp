#include "open_abb_driver/ABBNode.h"

namespace open_abb_driver
{
	
	RobotController::RobotController( const ros::NodeHandle& nh, const ros::NodeHandle& ph ) 
	: nodeHandle( nh ), privHandle( ph ) 
	{
		Initialize();
		
		handle_CartesianLog = privHandle.advertise<geometry_msgs::PoseStamped>("pose", 100);
		handle_JointsLog = privHandle.advertise<sensor_msgs::JointState>("jointstate", 100);
		handle_ForceLog = privHandle.advertise<geometry_msgs::WrenchStamped>("wrench", 100);
	
		handle_Ping = privHandle.advertiseService("Ping", &RobotController::robot_Ping, this);
		handle_SetCartesian = privHandle.advertiseService("SetCartesian", &RobotController::robot_SetCartesian, this);
		handle_GetCartesian = privHandle.advertiseService("GetCartesian", &RobotController::robot_GetCartesian, this);
		handle_SetJoints = privHandle.advertiseService("SetJoints", &RobotController::robot_SetJoints, this);
		handle_GetJoints = privHandle.advertiseService("GetJoints", &RobotController::robot_GetJoints, this);
		handle_SetTool = privHandle.advertiseService("SetTool", &RobotController::robot_SetTool, this);
		handle_SetWorkObject = privHandle.advertiseService("SetWorkObject", &RobotController::robot_SetWorkObject, this);
		handle_SetSpeed = privHandle.advertiseService("SetSpeed", &RobotController::robot_SetSpeed, this);
		handle_SetZone = privHandle.advertiseService("SetZone", &RobotController::robot_SetZone, this);
		handle_SpecialCommand = privHandle.advertiseService("SpecialCommand", &RobotController::robot_SpecialCommand, this);
		handle_SetDIO = privHandle.advertiseService("SetDIO", &RobotController::robot_SetDIO, this);
	}
	
	RobotController::~RobotController() 
	{}
	
	bool RobotController::Initialize()
	{
		std::string robotIp;
		int robotMotionPort;
		int robotLoggerPort;
		
		ROS_INFO( "Connecting to the ABB motion server..." );
		privHandle.param<std::string>( "robot_ip", robotIp, "192.168.125.1" );
		privHandle.param( "robot_motion_port", robotMotionPort, 5000 );
		controlInterface = std::make_shared<ABBControlInterface>( robotIp, robotMotionPort );
		
		ROS_INFO( "Connecting to the ABB logger server..." );
		privHandle.param( "robot_logger_port", robotLoggerPort, 5001 );
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

	bool RobotController::SetWorkObject( double x, double y, double z, double q0, 
										 double qx, double qy, double qz )
	{
		if( !controlInterface->SetWorkObject( x, y, z, q0, qx, qy, qz ) ) { return false; }
		
		curWobjTransform.setOrigin(tf::Vector3(x, y, z));
		curWobjTransform.setRotation(tf::Quaternion(qx, qy, qz, q0));
		return true;
	}
	
	bool RobotController::ConfigureRobot()
	{
		double defWOx,defWOy,defWOz,defWOq0,defWOqx,defWOqy,defWOqz;
		double defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz;
		int zone;
		double speedTCP, speedORI;
		
		//WorkObject
		privHandle.param("workobject_x",defWOx,0.0);
		privHandle.param("workobject_y",defWOy,0.0);
		privHandle.param("workobject_z",defWOz,0.0);
		privHandle.param("workobject_q0",defWOq0,1.0);
		privHandle.param("workobject_qx",defWOqx,0.0);
		privHandle.param("workobject_qy",defWOqy,0.0);
		privHandle.param("workobject_qz",defWOqz,0.0);
		
		ROS_INFO( "Setting default work object..." );
		if (SetWorkObject(defWOx,defWOy,defWOz,defWOq0,defWOqx,defWOqy,defWOqz))
			return false;
		// TODO

		
		//Tool
		privHandle.param("tool_x",defTx,0.0);
		privHandle.param("tool_y",defTy,0.0);
		privHandle.param("tool_z",defTz,0.0);
		privHandle.param("tool_q0",defTq0,1.0);
		privHandle.param("tool_qx",defTqx,0.0);
		privHandle.param("tool_qy",defTqy,0.0);
		privHandle.param("tool_qz",defTqz,0.0);
		
		ROS_INFO( "Setting default tool..." );
		if (!controlInterface->SetTool(defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz))
			return false;
		
		//Zone
		privHandle.param("zone",zone,1);
		if( !controlInterface->SetZone(zone) )
			return false;
		
		//Speed
		privHandle.param("speedTCP",speedTCP,250.0);
		privHandle.param("speedORI",speedORI,250.0);
		if (!controlInterface->SetSpeed(speedTCP, speedORI))
			return false;
		
		return true;
	}
	
	bool RobotController::robot_Ping(open_abb_driver::robot_Ping::Request& req, 
									 open_abb_driver::robot_Ping::Response& res)
	{
		return controlInterface->Ping();
	}
	
	bool RobotController::robot_SetCartesian(
		open_abb_driver::robot_SetCartesian::Request& req, 
		open_abb_driver::robot_SetCartesian::Response& res)
	{
		return controlInterface->SetCartesian(req.cartesian[0], req.cartesian[1], req.cartesian[2],
			req.quaternion[0], req.quaternion[1], req.quaternion[2], req.quaternion[3]);
	}
	
	bool RobotController::robot_GetCartesian(
		open_abb_driver::robot_GetCartesian::Request& req, 
		open_abb_driver::robot_GetCartesian::Response& res)
	{
		return controlInterface->GetCartesian(res.x, res.y, res.z, res.q0, res.qx, res.qy, res.qz);
	}
	
	bool RobotController::robot_SetJoints(
		open_abb_driver::robot_SetJoints::Request& req, 
		open_abb_driver::robot_SetJoints::Response& res)
	{	
		// ROS currently uses boost::array, so we have to copy it to maintain compatibility
		std::array<double,6> position;
		std::copy( req.position.begin(), req.position.end(), position.begin() );
		return controlInterface->SetJoints( position );
	}
	
	bool RobotController::robot_GetJoints(
		open_abb_driver::robot_GetJoints::Request& req, 
		open_abb_driver::robot_GetJoints::Response& res)
	{
		std::array<double,6> position;
		if( controlInterface->GetJoints(position) )
		{
			std::copy( position.begin(), position.end(), res.joints.begin() );
			return true;
		}
		else { return false; }
	}
	
	bool RobotController::robot_SetTool(
		open_abb_driver::robot_SetTool::Request& req, 
		open_abb_driver::robot_SetTool::Response& res)
	{
		return controlInterface->SetTool(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz);
	}
	
	bool RobotController::robot_SetWorkObject(
		open_abb_driver::robot_SetWorkObject::Request& req, 
		open_abb_driver::robot_SetWorkObject::Response& res)
	{
		return controlInterface->SetWorkObject(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz);
	}
	
	bool RobotController::robot_SetSpeed(
		open_abb_driver::robot_SetSpeed::Request& req, 
		open_abb_driver::robot_SetSpeed::Response& res)
	{
		return controlInterface->SetSpeed(req.tcp, req.ori);
	}
	
	bool RobotController::robot_SetZone(
		open_abb_driver::robot_SetZone::Request& req, 
		open_abb_driver::robot_SetZone::Response& res)
	{
		return controlInterface->SetZone(req.mode);
	}
	
	bool RobotController::robot_SpecialCommand(
		open_abb_driver::robot_SpecialCommand::Request& req, 
		open_abb_driver::robot_SpecialCommand::Response& res)
	{
		return controlInterface->SpecialCommand( req.command, req.param1, req.param2, 
												 req.param3, req.param4, req.param5 );
	}
	
	// Lock/Unlock tool changer
	bool RobotController::robot_SetDIO(
		open_abb_driver::robot_SetDIO::Request& req, 
		open_abb_driver::robot_SetDIO::Response& res)
	{
		return controlInterface->SetDIO(req.DIO_num, req.state);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	open_abb_driver::RobotController ABBrobot( nh, ph );
	
	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();
	
	return 0;
}
