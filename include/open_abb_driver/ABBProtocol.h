#ifndef ABB_PROTOCOL_H_
#define ABB_PROTOCOL_H_

#include <iostream>
#include <string>
#include "math.h"
#include <cstdio>
#include <array>

namespace open_abb_driver
{

/*! \brief ABB server interpreter.
 * Collection of methods to format and parse messages between PC and server 
 * running in ABB controller.  */
class ABBProtocol
{
public:
	
	/*! \brief Formats message to ping the ABB robot.
	* @return String to be sent to ABB server. */
	static std::string PingRobot();
	
	/*! \brief Formats message to set the cartesian coordinates of the ABB robot.
	* The coordinates are always with respect to the currently defined work object and tool.
	* @param x X-coordinate of the robot.
	* @param y Y-coordinate of the robot.
	* @param z Z-coordinate of the robot.
	* @param q0 First component of the orientation quaternion.
	* @param qx Second component of the orientation quaternion.
	* @param qy Third component of the orientation quaternion.
	* @param qz Fourth component of the orientation quaternion.
	* @param linear Whether to use linear tool movement
	* @return String to be sent to ABB server. */
	static std::string SetCartesian( double x, double y, double z, double q0, 
									 double qx, double qy, double qz, bool linear = false );
	
	/*! \brief Formats message to set the joint coordinates of the ABB robot.
	* @param joint1 Value of joint 1.
	* @param joint2 Value of joint 2.
	* @param joint3 Value of joint 3.
	* @param joint4 Value of joint 4.
	* @param joint5 Value of joint 5.
	* @param joint6 Value of joint 6.
	* @return String to be sent to ABB server.*/
	static std::string SetJoints( double joint1, double joint2, double joint3, 
								  double joint4, double joint5, double joint6 );
	
	/*! \brief Formats message to query the ABB robot for its cartesian coordinates.
	* @return String to be sent to ABB server. */
	static std::string GetCartesian();
	
	/*! \brief Formats message to query the ABB robot for its joint axis coordinates.
	* @return String to be sent to ABB server. */
	static std::string GetJoints();
	
	/*! \brief Formats message to define the tool coordinates.
	* @param x X-coordinate of the tool.
	* @param y Y-coordinate of the tool.
	* @param z Z-coordinate of the tool.
	* @param q0 First component of the orientation quaternion of the tool.
	* @param qx Second component of the orientation quaternion of the tool.
	* @param qy Third component of the orientation quaternion of the tool.
	* @param qz Fourth component of the orientation quaternion of the tool.
	* @return String to be sent to ABB server. */
	static std::string SetTool(double x, double y, double z, double q0, double qx, 
							   double qy, double qz);
	
	/*! \brief Formats message to define the coordinates of the work object reference frame.
	* @param x X-coordinate of the work object reference frame.
	* @param y Y-coordinate of the work object reference frame.
	* @param z Z-coordinate of the work object reference frame.
	* @param q0 First component of the orientation quaternion of the work object reference frame.
	* @param qx Second component of the orientation quaternion of the work object reference frame.
	* @param qy Third component of the orientation quaternion of the work object reference frame.
	* @param qz Fourth component of the orientation quaternion of the work object reference frame.
	* @return String to be sent to ABB server.	*/
	static std::string SetWorkObject(double x, double y, double z, double q0, 
									 double qx, double qy, double qz);
	
	/*! \brief Formats message to set the speed of the ABB robot.
  * The values specified for tcp and ori are modified by the percentage of override 
  * specified by the operator in the teach pendant. 
  * @param tcp Linear speed of the TCP in mm/s (max recommended value ~ 500).
  * @param ori Reorientation speed of the TCP in deg/s (max recommended value ~ 150).
  * @return String to be sent to ABB server. */
	static std::string SetSpeed(double tcp, double ori);
	
	/*! \brief Formats message to set the tracking mode (zone) for the robot. 
	 @param fine Whether or not to use fine positioning mode.
	 @param tcp_mm The tool coordinate position tolerance in millimeters.
	 @param ori_mm The orientation position tolerance in millimeters.
	 @param ori_deg The orientation angular tolerance in millimeters. */
	static std::string SetZone(bool fine=0, double tcp_mm = 5.0, double ori_mm = 5.0, 
							   double ori_deg = 1.0);
	
	/*! \brief Sets the servo softness for each axis. 0 is full stiffness, 100 is full softness. */
	static std::string SetSoftness( double s1, double s2, double s3, double s4, double s5, double s6 );
	
	/*! \brief Formats message to add a cartesian coordinate waypoint to the buffer.
	* The coordinates are always with respect to the currently defined work object and tool.
	* @param x X-coordinate of the robot.
	* @param y Y-coordinate of the robot.
	* @param z Z-coordinate of the robot.
	* @param q0 First component of the orientation quaternion.
	* @param qx Second component of the orientation quaternion.
	* @param qy Third component of the orientation quaternion.
	* @param qz Fourth component of the orientation quaternion.
	* @return String to be sent to ABB server. */
	static std::string AddCartesianWaypoint( double x, double y, double z, double q0, 
											 double qx, double qy, double qz );
	
	/*! \brief Formats message to clear all cartesian waypoints in the buffer. */
	static std::string ClearWaypointBuffer();
	
	/*! \brief Formats message to request cartesian waypoint buffer size. */
	static std::string GetBufferSize();
	
	/*! \brief Formats message to begin executing the waypoint buffer. */
	static std::string ExecuteBuffer();
	
	/*! \brief Formats message to set circular move center point. */
	static std::string SetCircularCenter( double x, double y, double z, double qw,
										  double qx, double qy, double qz );
	
	/*! \brief Formats message to set and execute circular move target point. */
	static std::string SetCircularTarget( double x, double y, double z, double qw,
										  double qx, double qy, double qz );
	
	static std::string SetCircularTarget();
	
	/*! \brief Formats message to request the robot serial, robotware version, and type. */
	static std::string GetRobotInfo();
	
	/*! \brief Formats message to close the connection with the server in the ABB robot.
	* @param idCode User code identifying the message. Will be sent back with the acknowledgement.
	* @return String to be sent to ABB server. */
	static std::string CloseConnection();
	
	/*! \brief Parser for the answer from the controller to the command getCartesian().
	* @param msg String message to parse.
	* @param x Placer for the X-coordinate of the ABB robot.
	* @param y Placer for the Y-coordinate of the ABB robot.
	* @param z Placer for the Z-coordinate of the ABB robot.
	* @param q0 Placer for the first component of the orientation quaternion of the ABB robot.
	* @param qx Placer for the second component of the orientation quaternion of the ABB robot.
	* @param qy Placer for the third component of the orientation quaternion of the ABB robot.
	* @param qz Placer for the fourth component of the orientation quaternion of the ABB robot.
	* @return Whether the message was received correctly or not by the ABB robot. */
	static int ParseCartesian( const std::string& msg, double* x, double* y, double* z,
							   double* q0, double* qx, double* qy, double* qz );
	
	/*! \brief Parser for the answer from the controller to the command getJoints().
	* @param msg String message to parse.
	* @param joint1 Placer for the joint 1 of the ABB robot.
	* @param joint2 Placer for the joint 2 of the ABB robot.
	* @param joint3 Placer for the joint 3 of the ABB robot.
	* @param joint4 Placer for the joint 4 of the ABB robot.
	* @param joint5 Placer for the joint 5 of the ABB robot.
	* @param joint6 Placer for the joint 6 of the ABB robot.
	* @return Whether the message was received correctly or not by the ABB robot. */
	static int ParseJoints( const std::string& msg, std::array<double,6>& position );  
};
	
}

#endif
