#include "oad.h"

/**
  * Formats message to ping the ABB robot.
  * @return string to send to the ABB server.
  */
std::string oad::pingRobot()
{
  std::string msg("00 ");//instruction code;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the cartesian coordinates of the ABB robot.
  * The coordinates are always with respect to the currently defined work object and tool.
  * @param x X-coordinate of the robot.
  * @param y Y-coordinate of the robot.
  * @param z Z-coordinate of the robot.
  * @param q0 First component of the orientation quaternion.
  * @param qx Second component of the orientation quaternion.
  * @param qy Third component of the orientation quaternion.
  * @param qz Fourth component of the orientation quaternion.
  * @return String to be sent to ABB server.
  */
std::string oad::setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz)
{
  char buff[10];
  std::string msg("01 ");//instruction code;
  
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the joint coordinates of the ABB robot.
  * @param joint1 Value of joint 1.
  * @param joint2 Value of joint 2.
  * @param joint3 Value of joint 3.
  * @param joint4 Value of joint 4.
  * @param joint5 Value of joint 5.
  * @param joint6 Value of joint 6.
  * @return String to be sent to ABB server.
  */
std::string oad::setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6)
{
  char buff[10];
  std::string msg("02 ");//instruction code;
  
  sprintf(buff,"%+08.2lf ",joint1);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint2);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint3);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint4);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint5);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint6);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to query the ABB robot for its cartesian coordinates.
  * @return String to be sent to ABB server.
  */
std::string oad::getCartesian()
{
  std::string msg("04 ");//instruction code;

  msg += "#";
  return (msg);
}

/**
  * Formats message to query the ABB robot for its joint axis coordinates.
  * @return String to be sent to ABB server.
  */
std::string oad::getJoints()
{
  std::string msg("05 ");//instruction code;
  msg += "#";

  return (msg);
}

/**
  * Formats message to define the tool coordinates.
  * @param x X-coordinate of the tool.
  * @param y Y-coordinate of the tool.
  * @param z Z-coordinate of the tool.
  * @param q0 First component of the orientation quaternion of the tool.
  * @param qx Second component of the orientation quaternion of the tool.
  * @param qy Third component of the orientation quaternion of the tool.
  * @param qz Fourth component of the orientation quaternion of the tool.
  * @return String to be sent to ABB server.
  */
std::string oad::setTool(double x, double y, double z, double q0, double qx, double qy, double qz)
{
  char buff[10];
  std::string msg("06 ");//instruction code;
  
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to define the coordinates of the work object reference frame.
  * @param x X-coordinate of the work object reference frame.
  * @param y Y-coordinate of the work object reference frame.
  * @param z Z-coordinate of the work object reference frame.
  * @param q0 First component of the orientation quaternion of the work object reference frame.
  * @param qx Second component of the orientation quaternion of the work object reference frame.
  * @param qy Third component of the orientation quaternion of the work object reference frame.
  * @param qz Fourth component of the orientation quaternion of the work object reference frame.
  * @return String to be sent to ABB server.
  */
std::string oad::setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz)
{
  char buff[10];
  std::string msg("07 ");//instruction code;
  
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the speed of the ABB robot.
  * The values specified for tcp and ori are modified by the percentage of override specified by the operator in the teach pendant. 
  * @param tcp  Linear speed of the TCP in mm/s (max recommended value ~ 500).
  * @param ori  Reorientation speed of the TCP in deg/s (max recommended value ~ 150).
  * @param leax Linear speed of external axis (default value = 0 mm/sec).
  * @param reax Angular speed of external axis (default value = 0 deg/sec).
  * @return String to be sent to ABB server.
  */
std::string oad::setSpeed(double tcp, double ori, double leax, double reax)
{
  char buff[10];
  std::string msg("08 ");//instruction code;
  
  sprintf(buff,"%08.1lf ",tcp);
  msg += buff ;
  sprintf(buff,"%08.2lf ",ori);
  msg += buff ;
  sprintf(buff,"%08.1lf ",leax);
  msg += buff ;
  sprintf(buff,"%08.2lf ",reax);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the zone mode of the ABB robot (distance from where to interpolate to the next destination).
  * @param fine   Specify motion mode: 1 for stop point - 0 for fly by point.
  * @param tcp_mm Linear distance from target point to begin to interpolate the position of the TCP (recommended = 5.0mm)
  * @param ori_mm Linear distance from the target point to begin to interpolate the orientation of the TCP (recommended = 5.0mm)
  * @param ori_deg Angular distance from the target point to begin to interpolate the orientation of the TCP (recommended = 1.0deg)
  * @return String to be sent to ABB server.
  */
std::string oad::setZone(bool fine, double tcp_mm, double ori_mm, double ori_deg)
{
 char buff[10];
  std::string msg("09 ");//instruction code;
  
  sprintf(buff,"%.1d ",fine);
  msg += buff ;
  sprintf(buff,"%.2lf ", tcp_mm);
  msg += buff ;
  sprintf(buff,"%.2lf ", ori_mm);
  msg += buff ;
  sprintf(buff,"%.2lf ", ori_deg);
  msg += buff ;
  msg += "#";

  return (msg);
}


/**
  * Formats message to set the zone of the ABB robot to one of a few predefined modes.
  * Modes:
  * ---------------
  * Mode - Name in RAPID - Linear  - Orientation(mm)  -  Orientation (deg)
  * 0          fine         0 mm          0 mm                 0°
  * 1          z0           0.3 mm        0.3 mm               0.03°
  * 2          z1           1 mm          1 mm                 0.1°
  * 3          z5           5 mm          8 mm                 0.8°
  * 4          z10         10 mm          15 mm                1.5°
  * 5          z15         15 mm          23 mm                2.3°
  * 6          z20         20 mm          30 mm                3.0°
  * @param mode Selected mode (0-6).
  * @return String to be sent to ABB server.
  */  
std::string oad::setPredefinedZone(int mode)
{
  switch (mode)
    {
    case 0:
      return(oad::setZone(1, 0.0, 0.0, 0.0));
    case 1:
      return(oad::setZone(0, 0.3, 0.3, 0.03));
    case 2:
      return(oad::setZone(0, 1.0, 1.0, 0.10));
    case 3:
      return(oad::setZone(0, 5.0, 8.0, 0.80));
    case 4:
      return(oad::setZone(0, 10.0, 15.0, 1.50));
    case 5:
      return(oad::setZone(0, 15.0, 23.0, 2.30));
    case 6:
      return(oad::setZone(0, 20.0, 30.0, 3.00));
    default:
      return(oad::setZone(1, 0.0, 0.0, 0.0));
    }
}



/**
  * Formats message to close the connection with the server in the ABB robot.
  * @return String to be sent to ABB server.
  */
std::string oad::closeConnection()
{
  std::string msg("99 ");//instruction code;
  msg += "#";
  return (msg);
}

/**
  * Parser for the answer from the controller to the command getCartesian().
  * @param msg String message to parse.
  * @param x Placer for the X-coordinate of the ABB robot.
  * @param y Placer for the Y-coordinate of the ABB robot.
  * @param z Placer for the Z-coordinate of the ABB robot.
  * @param q0 Placer for the first component of the orientation quaternion of the ABB robot.
  * @param qx Placer for the second component of the orientation quaternion of the ABB robot.
  * @param qy Placer for the third component of the orientation quaternion of the ABB robot.
  * @param qz Placer for the fourth component of the orientation quaternion of the ABB robot.
  * @return Whether the message was received correctly or not by the ABB robot.
  */
int oad::parseCartesian(std::string msg, double *x, double *y, 
    double *z,double *q0, double *qx, double *qy, double*qz)
{
  int ok;
  sscanf(msg.c_str(),"%*d %d %lf %lf %lf %lf %lf %lf %lf",&ok,x,y,z,q0,qx,qy,qz);
  return ok;
}

/**
  * Parser for the answer from the controller to the command getJoints().
  * @param msg String message to parse.
  * @param joint1 Placer for the joint 1 of the ABB robot.
  * @param joint2 Placer for the joint 2 of the ABB robot.
  * @param joint3 Placer for the joint 3 of the ABB robot.
  * @param joint4 Placer for the joint 4 of the ABB robot.
  * @param joint5 Placer for the joint 5 of the ABB robot.
  * @param joint6 Placer for the joint 6 of the ABB robot.
  * @return Whether the message was received correctly or not by the ABB robot.
  */
int oad::parseJoints(std::string msg,  double *joint1, 
    double *joint2, double *joint3, double *joint4, 
    double *joint5, double *joint6)
{
  int ok;
  sscanf(msg.c_str(),"%*d %d %lf %lf %lf %lf %lf %lf",&ok,joint1,joint2,joint3,joint4,joint5,joint6);
  return ok;
}

/**
  * Check correct execution.
  * @param msg String message to parse.
  * @return Whether the message was received correctly or not by the ABB robot.
  */
int oad::checkReply(std::string msg)
{
  int ok;
  sscanf(msg.c_str(),"%*d %d",&ok);
  return ok;
}
