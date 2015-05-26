#include "open_abb_driver/abb_comm.h"

using namespace std;

namespace open_abb_driver
{

/**
  * Formats message to ping the ABB robot.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::pingRobot(int idCode)
{
  char buff[10];
  string msg("00 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[10];
  string msg("01 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode)
{
  char buff[10];
  string msg("02 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::getCartesian(int idCode)
{
  char buff[10];
  string msg("03 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";
  return (msg);
}

/**
  * Formats message to query the ABB robot for its joint axis coordinates.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::getJoints(int idCode)
{
  char buff[10];
  string msg("04 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::setTool(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[10];
  string msg("06 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[10];
  string msg("07 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
  * @param tcp Linear speed of the TCP in mm/s (max recommended value ~ 500).
  * @param ori Reorientation speed of the TCP in deg/s (max recommended value ~ 150).
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::setSpeed(double tcp, double ori, int idCode)
{
  char buff[10];
  string msg("08 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%08.1lf ",tcp);
  msg += buff ;
  sprintf(buff,"%08.2lf ",ori);
  msg += buff ;
  msg += "#";

  return (msg);
}


string ABBComm::setZone(bool fine, double tcp_mm, double ori_mm, double ori_deg, int idCode)
{
 char buff[10];
  string msg("09 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
  * Formats message to call special command.
  * @param command Number identifying the special command.
  * @param param1 General purpose parameter 1.
  * @param param2 General purpose parameter 2.
  * @param param3 General purpose parameter 3.
  * @param param4 General purpose parameter 4.
  * @param param5 General purpose parameter 5.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::specialCommand(int command, double param1, double param2, double param3, double param4, double param5, int idCode)
{
  char buff[12];
  string msg("10 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%.1d ",command);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param1);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param2);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param3);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param4);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param5);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the vacuum on/off.
  * @param vacuum 1-on 0-off.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::setVacuum(int vacuum, int idCode)
{
  char buff[10];
  string msg("11 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%.2d ",vacuum);
  msg += buff ;
  msg += "#";

  return (msg);
}



string ABBComm::setDIO(int dio_number, int dio_state, int idCode)
{
  char buff[10];
  string msg("26 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%.2d ", dio_number);
  msg += buff ;
  sprintf(buff,"%.2d ", dio_state);
  msg += buff ;
  msg += "#";
  return (msg);
}


/**
  * Formats message to close the connection with the server in the ABB robot.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBComm::closeConnection(int idCode)
{
  char buff[10];
  string msg("99 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
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
int ABBComm::parseCartesian(std::string msg, double *x, double *y, 
    double *z,double *q0, double *qx, double *qy, double*qz)
{
  int ok, idCode;
  sscanf(msg.c_str(),"%*d %d %d %*f %lf %lf %lf %lf %lf %lf %lf",&idCode,&ok,x,y,z,q0,qx,qy,qz);
  if (ok)
    return idCode;
  else
    return -1;
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
int ABBComm::parseJoints(std::string msg,  double *joint1, 
    double *joint2, double *joint3, double *joint4, 
    double *joint5, double *joint6)
{
  int ok, idCode;
  sscanf(msg.c_str(),"%*d %d %d %*f %lf %lf %lf %lf %lf %lf",&idCode,&ok,joint1,joint2,joint3,joint4,joint5,joint6);
  if (ok)
    return idCode;
  else
    return -1;
}

}