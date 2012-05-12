#ifndef OAD_H
#define OAD_H

#include <stdio.h>
#include <string>

/** \class namespace
    \brief OAD interpreter for C++.
    OAD collection of methods to format and parse messages between PC and server running in ABB controller.   
*/
namespace oad
{
  std::string pingRobot();
  std::string setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz);
  std::string setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6);
  std::string getCartesian();
  std::string getJoints();
  std::string setTool(double x, double y, double z, double q0, double qx, double qy, double qz);
  std::string setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz);
  std::string setSpeed(double tcp, double ori, double leax=0.0, double reax=0.0);
  std::string setZone(bool fine, double tcp_mm = 5.0, double ori_mm = 5.0, double ori_deg = 1.0);
  std::string setPredefinedZone(int mode);
  std::string closeConnection();
  int checkReply(std::string msg);
  int parseCartesian(std::string msg, double *x, double *y, double *z,
      double *q0, double *qx, double *qy, double *qz);
  int parseJoints(std::string msg, double *joint1, double *joint2, 
      double *joint3, double *joint4, double *joint5, double *joint6);  
}
#endif
