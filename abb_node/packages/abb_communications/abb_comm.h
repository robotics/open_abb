#ifndef ABBINTERPRETER_H
#define ABBINTERPRETER_H

#include <iostream>
#include <string>
#include "math.h"
#include <cstdio>

using namespace std;

/** \class namespace
    \brief ABB server interpreter.
    Collection of methods to format and parse messages between PC and server running in ABB controller.   
*/
namespace abb_comm
{
  string pingRobot(int idCode=0);
  string setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode=0);
  string setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode=0);
  string getCartesian(int idCode=0);
  string getJoints(int idCode=0);
  string setTool(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode=0);
  string setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode=0);
  string setSpeed(double tcp, double ori, int idCode=0);
  string setZone(bool fine=0, double tcp_mm = 5.0, double ori_mm = 5.0, double ori_deg = 1.0, int idCode=0);
  string specialCommand(int command, double param1, double param2, double param3, double param4, double param5, int idCode=0);
  string setVacuum(int vacuum=0, int idCode=0);
  string setDIO(int dio_number=0, int dio_state=0, int idCode=0);
  string closeConnection(int idCode=0);
  int parseCartesian(string msg, double *x, double *y, double *z,
      double *q0, double *qx, double *qy, double *qz);
  int parseJoints(string msg, double *joint1, double *joint2, 
      double *joint3, double *joint4, double *joint5, double *joint6);  
}
#endif
