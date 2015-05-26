#ifndef ABBINTERPRETER_H
#define ABBINTERPRETER_H

#include <iostream>
#include <string>
#include "math.h"
#include <cstdio>

/** \class namespace
    \brief ABB server interpreter.
    Collection of methods to format and parse messages between PC and server running in ABB controller.   
*/
namespace open_abb_driver
{
	
class ABBComm
{
public:
	
	static std::string pingRobot(int idCode=0);
	static std::string setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode=0);
	static std::string setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode=0);
	static std::string getCartesian(int idCode=0);
	static std::string getJoints(int idCode=0);
	static std::string setTool(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode=0);
	static std::string setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode=0);
	static std::string setSpeed(double tcp, double ori, int idCode=0);
	static std::string setZone(bool fine=0, double tcp_mm = 5.0, double ori_mm = 5.0, double ori_deg = 1.0, int idCode=0);
	static std::string specialCommand(int command, double param1, double param2, double param3, double param4, double param5, int idCode=0);
	static std::string setVacuum(int vacuum=0, int idCode=0);
	static std::string setDIO(int dio_number=0, int dio_state=0, int idCode=0);
	static std::string closeConnection(int idCode=0);
	static int parseCartesian( std::string msg, double *x, double *y, double *z,
							   double *q0, double *qx, double *qy, double *qz );
	static int parseJoints( std::string msg, double *joint1, double *joint2, 
							double *joint3, double *joint4, double *joint5, double *joint6 );  
};
	
}

#endif
