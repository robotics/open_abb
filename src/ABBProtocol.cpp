#include "open_abb_driver/ABBProtocol.h"

using namespace std;

namespace open_abb_driver
{
		
	string ABBProtocol::PingRobot()
	{
		string msg("00 ");//instruction code;
		msg += "#";
		
		return (msg);
	}
	
	string ABBProtocol::SetCartesian( double x, double y, double z, double q0, double qx, 
								  double qy, double qz )
	{
		char buff[10];
		string msg("01 ");//instruction code;
		
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
	
	string ABBProtocol::SetJoints( double joint1, double joint2, double joint3, double joint4, 
							   double joint5, double joint6 )
	{
		char buff[10];
		string msg("02 ");//instruction code;
		
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
	
	string ABBProtocol::GetCartesian()
	{
		string msg("03 ");//instruction code;
		
		msg += "#";
		return (msg);
	}
	
	string ABBProtocol::GetJoints()
	{
		string msg("04 ");//instruction code;
		
		msg += "#";
		return (msg);
	}
	
	string ABBProtocol::SetTool( double x, double y, double z, double q0, double qx, 
							 double qy, double qz )
	{
		char buff[10];
		string msg("06 ");//instruction code;
		
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
	
	string ABBProtocol::SetWorkObject( double x, double y, double z, double q0, double qx, 
								   double qy, double qz )
	{
		char buff[10];
		string msg("07 ");//instruction code;
		
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
	
	string ABBProtocol::SetSpeed( double tcp, double ori )
	{
		char buff[10];
		string msg("08 ");//instruction code;
		
		sprintf(buff,"%08.1lf ",tcp);
		msg += buff ;
		sprintf(buff,"%08.2lf ",ori);
		msg += buff ;
		msg += "#";
		
		return (msg);
	}
	
	
	string ABBProtocol::SetZone( bool fine, double tcp_mm, double ori_mm, double ori_deg )
	{
		char buff[10];
		string msg("09 ");//instruction code;
		
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
	
	
	string ABBProtocol::SpecialCommand( int command, double param1, double param2, 
									double param3, double param4, double param5 )
	{
		char buff[12];
		string msg("10 ");//instruction code;
		
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
	
	string ABBProtocol::SetDIO( int dio_number, int dio_state )
	{
		char buff[10];
		string msg("26 ");//instruction code;
		
		sprintf(buff,"%.2d ", dio_number);
		msg += buff ;
		sprintf(buff,"%.2d ", dio_state);
		msg += buff ;
		msg += "#";
		return (msg);
	}
	
	string ABBProtocol::CloseConnection()
	{
		string msg("99 ");//instruction code;
		
		msg += "#";
		
		return (msg);
	}
	
	int ABBProtocol::ParseCartesian( const std::string& msg, double* x, double* y, double* z,
								 double* q0, double* qx, double* qy, double* qz )
	{
		int ok, idCode;
		sscanf(msg.c_str(),"%d %d %lf %lf %lf %lf %lf %lf %lf",&idCode,&ok,x,y,z,q0,qx,qy,qz);
		if (ok)
			return idCode;
		else
			return -1;
	}
	
	int ABBProtocol::ParseJoints( const std::string& msg, std::array<double,6>& position )
	{
		int ok, idCode;
		sscanf(msg.c_str(),"%d %d %lf %lf %lf %lf %lf %lf",&idCode,&ok,
			   &position[0], &position[1], &position[2], &position[3], &position[4], &position[5]);
		if (ok)
			return idCode;
		else
			return -1;
	}
	
}
