#include <errno.h>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h> 


#include "open_abb_driver/ABBProtocol.h"
#include "open_abb_driver/ABBControlInterface.h"

#define RAD_2_DEG (57.2957795)
#define DEG_2_RAD (0.01745329251)
#define MM_2_M (0.001)
#define M_2_MM (1000)

namespace open_abb_driver
{
	
	const ABBControlInterface::ZoneValue 
	ABBControlInterface::zone_values[ABBControlInterface::NUM_ZONES] = 
	{
		// p_tcp (mm), p_ori (mm), ori (deg)
		{0.0,   0.0,  0.0},   // ZONE_FINE
		{0.3,   0.3,  0.03},  // ZONE_1
		{1.0,   1.0,  0.1},   // ZONE_2
		{5.0,   8.0,  0.8},   // ZONE_3
		{10.0,  15.0, 1.5},   // ZONE_4
		{20.0,  30.0, 3.0}    // ZONE_5
	};
	
	const int ABBControlInterface::max_buffer_len = 10000;
	
	ABBControlInterface::ABBControlInterface( const std::string& ip, int port )
	{
		motionSocket = socket(PF_INET, SOCK_STREAM, 0);
		if( motionSocket == -1 )
		{
			throw std::runtime_error( "Could not open the control socket." );
		}
		
		struct sockaddr_in remoteSocket;
		remoteSocket.sin_family = AF_INET;
		remoteSocket.sin_port = htons(port);
		inet_pton( AF_INET, ip.c_str(), &remoteSocket.sin_addr.s_addr );
		if( connect( motionSocket, (sockaddr*)&remoteSocket, sizeof(remoteSocket) ) == -1 )
		{
			throw std::runtime_error( "Could not connect the control socket." );
		}
		
	}
	
	ABBControlInterface::~ABBControlInterface()
	{
		std::string message = ABBProtocol::CloseConnection();
		send( motionSocket, message.c_str(), message.size(), 0 );
		usleep( 1E6 );
		close(motionSocket);
	}
	
	bool ABBControlInterface::Ping()
	{
		std::string message = ABBProtocol::PingRobot();
		char reply[max_buffer_len];
		
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::SetCartesian(double x, double y, double z, 
									   double q0, double qx, double qy, double qz)
	{
		// Inputs come in meters, commands given in millimeters
		x *= M_2_MM;
		y *= M_2_MM;
		z *= M_2_MM;
		NormalizeQuaternion( q0, qx, qy, qz );
		
		std::string message = ABBProtocol::SetCartesian(x, y, z, q0, qx, qy, qz);
		char reply[max_buffer_len];
		
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::GetCartesian(double &x, double &y, double &z, 
									   double &q0, double &qx, double &qy, double &qz)
	{
		std::string message = ABBProtocol::GetCartesian();
		char reply[max_buffer_len];
		
		if(SendAndReceive(message, reply))
		{
			ABBProtocol::ParseCartesian(reply, &x, &y, &z, &q0, &qx, &qy, &qz);
			x *= MM_2_M;
			y *= MM_2_M;
			z *= MM_2_M;
			return true;
		}
		else
		{
			return false;
		}
	}
	
	bool ABBControlInterface::SetJoints( const std::array<double,6>& position )
	{
		std::string message = ABBProtocol::SetJoints( position[0]*RAD_2_DEG, 
													  position[1]*RAD_2_DEG,
													  position[2]*RAD_2_DEG,
													  position[3]*RAD_2_DEG,
													  position[4]*RAD_2_DEG,
													  position[5]*RAD_2_DEG );
		char reply[max_buffer_len];
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::GetJoints( std::array<double,6>& position )
	{
		std::string message = ABBProtocol::GetJoints();
		char reply[max_buffer_len];
		
		if( SendAndReceive( message, reply ) )
		{
			ABBProtocol::ParseJoints( reply, position );
			position[0] *= DEG_2_RAD;
			position[1] *= DEG_2_RAD;
			position[2] *= DEG_2_RAD;
			position[3] *= DEG_2_RAD;
			position[4] *= DEG_2_RAD;
			position[5] *= DEG_2_RAD;
			return true;
		}
		else
			return false;
	}
	
	bool ABBControlInterface::SetTool( double x, double y, double z, double q0, double qx, double qy, double qz )
	{
		NormalizeQuaternion( q0, qx, qy, qz );
		
		char message[max_buffer_len];
		char reply[max_buffer_len];
		strcpy(message, ABBProtocol::SetTool( x, y, z, q0, qx, qy, qz ).c_str());
		
		return SendAndReceive( message, reply );
	}
	
	bool ABBControlInterface::SetWorkObject( double x, double y, double z, double q0, double qx, double qy, double qz )
	{
		NormalizeQuaternion( q0, qx, qy, qz );
		
		char message[max_buffer_len];
		char reply[max_buffer_len];
		strcpy(message, ABBProtocol::SetWorkObject(x, y, z, q0, qx, qy, qz).c_str());
		
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::SetSpeed(double tcp, double ori)
	{
		char message[max_buffer_len];
		char reply[max_buffer_len];
		strcpy(message, ABBProtocol::SetSpeed(tcp, ori).c_str());
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::SetZone(int z)
	{
		std::string message;
		char reply[max_buffer_len];
		
		// Make sure the specified zone number exists
		if (z < 0 || z > NUM_ZONES)
		{
			std::cout << "Invalid zone mode of " << z  << std::endl;
			return false;
		}

		message = ABBProtocol::SetZone( z == ZONE_FINE, zone_values[z].p_tcp, zone_values[z].p_ori, zone_values[z].ori );
		return SendAndReceive(message, reply);

	}
	
	bool ABBControlInterface::SpecialCommand(int command, double param1, double param2, double param3, double param4, double param5)
	{
		std::string message = ABBProtocol::SpecialCommand(command, param1, param2, param3, param4, param5);
		char reply[max_buffer_len];
		
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::SetDIO(int dio_num, int dio_state)
	{
		std::string message = ABBProtocol::SetDIO(dio_num, dio_state);
		char reply[max_buffer_len];
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::SendAndReceive( const std::string& message, char* reply )
	{
		WriteLock lock( motionSocketMutex );
		if ( send(motionSocket, message.c_str(), message.size(), 0) == -1 )
		{
			std::cout << "Failed to send command to robot: " << errno << std::endl;
			return false;
		}
		
		int t = recv( motionSocket, reply, max_buffer_len-1, 0 );
		
		if( t <= 0 )
		{
			std::cout << "Failed to receive answer from ABB robot." << std::endl;
			return false;
		}
		
		reply[t] = '\0';
		int rcvIdCode, ok;
		sscanf(reply,"%d %d", &rcvIdCode, &ok);
		if( rcvIdCode == -1 )
		{
			std::cout << "Received non-ack from robot. " << std::endl;
			return false;
		}
		
		if (ok == SERVER_OK)
		{
			return true;
		}

		std::cout << "Invalid ack value of " << ok << std::endl;
		return false;
	}
	
	
	
	double ABBControlInterface::NormalizeQuaternion( double& q0, double& qx, double& qy, double& qz )
	{
		double norm = std::sqrt( q0*q0 + qx*qx + qy*qy + qz*qz );
		q0 = q0/norm;
		qx = qx/norm;
		qy = qy/norm;
		qz = qz/norm;
		return norm;
		
	}
}
