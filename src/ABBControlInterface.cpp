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

#include "ikfast/ikfast.h"

#include <Eigen/Geometry>

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
	
	bool ABBControlInterface::SetCartesian(double x, double y, double z, double qw, double qx, 
										   double qy, double qz, bool linear)
	{
		
		Eigen::Quaterniond quat( qw, qx, qy, qz );
		Eigen::Matrix<double,3,3,Eigen::RowMajor> rotMat = quat.toRotationMatrix();
		
		ikfast::IkReal rot[9], trans[3];
		// IK array is row-major
		for( unsigned int i = 0; i < 9; i++ )
		{
			rot[i] = rotMat(i);
		}
		trans[0] = x; 
		trans[1] = y; 
		trans[2] = z;
		
		ikfast::IkSolutionList<ikfast::IkReal> solutions;
		
		bool success = ikfast::ComputeIk( trans, rot, NULL, solutions );
		if( !success )
		{
			std::cout << "Failed to get IK solution." << std::endl;
		}
		else
		{
			std::cout << "Found " << solutions.GetNumSolutions() << " solutions!" << std::endl;
		}
		
		// Inputs come in meters, commands given in millimeters
		x *= M_2_MM;
		y *= M_2_MM;
		z *= M_2_MM;
		NormalizeQuaternion( qw, qx, qy, qz );
		
		std::string message = ABBProtocol::SetCartesian(x, y, z, qw, qx, qy, qz, linear);
		char reply[max_buffer_len];
		
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::GetCartesian(double &x, double &y, double &z, 
									   double &qw, double &qx, double &qy, double &qz)
	{
		std::string message = ABBProtocol::GetCartesian();
		char reply[max_buffer_len];
		
		if(SendAndReceive(message, reply))
		{
			ABBProtocol::ParseCartesian(reply, &x, &y, &z, &qw, &qx, &qy, &qz);
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
		{
			return false;
		}
	}
	
	bool ABBControlInterface::SetTool( double x, double y, double z, double qw, double qx, double qy, double qz )
	{
		NormalizeQuaternion( qw, qx, qy, qz );
		x *= M_2_MM;
		y *= M_2_MM;
		z *= M_2_MM;
		
		char message[max_buffer_len];
		char reply[max_buffer_len];
		strcpy(message, ABBProtocol::SetTool( x, y, z, qw, qx, qy, qz ).c_str());
		
		return SendAndReceive( message, reply );
	}
	
	// TODO Change from m to mm
	bool ABBControlInterface::SetWorkObject( double x, double y, double z, double qw, double qx, double qy, double qz )
	{
		NormalizeQuaternion( qw, qx, qy, qz );
		x *= M_2_MM;
		y *= M_2_MM;
		z *= M_2_MM;
		
		char message[max_buffer_len];
		char reply[max_buffer_len];
		strcpy(message, ABBProtocol::SetWorkObject(x, y, z, qw, qx, qy, qz).c_str());
		
		return SendAndReceive(message, reply);
	}
	
	bool ABBControlInterface::SetSpeed(double tcp, double ori)
	{
		tcp *= M_2_MM;
		ori *= M_2_MM;
		
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
	
	bool ABBControlInterface::SetSoftness( const std::array<double,6>& softness )
	{
		std::string message;
		char reply[max_buffer_len];
		
		message = ABBProtocol::SetSoftness( softness[0], softness[1], softness[2], 
											softness[3], softness[4], softness[5] );
		return SendAndReceive( message, reply );
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
	
	bool ABBControlInterface::UnwindAxes( const std::array<double,6>& thresholds )
	{
		std::array<double,6> joints;
		double x, y, z, qw, qx, qy, qz;
		if( !GetCartesian( x, y, z, qw, qx, qy, qz ) ) { return false; }

		while( true )
		{
			if( !GetJoints( joints ) ) { return false; }
			
			bool anyUnwind = false;
			for( unsigned int i = 0; i < 6; i++ )
			{
				if( std::abs( joints[i] ) > thresholds[i] ) 
				{
					joints[i] = 0; //0.01*std::rand();
					anyUnwind = true;
					std::cout << "Unwinding axis " << i+1 << " with thresh " << thresholds[i] << std::endl;
				}
			}
					
			if( !anyUnwind ) { break; }
			
			if( !SetJoints( joints ) ) { return false; }
			if( !SetCartesian( x, y, z, qw, qx, qy, qz, false ) ) { return false; }
		}
		return true;
	}
	
	double ABBControlInterface::NormalizeQuaternion( double& qw, double& qx, double& qy, double& qz )
	{
		double norm = std::sqrt( qw*qw + qx*qx + qy*qy + qz*qz );
		qw = qw/norm;
		qx = qx/norm;
		qy = qy/norm;
		qz = qz/norm;
		return norm;
		
	}
}
