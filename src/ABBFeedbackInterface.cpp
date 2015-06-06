#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h> 

#include "open_abb_driver/ABBFeedbackInterface.h"

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#define RAD_2_DEG (57.2957795)
#define DEG_2_RAD (0.01745329251)
#define MM_2_M (0.001)
#define M_2_MM (1000)

namespace open_abb_driver
{
	
	ABBFeedbackInterface::ABBFeedbackInterface( const std::string& ip, int port,
												size_t bufferSize )
		: outgoing( bufferSize )
	{
		
		loggerSocket = socket(PF_INET, SOCK_STREAM, 0);
		if( loggerSocket == -1 )
		{
			std::stringstream ss;
			ss << "Could not connect to logger socket [" << errno << "]";
			throw std::runtime_error( ss.str() );
		}
		
		struct sockaddr_in remoteSocket;
		remoteSocket.sin_family = AF_INET;
		remoteSocket.sin_port = htons( port );
		inet_pton( AF_INET, ip.c_str(), &remoteSocket.sin_addr.s_addr );
		if( connect( loggerSocket, (sockaddr*)&remoteSocket, sizeof(remoteSocket) ) == -1 )
		{
			std::stringstream ss;
			ss << "Could not connect to the logger server [" << errno << "]";
			throw std::runtime_error( ss.str() );
		}
		
		//Set socket as non-blocking
		int flags;
		flags = fcntl( loggerSocket, F_GETFL, 0 );
		if( flags == -1 )
		{
			throw std::runtime_error( "Could not read the socket flags." );
		}
		
		if(fcntl(loggerSocket, F_SETFL, flags | O_NONBLOCK) == -1)
		{
			throw std::runtime_error( "Could not set the socket to non-blocking mode." );
		}
		
	}
	
	ABBFeedbackInterface::~ABBFeedbackInterface()
	{
		close( loggerSocket );
	}
	
	void ABBFeedbackInterface::Spin()
	{
		int t;
		int code;
		char buffer[2000];
		const char* partialBuffer;
		
		// Read all information from the tcp/ip socket
		if( (t=recv(loggerSocket, buffer, 2000-1, 0)) > 0 )
		{
			// Add an end character to form our string
			buffer[t] = '\0';
			
			std::string buff( buffer );
			std::vector< std::string > splits;
			boost::split( splits, buff, boost::is_any_of( "#" ) );
			
			BOOST_FOREACH( const std::string& chunk, splits )
			{
				if( chunk.size() == 0 ) { continue; }
				partialBuffer = chunk.c_str();
				// The number after the start character is the type of message
				sscanf(partialBuffer,"%d", &code);
				Feedback msg;
				
				if( code == 0 )
				{
					CartesianFeedback cmsg;
					char date[2000];
					char time[2000];
					sscanf(partialBuffer,"%*d %s %s %*f %lf %lf %lf %lf %lf %lf %lf",
										date,
										time,
										&cmsg.x, &cmsg.y, &cmsg.z,
										&cmsg.qw, &cmsg.qx, &cmsg.qy, &cmsg.qz);
				
					cmsg.x *= MM_2_M;
					cmsg.y *= MM_2_M;
					cmsg.z *= MM_2_M;

					msg = cmsg;
				}
				else if( code == 1 )
				{
					JointFeedback jmsg;
					char date[2000];
					char time[2000];
					sscanf(partialBuffer,"%*d %s %s %*f %lf %lf %lf %lf %lf %lf",
										date,
										time,
										&jmsg.joints[0],
										&jmsg.joints[1],
										&jmsg.joints[2],
										&jmsg.joints[3],
										&jmsg.joints[4],
										&jmsg.joints[5] );
					
					// If we read in the correct number of parameters, save this message
					for ( int i = 0; i < jmsg.joints.size(); i++) 
					{
						jmsg.joints[i] *= DEG_2_RAD;
					}
					msg = jmsg;
				}
				else
				{
					std::stringstream ss;
					ss << "Received invalid feedback code [" << code << "]";
					throw std::runtime_error( ss.str() );
				}
				
				boost::unique_lock<boost::mutex> lock( mutex );
				outgoing.push_back( msg );
			}
		}
	}
	
	bool ABBFeedbackInterface::HasFeedback() const
	{
		boost::unique_lock<boost::mutex> lock( mutex );
		return !outgoing.empty();
	}
	
	Feedback ABBFeedbackInterface::GetFeedback()
	{
		boost::unique_lock<boost::mutex> lock( mutex );
		Feedback ret = outgoing.front();
		outgoing.pop_front();
		return ret;
	}
	
}
