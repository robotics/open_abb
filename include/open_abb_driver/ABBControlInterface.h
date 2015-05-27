#ifndef _ABB_INTERFACE_H_
#define _ABB_INTERFACE_H_

#include <boost/thread/thread.hpp>
// TODO Use boost ptimes for date/time
#include <array>
#include <memory>

namespace open_abb_driver
{
	
	class ABBControlInterface
	{
	public:
		
		typedef std::shared_ptr<ABBControlInterface> Ptr;
		
		struct ZoneValue
		{
			double p_tcp; // TCP path zone (mm)
			double p_ori; // Zone size for orientation (mm)
			double ori;   // Tool orientation (degrees)
		};
	
		enum ZoneType
		{
			ZONE_FINE = 0,
			ZONE_1,
			ZONE_2,
			ZONE_3,
			ZONE_4,
			ZONE_5,
			NUM_ZONES
		};
	
		static const ZoneValue zone_values[NUM_ZONES];
		static const int max_buffer_len;
		
		ABBControlInterface( const std::string& ip, int port );
		~ABBControlInterface();
				
		/*! \brief Reads data off the logger socket and populates the output queue. */
		void LoggerSpin();
		
		/*! \brief Ping the robot controller. */
		bool Ping();

		/*! \brief Specify the TCP cartesian coordinates in meters. */
		bool SetCartesian( double x, double y, double z, double q0, double qx, double qy, double qz );
		/*! \brief Retrieve the TCP cartesian coordinates in meters. */
		bool GetCartesian( double &x, double &y, double &z, double &q0, double &qx, double &qy, double &qz );
		
		/*! \brief Specify the joint angles in radians. */
		bool SetJoints( const std::array<double,6>& position );
		/*! \brief Retrieve the joint angles in radians. */
		bool GetJoints( std::array<double,6>& position );
		
		/*! \brief Specify the tool coordinate frame. */
		bool SetTool( double x, double y, double z, double q0, double qx, double qy, double qz );
		/*! \brief Specify the work object coordinate frame. */
		bool SetWorkObject( double x, double y, double z, double q0, double qx, double qy, double qz );
		
		bool SpecialCommand( int command, double param1=0, double param2=0, double param3=0, double param4=0, double param5=0 );
		bool SetDIO( int dio_num, int dio_state );
		
		/*! \brief Specify the translation and orientation speeds in mm/s. */
		bool SetSpeed( double tcp, double ori );
		
		/*! \brief Set the tracking precision (zone). */
		bool SetZone( int z );
		
	private:
		
		enum ServerAck
		{
			SERVER_BAD_MSG = 0,
			SERVER_OK = 1
		};
		
		typedef boost::shared_mutex Mutex;
		typedef boost::unique_lock< Mutex > WriteLock;
		typedef boost::shared_lock< Mutex > ReadLock;
		
		Mutex motionSocketMutex;

		int motionSocket;
		
		// Helper function for communicating with robot server
		bool SendAndReceive( const std::string& message, char* reply );
		
		static double NormalizeQuaternion( double& q0, double& qx, double& qy, double& qz );
	};
}

#endif
