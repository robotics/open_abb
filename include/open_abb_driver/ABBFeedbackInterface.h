#ifndef ABB_INTERFACE_FEEDBACK_H_
#define ABB_INTERFACE_FEEDBACK_H_

#include <boost/thread/thread.hpp>
#include <boost/variant.hpp>
#include <boost/circular_buffer.hpp>

#include <memory>

namespace open_abb_driver
{
	
	class ABBFeedbackInterface 
	{
	public:
		
		typedef std::shared_ptr<ABBFeedbackInterface> Ptr;
		
		struct JointFeedback
		{
			std::string date;
			std::string time;
			std::array<double,6> joints;
		};
		
		struct CartesianFeedback
		{
			std::string date;
			std::string time;
			double x;
			double y;
			double z;
			double qw;
			double qx;
			double qy;
			double qz;
		};
		
		typedef boost::variant< JointFeedback, CartesianFeedback > Feedback;
		
		ABBFeedbackInterface( const std::string& ip, int port, size_t bufferSize = 10 );
		~ABBFeedbackInterface();
		
		void Spin();
		
	private:
		
		int loggerSocket;
		
		boost::circular_buffer<Feedback> outgoing;
		
	};
}

#endif
