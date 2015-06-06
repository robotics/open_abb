#include "open_abb_driver/PoseSE3.h"

#include <cmath>

namespace open_abb_driver 
{

	SECoefficients::SECoefficients( double theta ) 
	{
		double tt = theta*theta;
		
		if( tt < 1E-6 ) 
		{
			b = 0.5 + tt/24 - tt*tt/720;
			c = 1.0/6 - tt/120 + tt*tt/5040;
			a = 1 - tt*c;
		} 
		else 
		{
			a = sin(theta)/theta;
			b = (1 - cos(theta))/tt;
			c = (1 - a)/tt;
		}
	}
	
	PoseSE3::PoseSE3() 
	{
		tform.setIdentity();
	}

	PoseSE3::PoseSE3( const double x, const double y, const double z, const double yaw,
						const double pitch, const double roll ) 
	{	
		Quaternion quat( Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
						* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) );
		Translation trans(x, y, z);
		Translation zero(0,0,0);
		Transform qTrans = quat*zero;
		tform = trans*qTrans;
	}
	
	PoseSE3::PoseSE3( double x, double y, double z, double qw, double qx, double qy, double qz )
	{
		Quaternion quat( qw, qx, qy, qz );
		quat.normalize();
		Translation trans( x, y, z );
		Translation zero( 0, 0, 0 );
		tform = trans*(quat*zero);
	}
	
	PoseSE3::PoseSE3( const TranslationVector& t, double angle, const AxisVector& a ) 
	{
		Quaternion quat( Eigen::AngleAxisd( angle, a ) );
		Translation trans( t );
		Translation zero(0,0,0);
		Transform qTrans = quat*zero;
		tform = trans*qTrans;
	}

	PoseSE3::PoseSE3( const Vector& vec ) 
	{
		Quaternion quat( vec(3), vec(4), vec(5), vec(6));
		Translation trans(vec(0), vec(1), vec(2));
		quat.normalize();
		Translation zero(0,0,0);
		Transform qTrans = quat*zero;
		tform = trans*qTrans;
	}

	PoseSE3::Matrix PoseSE3::ToMatrix() const 
	{
		return tform.matrix();
	}

	PoseSE3::PoseSE3( const Transform& t ) 
		: tform( t ) 
	{}

	PoseSE3::PoseSE3( const Matrix& mat ) 
		: tform( mat ) 
	{}

	PoseSE3::PoseSE3( Quaternion q, const Translation& t ) 
	{
		Translation trans( t );
		q.normalize();
		Translation zero(0,0,0);
		Transform qTrans = q*zero;
		tform = trans*qTrans;
	}
	
	PoseSE3::Translation PoseSE3::GetTranslation() const 
	{
		return Translation( tform.translation() );
	}

	PoseSE3::Quaternion PoseSE3::GetQuaternion() const 
	{
		Quaternion quat( tform.linear() );
		quat.normalize();
		return quat;
	}

	PoseSE3::EulerAngles PoseSE3::GetEulerAngles() const 
	{
		double temp1, temp2;
		double psi, theta, phi;
		Quaternion quat = GetQuaternion();
		
		temp1 = quat.w()*quat.y() - quat.z()*quat.x();
		if(temp1 > 0.999) 
		{
			psi = 2*atan2(quat.w(), quat.x());
			theta = M_PI/2;
			phi = 0;
		} 
		else if(temp1 < -0.999) 
		{
			psi = -2*atan2(quat.w(), quat.x());
			theta = -M_PI/2;
			phi = 0;
		} 
		else 
		{
			theta = asin(2.0*temp1);
			temp1 = 2.0*(quat.w()*quat.x() + quat.y()*quat.z());
			temp2 = 1.0 - 2.0*(quat.x()*quat.x() + quat.y()*quat.y());
			phi = atan2(temp1, temp2);
			temp1 = 2.0*(quat.w()*quat.z() + quat.x()*quat.y());
			temp2 = 1.0 - 2.0*(quat.y()*quat.y() + quat.z()*quat.z());
			psi = atan2(temp1, temp2);
		}

		EulerAngles e;
		e.yaw = psi;
		e.pitch = theta;
		e.roll = phi;
		return e;
	}
	
	PoseSE3::Vector PoseSE3::ToVector() const 
	{
		Vector v;
		Transform::ConstTranslationPart trans = tform.translation();
		Quaternion quat = GetQuaternion();
		v << trans(0), trans(1), trans(2), quat.w(), quat.x(), quat.y(), quat.z();
		return v;
	}

	PoseSE3::Transform PoseSE3::ToTransform() const 
	{
		// This forces the composition of translation and quat to produce an isometry
		// as only quaternion right multiplication produces an isometry.
		return tform;
	}

	PoseSE3 PoseSE3::Inverse() const 
	{
//             Translation tInv( quat.inverse()*(-trans.translation()) );
//             Quaternion rInv = quat.inverse();

		PoseSE3 ret( tform.inverse() );
		return ret;
	}
	
	PoseSE3 PoseSE3::Exp( const PoseSE3::TangentVector& other ) const 
	{
		PoseSE3 delta = se3exp( other );
		return delta*(*this);
	}

	PoseSE3 se3exp( const PoseSE3::TangentVector& other ) 
	{
		PoseSE3::TranslationVector u = other.block<3,1>(0,0);
		PoseSE3::AxisVector w = other.block<3,1>(3,0);
		
		double theta = w.norm();
		SECoefficients coeffs( theta );
		
		PoseSE3::Quaternion::Matrix3 wx = cross_product_matrix(w);
		PoseSE3::Quaternion::Matrix3 R = PoseSE3::Quaternion::Matrix3::Identity() + coeffs.a*wx + coeffs.b*wx*wx;
		PoseSE3::Quaternion::Matrix3 V = PoseSE3::Quaternion::Matrix3::Identity() + coeffs.b*wx + coeffs.c*wx*wx;
		PoseSE3::Matrix m = PoseSE3::Matrix::Identity();
		m.block<3,3>(0,0) = R;
		m.block<3,1>(0,3) = V*u;
		
		PoseSE3 ret(m);
		return ret;
	}

	// TODO Remove! This is convention-dependant
	// Adapted from code by Ethan Eade
	PoseSE3::TangentVector PoseSE3::Log( const PoseSE3& other ) const 
	{
		PoseSE3 diff = other/(*this);
		return se3log( diff );
	}

	PoseSE3::TangentVector se3log( const PoseSE3& diff ) 
	{
		PoseSE3::Transform::LinearMatrixType R = diff.ToTransform().rotation();
		
		PoseSE3::AxisVector w;
		w << 0.5 * (R(2,1) - R(1,2)),
		0.5 * (R(0,2) - R(2,0)),
		0.5 * (R(1,0) - R(0,1));
		
		double ct = 0.5  * (R.trace() - 1);
		double st = w.norm();
		double st2 = st*st;
		
		if (ct > 0.999856) 
		{
			// Small angles
			// Taylor expansion of f(x) = arcsin(x) / x
			// x^2 = st2
			double f = 1 + st2*( 1/6.0 + st2*(3/40.0 + st2*5/112.0) );
			w *= f;
		} 
		else if (ct > -0.99) 
		{
			w *= acos(ct)/sqrt(st);
		} 
		else 
		{
			// Angles near pi
			double theta = M_PI - asin(st);
			double invB = (theta*theta) / (1 - ct);
			
			double w00 = invB*(R(0,0) - ct);
			double w11 = invB*(R(1,1) - ct);
			double w22 = invB*(R(2,2) - ct);
			
			double w01 = invB*0.5*(R(0,1) + R(1,0));
			double w02 = invB*0.5*(R(0,2) + R(2,0));
			double w12 = invB*0.5*(R(1,2) + R(2,1));
			
			// Take sqrt of biggest element of w
			if (w00 > w11 && w00 > w22) 
			{
				w(0) = (w(0) < 0 ? -1 : 1) * sqrt(w00);
				w(1) = w01/w(0);
				w(2) = w02/w(0);
			} 
			else if (w11 > w22 && w11 > w00) 
			{
				w(1) = (w(1) < 0 ? -1 : 1) * sqrt(w11);
				w(0) = w01/w(1);
				w(2) = w12/w(1);
			} 
			else 
			{
				w(2) = (w(2) < 0 ? -1 : 1) * sqrt(w22);
				w(0) = w02 / w(2);
				w(1) = w12 / w(2);
			}
		}
		
		double theta = w.norm();
		SECoefficients coeffs( theta );
		
		PoseSE3::Quaternion::Matrix3 wx = cross_product_matrix( w );
		PoseSE3::Quaternion::Matrix3 V = PoseSE3::Quaternion::Matrix3::Identity() + coeffs.b*wx + coeffs.c*wx*wx;
		PoseSE3::TranslationVector t = diff.GetTranslation().vector();
		PoseSE3::TranslationVector u = V.partialPivLu().solve(t);
		
		PoseSE3::TangentVector v;
		v.block<3,1>(0,0) = u;
		v.block<3,1>(3,0) = w;
		return v;
	}

	PoseSE3::AdjointMatrix PoseSE3::GetAdjoint() const 
	{
		AdjointMatrix m = AdjointMatrix::Zero();
		m.block<3,3>(0,0) = tform.linear();
		m.block<3,3>(3,3) = tform.linear();
		TranslationVector t = tform.translation();
		m.block<3,3>(0,3) = cross_product_matrix(t)*tform.linear();
		return m;
	}

	PoseSE3::TangentVector PoseSE3::Adjoint( const TangentVector& other ) const 
	{
		AdjointMatrix m = GetAdjoint();
		return m*other;
	}

	PoseSE3 PoseSE3::operator+() const 
	{
		return *this;
	}

	PoseSE3 PoseSE3::operator-() const 
	{
		return Inverse();
	}

	PoseSE3 PoseSE3::operator*( const PoseSE3& other ) const 
	{
		Transform tTrans = ToTransform();
		Transform oTrans = other.ToTransform();
		return PoseSE3( tTrans*oTrans );
	}

	PoseSE3 PoseSE3::operator/( const PoseSE3& other ) const 
	{
		Transform tTrans = ToTransform();
		Transform oTrans = other.ToTransform().inverse(Eigen::Isometry);
		return PoseSE3( tTrans*oTrans );
	}

	PoseSE3 operator+( const PoseSE3& se3, PoseSE3::Vector& vec ) 
	{
		PoseSE3::Vector v = se3.ToVector();
		PoseSE3::Vector u = v + vec;
		return PoseSE3( u );
	}

	PoseSE3 operator+( PoseSE3::Vector& vec, const PoseSE3& se3 ) 
	{
		PoseSE3::Vector v = se3.ToVector();
		PoseSE3::Vector u = v + vec;
		return PoseSE3( u );
	}

	PoseSE3 operator-( const PoseSE3& se3, PoseSE3::Vector& vec ) 
	{
		PoseSE3::Vector v = se3.ToVector();
		PoseSE3::Vector u = v - vec;
		return PoseSE3( u );
	}

	PoseSE3 operator-( PoseSE3::Vector& vec, const PoseSE3& se3 ) 
	{
		PoseSE3::Vector v = se3.ToVector();
		PoseSE3::Vector u = vec - v;
		return PoseSE3( u );
	}

	std::ostream& operator<<(std::ostream& os, const PoseSE3& se3) 
	{
		PoseSE3::Vector v = se3.ToVector();
		os << v.transpose();
		return os;
	}

	template <class C>
	Eigen::Matrix<C,3,3> cross_product_matrix( const Eigen::Matrix<C,3,1>& v ) 
	{
		Eigen::Matrix<C,3,3> s;
		s << 0, -v(2), v(1),
				v(2), 0, -v(0),
				-v(1), v(0), 0;
		return s;
	}

	std::ostream& operator<<( std::ostream& os, const PoseSE3::EulerAngles& eul ) 
	{
		os << "Y: " << eul.yaw << " P: " << eul.pitch << " R: " << eul.roll;
		return os;
	}
	
}
