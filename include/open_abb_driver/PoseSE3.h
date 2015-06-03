#ifndef _POSE_SE3_H_
#define _POSE_SE3_H_

#include <Eigen/Geometry>
#include <iostream>

namespace open_abb_driver
{

	class PoseSE3 
	{
	public:

		static const int VectorDimension = 7;
		static const int TangentDimension = 6;
		static const int CovarianceDimension = 21;
		
		// Representation is [x, y, z, qw, qx, qy, qz]
		typedef double ScalarType;
		typedef Eigen::Matrix<ScalarType, VectorDimension, 1> Vector;
		typedef Eigen::Matrix<ScalarType, 3, 1> TranslationVector;
		typedef Eigen::Matrix<ScalarType, 3, 1> AxisVector;

		struct EulerAngles 
		{
			ScalarType yaw;
			ScalarType pitch;
			ScalarType roll;
		};
		
		typedef Eigen::Transform<ScalarType, 3, Eigen::Isometry> Transform;
		typedef Eigen::Matrix<ScalarType, 4, 4> Matrix;
		typedef Eigen::Quaternion<ScalarType> Quaternion;
		typedef Eigen::Translation<ScalarType, 3> Translation;

		// Probability definitions
		typedef Eigen::Matrix<ScalarType, TangentDimension, TangentDimension> CovarianceMatrix;
		typedef Eigen::Matrix<ScalarType, CovarianceDimension, 1> CovarianceVector;
		
		// Lie group definitions
		typedef Eigen::Matrix<ScalarType, TangentDimension, 1> TangentVector;
		typedef Eigen::Matrix<ScalarType, TangentDimension, TangentDimension> AdjointMatrix;
		
		PoseSE3();
		explicit PoseSE3( double x, double y, double z, double yaw,
						  double pitch, double roll );
		explicit PoseSE3( double x, double y, double z, double qw, double qx, double qy, double qz );
		explicit PoseSE3( const TranslationVector& t, double angle, const AxisVector& a );
		explicit PoseSE3( const Vector& vec );
		explicit PoseSE3( const Transform& trans );
		explicit PoseSE3( const Matrix& mat );
		explicit PoseSE3( Quaternion q, const Translation& t );

		Matrix ToMatrix() const;
		Transform ToTransform() const;
		Vector ToVector() const; //[x,y,z,qw,qx,qy,qz]
		PoseSE3 Inverse() const;

		PoseSE3::Translation GetTranslation() const;
		PoseSE3::Quaternion GetQuaternion() const;
		PoseSE3::EulerAngles GetEulerAngles() const;

		/*! \brief Integrates for unit time along the velocity direction starting
			* from this PoseSE3. Returns a new PoseSE3. */
		PoseSE3 Exp( const TangentVector& other ) const;

		/*! \brief Returns a velocity that starting from this PoseSE3 reaches
			* other in unit time. */
		PoseSE3::TangentVector Log( const PoseSE3& other ) const;

		/*! \brief Returns the adjoint matrix that maps velocities around the identity
			* to around this PoseSE3. */
		PoseSE3::AdjointMatrix GetAdjoint() const;

		/*! \brief Applies the adjoint to a velocity around the identity to around
			* this PoseSE3. */
		PoseSE3::TangentVector Adjoint( const TangentVector& other ) const;
		
		PoseSE3 operator+() const;
		PoseSE3 operator-() const;
		PoseSE3 operator*( const PoseSE3& other ) const;
		PoseSE3 operator/( const PoseSE3& other ) const;
		
	protected:

		Transform tform;

	};

	PoseSE3 se3exp( const PoseSE3::TangentVector& velocity );
	PoseSE3::TangentVector se3log( const PoseSE3& se3 );
	
	// TODO Think about what these even mean...
	PoseSE3 operator+( const PoseSE3& se3, PoseSE3::Vector& vec );
	PoseSE3 operator+( PoseSE3::Vector& vec, const PoseSE3& se3 );
	PoseSE3 operator-( const PoseSE3& se3, PoseSE3::Vector& vec );
	PoseSE3 operator-( PoseSE3::Vector& vec, const PoseSE3& se3 );

	std::ostream& operator<<( std::ostream& os, const PoseSE3& se3 );
	std::ostream& operator<<( std::ostream& os, const PoseSE3::EulerAngles& eul );
	
	// TODO Implement!
	PoseSE3::CovarianceVector unroll_covariance( const PoseSE3::CovarianceMatrix& cov );
	PoseSE3::CovarianceMatrix rollup_covariacne( const PoseSE3::CovarianceVector& vec );

	template<class C>
	Eigen::Matrix<C,3,3> cross_product_matrix( const Eigen::Matrix<C,3,1>& v );

	// Helper for exp functions
	struct SECoefficients {
		double a;
		double b;
		double c;
		SECoefficients( double theta );
	};
	
}

#endif
