#include "open_abb_driver/ABBKinematics.h"

namespace open_abb_driver
{
	ABBKinematics::ABBKinematics() 
	{
		weights.fill( 1.0 );
	}
		
	void ABBKinematics::SetJointLimits( unsigned int index, std::pair<double,double> limits )
	{
		if( index > 5 )
		{
			throw std::out_of_range( "Joint index must be 0-5." );
		}
		if( limits.first > limits.second )
		{
			throw std::runtime_error( "Joint limit must be in form [lower, upper]." );
		}
		
		
		jointLimits[index] = limits;
	}
		
	void ABBKinematics::SetJointWeights( const JointWeights& w )
	{
		weights = w;
	}
	
	PoseSE3 ABBKinematics::ComputeFK( const JointAngles& angles )
	{
		using namespace ikfast;
		
		IkReal joints[6], rot[9], trans[3];
		for( unsigned int i = 0; i < 6; i++ )
		{
			joints[i] = angles[i];
		}
		// Have to account for joint 2 and 3 parallel link
		joints[2] = joints[2] - joints[1];
		
		ComputeFk( joints, trans, rot );
		
		Eigen::Matrix3d R;
		R(0,0) = rot[0];
		R(0,1) = rot[1];
		R(0,2) = rot[2];
		R(1,0) = rot[3];
		R(1,1) = rot[4];
		R(1,2) = rot[5];
		R(2,0) = rot[6];
		R(2,1) = rot[7];
		R(2,2) = rot[8];
		PoseSE3::Quaternion quat( R );
		PoseSE3::Translation t( trans[0], trans[1], trans[2] );
		
		return PoseSE3( quat, t );
	}
	
	bool ABBKinematics::ComputeIK( const PoseSE3& ref, std::vector<JointAngles>& solutions )
	{
		using namespace ikfast;
		
		IkSolutionList<IkReal> ikSols;
		IkReal rot[9], trans[3];
		
		PoseSE3::Matrix H = ref.ToMatrix();
		rot[0] = H(0,0);
		rot[1] = H(0,1);
		rot[2] = H(0,2);
		rot[3] = H(1,0);
		rot[4] = H(1,1);
		rot[5] = H(1,2);
		rot[6] = H(2,0);
		rot[7] = H(2,1);
		rot[8] = H(2,2);
		
		trans[0] = H(0,3);
		trans[1] = H(1,3);
		trans[2] = H(2,3);
		
		solutions.clear();
		bool success = ComputeIk( trans, rot, NULL, ikSols );
		if( !success ) { return false; }
		
		std::array<IkReal,6> solvalues;
		JointAngles angles;
		for( unsigned int i = 0; i < ikSols.GetNumSolutions(); i++ )
		{
			const IkSolutionBase<IkReal>& sol = ikSols.GetSolution( i );
			sol.GetSolution( solvalues.data(), NULL );
			
			for( unsigned int j = 0; j < 6; j++ )
			{
				angles[j] = solvalues[j];
			}
			// Have to account for joint 2 and 3 parallel link
			angles[2] += angles[1];
			
			bool inLimits = true;
			for( unsigned int j = 0; j < 6; j++ )
			{
				if( angles[j] < jointLimits[j].first || angles[j] > jointLimits[j].second ) 
				{ 
					inLimits = false;
					break;
				}
			}
			
			if( inLimits ) { solutions.push_back( angles ); }
		}
		return true;
	}
	
	JointAngles ABBKinematics::GetBestSolution( const JointAngles& currentAngles,
												   const std::vector<JointAngles>& solutions )
	{
		if( solutions.size() == 0 )
		{
			throw std::runtime_error( "Cannot return solution from empty solution set." );
		}
		
		JointAngles best;
		double bestScore = std::numeric_limits<double>::infinity();
		for( unsigned int i = 0; i < solutions.size(); i++ )
		{
			double score = CalculateScore( currentAngles, solutions[i] );
			if( score < bestScore )
			{
				bestScore = score;
				best = solutions[i];
			}
		}
		return best;
	}
	
	double ABBKinematics::CalculateScore( const JointAngles& a, const JointAngles& b )
	{
		double score = 0;
		for( unsigned int i = 0; i < 6; i++ )
		{
			double diff = std::abs( a[i] - b[i] );
			score += diff * weights[i];
		}
		return score;
	}
	
}