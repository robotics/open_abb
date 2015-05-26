#include <math.h>

#include "open_abb_driver/matvec/Vec.h"
#include "open_abb_driver/matvec/Mat.h"
#include "open_abb_driver/matvec/RotMat.h"
#include "open_abb_driver/matvec/Quaternion.h"

namespace open_abb_driver
{
	
namespace matvec
{
	
Quaternion::Quaternion() : Vec(4) {}

Quaternion::Quaternion(const double constant) : Vec(constant,4) {}

Quaternion::Quaternion(double const *values) : Vec(values,4) {}

Quaternion::Quaternion(char const *string) : Vec(string,4) {}

Quaternion::Quaternion(double const q0, Vec const qv) : Vec(4)
{
  v[0] = q0;
  v[1] = qv[0];
  v[2] = qv[1];
  v[3] = qv[2];
}

Quaternion::Quaternion(const Vec &origin_vector) : Vec(4) 
{
	if (origin_vector.nn==4)
	{
		for(int i=0;i<4;i++)
			v[i]=origin_vector[i];
	}	

}

Quaternion::Quaternion(const Quaternion &origin_quaternion) : Vec((Vec)origin_quaternion) {}

Quaternion & Quaternion::operator =(const double constant){return(*this=Quaternion((Vec)*this=constant));}
Quaternion & Quaternion::operator +=(const Quaternion &original){return(*this=Quaternion((Vec)*this+=(Vec)original));}
Quaternion & Quaternion::operator -=(const Quaternion &original){return(*this=Quaternion((Vec)*this-=(Vec)original));}
Quaternion Quaternion::operator +(const Quaternion &original)const {return (Quaternion((Vec)*this+(Vec)original));}
Quaternion Quaternion::operator -(const Quaternion &original)const {return (Quaternion((Vec)*this-(Vec)original));}
Quaternion Quaternion::operator -()const {return(Quaternion(-(Vec)*this));}
Quaternion Quaternion::operator *(const double constant)const {return (Quaternion((Vec)*this*constant));}
Quaternion Quaternion::operator /(const double constant)const {return (Quaternion((Vec)*this/constant));}
Quaternion Quaternion::operator ^(const Quaternion &original)const 
{
	Quaternion w;
	w[0]=v[0]*original[0] - v[1]*original[1] - v[2]*original[2] - v[3]*original[3];
	w[1]=v[0]*original[1] + v[1]*original[0] + v[2]*original[3] - v[3]*original[2];
	w[2]=v[0]*original[2] - v[1]*original[3] + v[2]*original[0] + v[3]*original[1];
	w[3]=v[0]*original[3] + v[1]*original[2] - v[2]*original[1] + v[3]*original[0];

	return w;
}
double Quaternion::operator *(const Quaternion &original)const 
{
	double e;
	e=v[0]*original[0] + v[1]*original[1] + v[2]*original[2] + v[3]*original[3];

	return e;
}

Quaternion Quaternion::conjugate()const 
{
	Quaternion w;
	w[0]=v[0];
	w[1]=-v[1];
	w[2]=-v[2];
	w[3]=-v[3];

	return w;
}

double Quaternion::getScalar()const 
{
	return v[0];
}

Vec Quaternion::getVector()const 
{
	Vec w(3);
	w[0]=v[1];
	w[1]=v[2];
	w[2]=v[3];
	return w;
}

void Quaternion::setScalar(const double s)
{
	v[0]=s;
}

void  Quaternion::setVector(const Vec &vector) 
{
	if(vector.nn==3)
	{
		v[1]=vector[0];
		v[2]=vector[1];
		v[3]=vector[2];
	}
}

Quaternion Quaternion::inverse()const 
{
	double n=norm();
	if (n!=0)
		return ((this->conjugate())/(n*n));
	else
		return (Quaternion(0.0));
}

Mat Quaternion::leftMat()const 
{
	Mat w(0.0,4,4);
	w[0][0]=v[0]   ; w[0][1]=-v[1]   ; w[0][2]=-v[2]   ; w[0][3]=-v[3]   ;
	w[1][0]=v[1]   ; w[1][1]=v[0]   ; w[1][2]=-v[3]   ; w[1][3]=v[2]   ;
	w[2][0]=v[2]   ; w[2][1]=v[3]   ; w[2][2]=v[0]   ; w[2][3]=-v[1]   ;
	w[3][0]=v[3]   ; w[3][1]=-v[2]   ; w[3][2]=v[1]   ; w[3][3]=v[0]   ;

	return w;
}

Mat Quaternion::rightMat()const 
{
	Mat w(0.0,4,4);
	w[0][0]=v[0]   ; w[0][1]=-v[1]   ; w[0][2]=-v[2]   ; w[0][3]=-v[3]   ;
	w[1][0]=v[1]   ; w[1][1]=v[0]   ; w[1][2]=v[3]   ; w[1][3]=-v[2]   ;
	w[2][0]=v[2]   ; w[2][1]=-v[3]   ; w[2][2]=v[0]   ; w[2][3]=v[1]   ;
	w[3][0]=v[3]   ; w[3][1]=v[2]   ; w[3][2]=-v[1]   ; w[3][3]=v[0]   ;

	return w;
}

double Quaternion::getAngle()const 
{
	Vec vector=getVector();
	double s=getScalar();
	double a=2*atan2(vector.norm(),s);
	if (a>PI)
		a=2*PI-a;
	return a;
}

Vec Quaternion::getAxis()const 
{
	double n=getVector().norm();
	if (n!=0)
		return(getVector()/n);
	else
		return(Vec("0.0 0.0 0.0",3));
}

RotMat Quaternion::getRotMat()const 
{
	RotMat w;
	w[0][0]=v[0]*v[0]+v[1]*v[1]-v[2]*v[2]-v[3]*v[3];
	w[0][1]=2*(v[1]*v[2]-v[0]*v[3]);
	w[0][2]=2*(v[1]*v[3]+v[0]*v[2]);
	w[1][0]=2*(v[1]*v[2]+v[0]*v[3]);
	w[1][1]=v[0]*v[0]-v[1]*v[1]+v[2]*v[2]-v[3]*v[3];
	w[1][2]=2*(v[2]*v[3]-v[0]*v[1]);
	w[2][0]=2*(v[1]*v[3]-v[0]*v[2]);
	w[2][1]=2*(v[2]*v[3]+v[0]*v[1]);
	w[2][2]=v[0]*v[0]-v[1]*v[1]-v[2]*v[2]+v[3]*v[3];
	return w;
}

}

}