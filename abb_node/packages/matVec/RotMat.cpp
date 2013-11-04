#include <math.h>

#include "Vec.h"
#include "Mat.h"
#include "Quaternion.h"
#include "RotMat.h"

RotMat::RotMat() : Mat(0.0,3,3)
{
	for(int i=0;i<3;i++) v[i][i]=1.0;
}
RotMat::RotMat(const RotMat &original) : Mat(original){}

RotMat::RotMat(double const *values) : Mat(values,3,3){}

RotMat::RotMat(char const *string) : Mat(string,3,3){}

RotMat::RotMat(const Vec X, const Vec Y, const Vec Z) : Mat(0.0,3,3)
{
	setRefFrame(X,Y,Z);
}

RotMat::RotMat(const Mat &original) : Mat(0.0,3,3)
{
	if ( (original.nn==3) && (original.mm==3) )
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				v[i][j]=original[i][j];
			}
		}
	} 
}


//(Explicitely Inherited for preserving the output class label)
RotMat & RotMat::operator=(const double constant){return(*this=RotMat((Mat)*this=constant));}
RotMat & RotMat::operator+=(const RotMat &original){return(*this=RotMat((Mat)*this+=(Mat)original));}
RotMat & RotMat::operator-=(const RotMat &original){return(*this=RotMat((Mat)*this-=(Mat)original));}
RotMat & RotMat::operator*=(const double constant){return(*this=RotMat((Mat)*this*=constant));}
RotMat & RotMat::operator/=(const double constant){return(*this=RotMat((Mat)*this/=constant));}
RotMat RotMat::operator +(const RotMat &original)const {return (RotMat((Mat)*this+(Mat)original));}
RotMat RotMat::operator -(const RotMat &original)const {return (RotMat((Mat)*this-(Mat)original));}
RotMat RotMat::operator *(const RotMat &original)const {return (RotMat((Mat)*this*(Mat)original));}
Vec RotMat::operator *(const Vec &original)const {return ((Mat)*this*original);}	
RotMat RotMat::operator *(const double constant)const {return (RotMat((Mat)*this*constant));}
RotMat RotMat::operator /(const double constant)const {return (RotMat((Mat)*this/constant));}


void RotMat::setRefFrame(const Vec &X, const Vec &Y, const Vec &Z)
{
	setCol(0,X);
	setCol(1,Y);
	setCol(2,Z);
}

void RotMat::rotX(const double alfa)
{
	v[0][0]=1;
	v[1][0]=0;
	v[2][0]=0;
	v[0][1]=0;
	v[1][1]=cos(alfa);
	v[2][1]=sin(alfa);
	v[0][2]=0;
	v[1][2]=-sin(alfa);
	v[2][2]=cos(alfa);
}

void RotMat::rotY(const double alfa)
{
	v[0][0]=cos(alfa);
	v[1][0]=0;
	v[2][0]=-sin(alfa);
	v[0][1]=0;
	v[1][1]=1;
	v[2][1]=0;
	v[0][2]=sin(alfa);
	v[1][2]=0;
	v[2][2]=cos(alfa);
}

void RotMat::rotZ(const double alfa)
{
	v[0][0]=cos(alfa);
	v[1][0]=sin(alfa);
	v[2][0]=0;
	v[0][1]=-sin(alfa);
	v[1][1]=cos(alfa);
	v[2][1]=0;
	v[0][2]=0;
	v[1][2]=0;
	v[2][2]=1;
}

void RotMat::setAxisAngle(const Vec &vector,const  double alfa)
{
	Vec auxvec=vector;
	auxvec.normalize();

	double c,s,t1,t2;
	c = cos(alfa);
	s = sin(alfa);
	t1 = 1-c;
	t2 = auxvec[0]*t1;

	v[0][0]=t2*auxvec[0] + c;
	v[1][0]=t2*auxvec[1] + auxvec[2]*s;
	v[2][0]=t2*auxvec[2] - auxvec[1]*s;
	v[0][1]=t2*auxvec[1] - auxvec[2]*s;
	v[1][1]=auxvec[1]*auxvec[1]*t1 + c;
	v[2][1]=auxvec[1]*auxvec[2]*t1 + auxvec[0]*s;
	v[0][2]=t2*auxvec[2] + auxvec[1]*s;
	v[1][2]=auxvec[1]*auxvec[2]*t1 - auxvec[0]*s;
	v[2][2]=auxvec[2]*auxvec[2]*t1 + c;
}

double RotMat::getAngle() const 
{
	return(acos((v[0][0]+v[1][1]+v[2][2]-1)/2));
}

Vec RotMat::getAxis() const 
{
	Vec w=Vec(3);

	double alpha,s,t1;
	alpha=getAngle();
	s = sin(alpha);
	if (s != 0) {
		t1 = 1/(2*s);
		w[0] = (v[2][1]-v[1][2])*t1;
		w[1] = (v[0][2]-v[2][0])*t1;
		w[2] = (v[1][0]-v[0][1])*t1;
	}
	else {
		if (alpha == 0) {
			w[0] = 0;
			w[1] = 0;
			w[2] = 0;
		}
		else {
			w[0] = sqrt((v[0][0]+1)/2);
			w[1] = v[1][2]/v[0][2]*w[0];
			w[2] = v[2][1]/v[0][1]*w[0];
		}
	}

	return w;
}

Quaternion RotMat::getQuaternion() const 
{
  Quaternion q;
  double t0 = 1.0 + v[0][0] + v[1][1] + v[2][2];
  double t1 = 1.0 + v[0][0] - v[1][1] - v[2][2];
  double t2 = 1.0 - v[0][0] + v[1][1] - v[2][2];
  double t3 = 1.0 - v[0][0] - v[1][1] + v[2][2];

  if (t0 >= t1 && t0 >= t2 && t0 >= t3)
  {
    double r = sqrt(t0);
    double s = 0.5 / r;

    q[0] = 0.5 * r;
    q[1] = ( v[2][1] - v[1][2] ) * s;
    q[2] = ( v[0][2] - v[2][0] ) * s;
    q[3] = ( v[1][0] - v[0][1] ) * s;
  }
  else if (t1 >= t2 && t1 >= t3)
  {
    double r = sqrt(t1);
    double s = 0.5 / r;

    q[0] = (v[2][1] - v[1][2] ) * s;
    q[1] = 0.5 * r;
    q[2] = (v[0][1] + v[1][0] ) * s;
    q[3] = (v[0][2] + v[2][0] ) * s;
  }
  else if (t2 >= t3)
  {
    double r = sqrt(t2);
    double s = 0.5 / r;

    q[0] = (v[0][2] - v[2][0] ) * s;
    q[1] = (v[0][1] + v[1][0] ) * s;
    q[2] = 0.5 * r;
    q[3] = (v[1][2] + v[2][1] ) * s;
  }
  else
  {
    double r = sqrt(t3);
    double s = 0.5 / r;

    q[0] = (v[1][0] - v[0][1]) * s;
    q[1] = (v[0][2] + v[2][0] ) * s;
    q[2] = (v[1][2] + v[2][1] ) * s;
    q[3] = 0.5 * r;
  }

  return q;

  /*

     Quaternion q;
     double trace = v[0][0] + v[1][1] + v[2][2] + 1.0;
     if( (trace - 1.0) > TOLERANCE )
     {
     double s = 0.5 / sqrt(trace);
     q[0] = 0.25 / s;
     q[1] = ( v[2][1] - v[1][2] ) * s;
     q[2] = ( v[0][2] - v[2][0] ) * s;
     q[3] = ( v[1][0] - v[0][1] ) * s;
     }
     else
     {
     if ( v[0][0] > v[1][1] && v[0][0] > v[2][2] )
     {
     double s = 2.0 * sqrt( 1.0 + v[0][0] - v[1][1] - v[2][2]);
  //q[0] = (v[1][2] - v[2][1] ) / s;
  q[0] = (v[2][1] - v[1][2] ) / s;
  q[1] = 0.25 * s;
  q[2] = (v[0][1] + v[1][0] ) / s;
  q[3] = (v[0][2] + v[2][0] ) / s;
  }
  else if (v[1][1] > v[2][2])
  {
  double s = 2.0 * sqrt( 1.0 + v[1][1] - v[0][0] - v[2][2]);
  q[0] = (v[0][2] - v[2][0] ) / s;
  q[1] = (v[0][1] + v[1][0] ) / s;
  q[2] = 0.25 * s;
  q[3] = (v[1][2] + v[2][1] ) / s;
  }
  else
  {
  double s = 2.0 * sqrt( 1.0 + v[2][2] - v[0][0] - v[1][1] );
  //q[0] = (v[0][1] - v[1][0] ) / s;
  q[0] = (v[1][0] - v[0][1]) / s;
  q[1] = (v[0][2] + v[2][0] ) / s;
  q[2] = (v[1][2] + v[2][1] ) / s;
  q[3] = 0.25 * s;
  }
  }
  return q;
  */
}

RotMat RotMat::inv() const 
// Instead of using the standard inv() command from Mat, it computes the transpose
{
  return(this->transp());
}
