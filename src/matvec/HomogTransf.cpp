#include <math.h>

#include "open_abb_driver/matvec/Vec.h"
#include "open_abb_driver/matvec/Mat.h"
#include "open_abb_driver/matvec/RotMat.h"
#include "open_abb_driver/matvec/HomogTransf.h"

namespace open_abb_driver
{
	
namespace matvec
{

HomogTransf::HomogTransf() : Mat(0.0,4,4)
{
	for(int i=0;i<4;i++) v[i][i]=1.0;
}

HomogTransf::HomogTransf(const HomogTransf &original) : Mat(original){}

HomogTransf::HomogTransf(const double *values) : Mat(values,4,4){}

HomogTransf::HomogTransf(const char *string) : Mat(string,4,4){}

HomogTransf::HomogTransf(const RotMat &rot, const Vec &trans) : Mat(4,4)
{
	this->setRotation(rot);
	this->setTranslation(trans);
	v[3][0]=0;
	v[3][1]=0;
	v[3][2]=0;
	v[3][3]=1;
}

HomogTransf::HomogTransf(const Mat &original) : Mat(4,4)
{
	if ( (original.nn==4) && (original.mm==4) )
	{
		for(int i=0;i<4;i++) {
			for(int j=0;j<4;j++) {
				v[i][j]=original[i][j];
			}
		}
	} 
}

HomogTransf & HomogTransf::operator=(const double constant){return(*this=HomogTransf((Mat)*this=constant));}
HomogTransf & HomogTransf::operator+=(const HomogTransf &original){return(*this=HomogTransf((Mat)*this+=(Mat)original));}
HomogTransf & HomogTransf::operator-=(const HomogTransf &original){return(*this=HomogTransf((Mat)*this-=(Mat)original));}
HomogTransf & HomogTransf::operator*=(const double constant){return(*this=HomogTransf((Mat)*this*=constant));}
HomogTransf & HomogTransf::operator/=(const double constant){return(*this=HomogTransf((Mat)*this/=constant));}
HomogTransf HomogTransf::operator +(const HomogTransf &original)const {return (HomogTransf((Mat)*this+(Mat)original));}
HomogTransf HomogTransf::operator -(const HomogTransf &original)const {return (HomogTransf((Mat)*this-(Mat)original));}
HomogTransf HomogTransf::operator *(const HomogTransf &original)const {return (HomogTransf((Mat)*this*(Mat)original));}
HomogTransf HomogTransf::operator *(const double constant)const {return (HomogTransf((Mat)*this*constant));}
HomogTransf HomogTransf::operator /(const double constant)const {return (HomogTransf((Mat)*this/constant));}

Vec HomogTransf::operator *(const Vec &original)const 
{
	if (original.nn==3)
	{
		Vec w(0.0,3);

		if(original.nn==3)
		{
			int i,j;
			for(i=0;i<3;i++)
			{
				for(j=0;j<3;j++) w[i]+=v[i][j]*original[j];
				w[i]+=v[i][3];
			}
		}
		return w;
	}
	else
		return (((Mat)(*this))*original);
}

void HomogTransf::setRotation(const RotMat &rot)
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++) v[i][j]=rot[i][j];
}

void HomogTransf::setTranslation(const Vec &trans)
{
	v[0][3]=trans[0];
	v[1][3]=trans[1];
	v[2][3]=trans[2];
}

RotMat HomogTransf::getRotation() const
{
	RotMat r;
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			r[i][j] = v[i][j];
	return r;
}

Vec HomogTransf::getTranslation() const
{
	Vec trans(3);
	trans[0] = v[0][3];
	trans[1] = v[1][3];
	trans[2] = v[2][3];
	return trans;
}


void HomogTransf::setScrew(const Vec &point, const Vec &director_vector, const double displacement, const double angle)
{
	double ca,sa,va;

	ca = cos(angle); sa = sin (angle);va = 1-cos(angle);

	v[0][0] = director_vector[0]*director_vector[0]*va+ca;		v[0][1] = director_vector[0]*director_vector[1]*va-director_vector[2]*sa;	v[0][2] = director_vector[0]*director_vector[2]*va+director_vector[1]*sa;
	v[1][0] = director_vector[0]*director_vector[1]*va+director_vector[2]*sa;	v[1][1] = director_vector[1]*director_vector[1]*va+ca;		v[1][2] = director_vector[1]*director_vector[2]*va-director_vector[0]*sa;
	v[2][0] = director_vector[0]*director_vector[2]*va-director_vector[1]*sa;	v[2][1] = director_vector[1]*director_vector[2]*va+director_vector[0]*sa; v[2][2] = director_vector[2]*director_vector[2]*va+ca; 
	v[3][0] = 0.0;					v[3][1] = 0.0;					v[3][2] = 0.0;

    v[0][3] =  displacement*director_vector[0]-point[0]*(v[0][0]-1)-point[1]*v[0][1]-point[2]*v[0][2];
    v[1][3] =  displacement*director_vector[1]-point[0]*v[1][0]-point[1]*(v[1][1]-1)-point[2]*v[1][2];
    v[2][3] =  displacement*director_vector[2]-point[0]*v[2][0]-point[1]*v[2][1]-point[2]*(v[2][2]-1);
	v[3][3] = 1.0;
}

HomogTransf HomogTransf::inv() const 
{
	HomogTransf h;

	h[0][0] = v[0][0];	h[0][1] = v[1][0];	h[0][2] = v[2][0]; 
	h[1][0] = v[0][1];	h[1][1] = v[1][1];	h[1][2] = v[2][1]; 
	h[2][0] = v[0][2];	h[2][1] = v[1][2];	h[2][2] = v[2][2]; 

	h[0][3] =  - (v[0][3]*v[0][0]) - (v[1][3]*v[1][0]) - (v[2][3]*v[2][0]);
	h[1][3] =  - (v[0][3]*v[0][1]) - (v[1][3]*v[1][1]) - (v[2][3]*v[2][1]);
	h[2][3] =  - (v[0][3]*v[0][2]) - (v[1][3]*v[1][2]) - (v[2][3]*v[2][2]);

	h[3][0] = 0.0;		h[3][1] = 0.0;		h[3][2] = 0.0;     h[3][3] = 1.0;

	return h;
}

}

}
