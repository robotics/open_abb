#include <math.h>

#include "Vec.h"
#include "Mat.h"
#include "Polynom.h"

Polynom::Polynom() : Vec() {}

Polynom::Polynom(const int n) : Vec(n+1) {} 	

Polynom::Polynom(const double constant, const int n) : Vec(constant,n+1) {}

Polynom::Polynom(double const *values, const int n) : Vec(values,n+1) {}

Polynom::Polynom(char const *string, const int n) : Vec(string,n+1) {}

Polynom::Polynom(const Vec &origin_vector) : Vec(origin_vector) {}

Polynom::Polynom(const Polynom &origin_polynom) : Vec((Vec)origin_polynom) {}

int Polynom::degree() const
{
	return(nn-1);
}

Polynom & Polynom::operator =(const double constant){return(*this=Polynom((Vec)*this=constant));}
Polynom Polynom::operator -() const {return(Polynom(-(Vec)*this));}
Polynom Polynom::operator *(const double constant){return (Polynom((Vec)*this*constant));}
Polynom Polynom::operator /(const double constant){return (Polynom((Vec)*this/constant));}

Polynom	Polynom::operator +(const Polynom &original)
{
	if(nn>original.nn)
	{
		Polynom w(*this);
		int i;
		for(i=0;i<original.nn;i++)
			w[i]+=original[i];
		return w;
	}
	else
	{
		Polynom w(original);
		int i;
		for(i=0;i<nn;i++)
			w[i]+=v[i];
		return w;
	}
}

Polynom & Polynom::operator +=(const Polynom &original)
{
	return(*this=(*this+original));
}

Polynom	Polynom::operator -(const Polynom &original)
{
	if(nn>original.nn)
	{
		Polynom w(*this);
		int i;
		for(i=0;i<original.nn;i++)
			w[i]-=original[i];
		return w;
	}
	else
	{
		Polynom w(-original);
		int i;
		for(i=0;i<nn;i++)
			w[i]+=v[i];
		return w;
	}
}

Polynom & Polynom::operator -=(const Polynom &original)
{
	return(*this=(*this-original));
}

Polynom Polynom::operator *(const Polynom &original)
{
	Polynom w(0.0,degree() + original.degree());
	int i,j;
	for(i=0;i<nn;i++)
		for(j=0;j<original.nn;j++)
			w[i+j]+=v[i]*original[j];
	return w;
}

double Polynom::operator()(const double x)
{
	int i;
	double w=v[0];
	for(i=1;i<nn;i++)
		w+=(v[i]*pow(x,i));
	return w;
}

void Polynom::interpolate(const Vec &x, const Vec &y, const int n)
{
	if((x.nn==n) && (y.nn==n))
	{
		int i,j,k;

		//First we compute the divided differences and we store them in the upper-left
		//triangular side of the matrix A
		Mat A(n,n);
		for(i=0;i<n;i++)
			A[i][0]=y[i];
		for(j=1;j<n;j++)
		{
			k=n-j;
			for(i=0;i<k;i++)
				A[i][j]=(A[i+1][j-1]-A[i][j-1])/(x[i+j] - x[i]);
		}

		//Now we construct the polynomial
		Polynom p(0);             //Interpolating polynomial (degree 0 at the begining of
		                          //the iteration)
		Polynom aux(1.0,0);           //Auxiliar polynomial
		Polynom monomial(1.0,1);      //Monomial (x-x_i)
		p[0]=A[0][0];
		for(i=1; i<n; i++)
		{
			monomial[0]=-x[i-1];  
			aux=aux*monomial;     //We compute the polynom (x-x_0)...(x-x_i)
			p = p + aux*A[0][i];  //We actualize the interpolating polynomial
		}
		*this=p;
	}
} 

