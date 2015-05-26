#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "open_abb_driver/matvec/Vec.h"
//#include "Mat.h"

namespace open_abb_driver
{
	
namespace matvec
{
	
Vec::Vec()
{
	nn=0;
	v=0;
}

Vec::Vec(const int n)
{
	if(n>=0)
	{
		nn = n;
		v = new double[n];
	}
	else{
		Vec();
	}
}

/** \brief Constructor. 
           Initializes the vector to a constant value.
  * Constructor that initializes the vector to a constant value a.
  * @param constant initialization value.
  * @param n length of the vector.
  * @see Vec(const int n). 
  */
Vec::Vec(const double constant, const int n)
{
	if(n>=0){
		nn = n;
		v = new double[n];
		int i;
		for(i=0; i<n; i++)
			v[i] = constant;
	}
	else{
		Vec();
	}
}

Vec::Vec(double const * values, const int n) 
{
	if (n>=0)
	{
		nn = n;
		v = new double[n];
		int i;
	 	for(i=0; i<n; i++)
		  {
			v[i] = *values;
			values++;
		  }
	}
	else Vec();
}

Vec::Vec(char const * string, const int n)
{
	if (n>0)
	{
		int i,j,k,error;
		char aux[30];	// Stores one of the floats in string.
		for (i=0;i<30;i++)
			aux[i]=0;

		nn = n;
		v = new double[n];
	
		i=0;	// Index of the char array string.
		j=0;	// Index of the char array aux.
		k=0;	// Index of vector.
	
		// Getting new elements of c until null-character.
		while (string[i]!=0) 
		{
			if (string[i] != ' ')
			{
				// Add chars to the aux string until a space is found.
				aux[j]=string[i];
				j++;
			}
			else 
			{
				// Convert the string aux to a double, and put it into the matrix.
				error = sscanf(aux,"%lf",&v[k]);
				if (error > 0)
				{
					// Conversion ok.
					if (k==nn-1)
					{
						// More reals in string than length of vector vector. Vector filled.
						return;
					}
					// Go to next element of the matrix.
					k++;
				}
				// Reset aux.
				j=0;
				while ( (aux[j] != 0) && (j<30) )
				{
					aux[j] = 0;
					j++;
				}
				j=0;
			}
			i++;	// Next element of string.
		}
		if (string[i-1]!=0) 
		{
			// Do the last conversion if necessary.
			sscanf(aux,"%lf",&v[k]);
		}
	}
	else Vec();
}

Vec::Vec(const Vec &origin_vector)
{
	nn = origin_vector.nn;
	v = new double[nn];
	int i;
	for(i=0; i<nn; i++)
		v[i] = origin_vector.v[i];
}

//Operators

double & Vec::operator[](const int i) const
{
	if((i>=0)&&(i<nn))
		return v[i];
	else
	{
	  double * a;
	  a=0;
	  	return *a;
	  //return 0;
	}
}


Vec & Vec::operator = (const Vec &original)
//		if vector and original were different sizes, vector
//		is resized to match the size of original
{
	if(this != &original)
	{
		if (nn != original.nn) {
			if (v != 0) delete [] (v);
			nn=original.nn;
			v = new double[nn];
		}
		int i;
		for (i=0; i<nn; i++)
			v[i]=original.v[i];
	}
	return *this;
}

Vec & Vec::operator = (const double constant)	//assign a to every element
{
	int i;
	for (i=0; i<nn; i++)
		v[i]=constant;
	return *this;
}

Vec & Vec::operator +=(const Vec &original)
{
	if (this != &original)
	{
		if (nn == original.nn) {
			int i;
			for (i=0; i<nn; i++)
			v[i] += original[i];
		}
	}
	return *this;
}

Vec & Vec::operator -=(const Vec &original)
{
	if (this != &original)
	{
		if (nn == original.nn){
			int i;
			for (i=0; i<nn; i++)
				v[i] -= original[i];
		}
	}
	return *this;
}

Vec & Vec::operator *=(const double constant)
{
	int i;
	for (i=0; i<nn; i++)
		v[i] *= constant;
	return *this;
}

Vec & Vec::operator /=(const double constant)
{
	int i;
	for (i=0; i<nn; i++)
		v[i] /= constant;
	return *this;
}

Vec Vec::operator +(const Vec &original) const
{
	if (nn == original.nn) {
		Vec w(nn);
		int i;
		for (i=0;i<nn;i++)
			w[i]=v[i]+original[i];
		return w;
	}
	Vec w(0);
	return w;
}

Vec Vec::operator -(const Vec &original) const
{
	if (nn == original.nn) {
		Vec w(nn);
		int i;
		for (i=0;i<nn;i++)
			w[i]=v[i]-original[i];
		return w;
	}
	Vec w(0);
	return w;
}

Vec Vec::operator -() const
{
	Vec w(nn);
	int i;
	for (i=0;i<nn;i++)
		w[i]=-v[i];
	return w;
}

Vec Vec::operator *(const double constant) const
{
	Vec w(nn);
	int i;
	for (i=0;i<nn;i++)
		w[i]=constant*v[i];
	return w;
}

Vec Vec::operator /(const double constant) const
{
	if (constant!=0)
	{
		Vec w(nn);
		int i;
		for (i=0;i<nn;i++)
			w[i]=v[i]/constant;
		return w;
	}
	Vec w(0);
	return w;
}

Vec Vec::operator +(const double constant) const
{
	Vec w(nn);
	int i;
	for (i=0;i<nn;i++)
		w[i]=constant+v[i];
	return w;
}

Vec Vec::operator -(const double constant) const
{
	Vec w(nn);
	int i;
	for (i=0;i<nn;i++)
		w[i]=v[i]-constant;
	return w;
}

double Vec::operator *(const Vec &original) const
{
	double a=0;
	if (nn == original.nn)
	{
		int i;
		for(i=0;i<nn;i++)
			a += v[i]*original[i];
	}
	return a;
}

Vec Vec::operator ^(const Vec &original) const
{
	Vec w(0.0,3);
	if ((nn == 3)&&(original.nn==3))
	{
		w[0] = v[1]*original[2]-v[2]*original[1];
		w[1] = v[2]*original[0]-v[0]*original[2];
		w[2] = v[0]*original[1]-v[1]*original[0];
	}
	return w;
}


std::ostream& operator<<(std::ostream &os, const Vec &orig)
{
  int i;

  os << "[ ";
  for (i=0; i<orig.nn;i++)
  {
    os << orig[i] << " ";
  }
  os << "]";

  return os;
}


double Vec::norm() const
{
	Vec w(*this);
	return(sqrt(w*w));
}

void Vec::normalize()
{
	double n;
	int i;
	n=norm();
	for(i=0;i<nn;i++)
		v[i]/=n;
}

double Vec::max() const
{
	double aux=-1e100;
	double *ve;
	ve=&v[0];
	for (int i=0; i<nn; i++)
	{
		if(*ve>aux)
			aux=*ve;
		ve++;
	}
	return aux;
}

double Vec::min() const
{
	double aux=1e100;
	double *ve;
	ve=&v[0];
	for (int i=0; i<nn; i++)
	{
		if(*ve<aux)
			aux=*ve;
		ve++;
	}
	return aux;
}

int Vec::maxInd() const
{
	double aux=-1e100;
	double *ve;
	ve=&v[0];
	int maxInd=0;
	for (int i=0; i<nn; i++)
	{
		if(*ve>aux)
		{
			maxInd=i;
			aux=*ve;
		}
		ve++;
	}
	return maxInd;
}

int Vec::minInd() const
{
	double aux=1e100;
	double *ve;
	ve=&v[0];
	int minInd=0;
	for (int i=0; i<nn; i++)
	{
		if(*ve<aux)
		{
			minInd=i;
			aux=*ve;
		}
		ve++;
	}
	return minInd;
}

double Vec::mean() const
{
  double *vpointer = &v[0];
  double aux=0.0;

  for (int i=0; i<nn; i++)
    {
      aux+=*vpointer;
      vpointer++;
    }
  return (aux/(double)nn);
}

double Vec::variance() const
{
  if(nn<=1)
    return 0;
  else
    {
      double *vpointer = &v[0];
      double aux=0;
      double m=mean();
      for (int i=0; i<nn; i++)
	{
	  aux+=(*vpointer - m) * (*vpointer - m);
	  vpointer++;
	}
      return (aux/(double)(nn-1));
    }
}

double Vec::stdev() const
{
  return(sqrt(variance()));
}

void Vec::randPerm()
{
  int n;
  int k;
  int tmp;
  
  for (int n=0; n<nn;n++)
    v[n]=n;

  n=nn;
  while (n > 1)
    {
        // Swap a random unshuffled card with the top-most card
        k = rand() % n;
        n--;
        tmp = v[n];
        v[n] = v[k];
        v[k] = tmp;
    }
}

Vec Vec::abs() const
{
  Vec w(nn);
  double *vpointer = &v[0];
  double *wpointer = &w[0];

  for (int i=0; i<nn; i++)
    {
      *wpointer = fabs(*vpointer);
      vpointer++;
      wpointer++;
    }
  return (w);
}

//Destructor
Vec::~Vec(void)
{
	if (v != 0)
		delete[] (v);
}

}

}