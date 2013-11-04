#if !defined(PI_INCLUDED)
#define PI 3.1415926535898
#define TOLERANCE 0.0000000001
#define DEG2RAD (PI / 180.0)
#define RAD2DEG (180.0 / PI)
#endif

#if !defined(VEC_INCLUDED)
#define VEC_INCLUDED

#include <iostream>


//#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define SIGN(a) (a > 0.0 ? 1 : (a<0.0 ? -1 : 0))
#define MAX(a,b) (a > b ? a:b)
#define MIN(a,b) (a < b ? a:b)

class Vec
{
public:
	int nn;	// size of array. upper index is nn-1
	double *v;
	int error;
	
	//Constructors
	Vec();
	Vec(const int n); 		// Zero-based array
	Vec(const double constant, const int n);	//Initialize to constant value
	Vec(double const * values, const int n);// Initialize to values in C-style array a	
	Vec(char const * string, const int n);  //Initialize to values in string
	Vec(const Vec &origin_vector);	// Copy constructor
	
	//Destructor
	~Vec();
	
	// Operators
	double & operator[](const int i) const;	//i'th element
	Vec & operator = (const Vec &original);	//assignment
	Vec & operator = (const double constant);	//assign a to every element
	Vec & operator +=(const Vec &original);
	Vec & operator -=(const Vec &original);
	Vec & operator *=(const double constant);
	Vec & operator /=(const double constant);
	Vec operator +(const Vec &original) const;
	Vec operator -(const Vec &original) const;
	Vec operator -() const;
	double operator *(const Vec &original) const; // Dot product.
	Vec operator *(const double constant) const;
	Vec operator /(const double constant) const;
	Vec operator +(const double constant) const;
	Vec operator -(const double constant) const;
	Vec operator ^(const Vec &original) const; //Cross Product. Only for 3-vectors

  friend std::ostream& operator<<(std::ostream& os, const Vec &orig);


	//Linear Algebra
	double norm() const;
	void normalize();
	double max() const;
	double min() const;
	int maxInd() const;
	int minInd() const;
	void randPerm();
	Vec abs() const;

	//Statistics
	double mean() const;
	double variance() const;
	double stdev() const;
};

#endif	// !defined(VEC_INCLUDED)
