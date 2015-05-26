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

namespace open_abb_driver
{

namespace matvec
{

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
#if !defined(MAT_INCLUDED)
#define MAT_INCLUDED

#include <iostream>

class Mat
{
public:
	int nn;     ///< Number of rows. Index range is 0..nn-1.
	int mm;     ///< Number of columns. Index range is 0..mm-1.
	double **v; ///< Storage of data.
	int error;  //Error type.

	// Constructors.
	Mat();
	Mat(const int n, const int m);
	Mat(const double constant, const int n, const int m);
	Mat(double const * values, const int n, const int m);
	Mat(char const * string, const int n, const int m);
	Mat(const Mat &origin_matrix);

	//Destructor
	~Mat();

	//Operators
	double* operator[](const int i) const;
	Mat& operator=(const Mat &original);
	Mat& operator=(const double constant);
	Mat& operator+=(const Mat &original);
	Mat& operator-=(const Mat &original);
	Mat operator +(const Mat &original) const;
	Mat operator -(const Mat &original) const;
	Mat operator *(const Mat &original) const;
	Vec operator *(const Vec &original) const;

	Mat operator -() const;

	Mat& operator+=(const double constant);
	Mat& operator-=(const double constant);
	Mat& operator*=(const double constant);
	Mat& operator/=(const double constant);
	Mat operator +(const double constant) const;
	Mat operator -(const double constant) const;
	Mat operator *(const double constant) const;
	Mat operator /(const double constant) const;

  friend std::ostream& operator<<(std::ostream& os, const Mat &orig);

	// LDU decomposition.
	Mat LDU(Vec &permutations, int &sign) const;
	void LDU(Mat &L, Mat &D, Mat &U, Mat &P) const;
	Vec LDUsolve(const Vec &b) const;
	Vec LDUsolve(const Mat &L, const Mat &D, const Mat &U, const Mat &P,const Vec &b) const;
	Mat LDUinverse() const;
	double LDUdet() const;
	Vec LSsolve(const Vec &b) const;

	//SVD decomposition.
	int SVD(Mat &U, Vec &sigma, Mat &V) const;
	
	//Linear Algebra
	Mat transp() const;
	Mat inv() const;
	double det() const;
	Vec getRow(const int n) const;
	Vec getCol(const int m) const;
	void setRow(const int n, const Vec &row);
	void setCol(const int m, const Vec &col);

	//Statistics
	double mean() const;
	double variance() const;
	double stdev() const;	

 private:
	double PYTHAG(double a, double b) const; //For SVD decomposition

};

#endif	// !defined(MAT_INCLUDED)
#if !defined(ROTMAT_INCLUDED)
#define ROTMAT_INCLUDED


class Quaternion;

class RotMat: public Mat
{
public:
	// Constructors
	RotMat();
	RotMat(double const *values);
	RotMat(char const *string);
	RotMat(const Vec X, const Vec Y, const Vec Z);
	RotMat(const RotMat &original); // Copy constructor.
	RotMat(const Mat &original); // Conversion constructor.

	//Operators
	//(Explicitely Inherited for preserving the output class label)
	RotMat& operator=(const double constant);
	RotMat& operator+=(const RotMat &original);
	RotMat& operator-=(const RotMat &original);
	RotMat& operator*=(const double constant);
	RotMat& operator/=(const double constant);
	RotMat operator +(const RotMat &original) const;
	RotMat operator -(const RotMat &original) const;
	RotMat operator *(const RotMat &original) const;
	Vec operator *(const Vec &original) const;
	RotMat operator *(const double constant) const;
	RotMat operator /(const double constant) const;

	void setRefFrame(const Vec &X, const Vec &Y, const Vec &Z);

	void rotX(const double alfa);
	void rotY(const double alfa);
	void rotZ(const double alfa);
	void setAxisAngle(const Vec &vector, const double alfa);

	//Transformations
	double getAngle() const;
	Vec getAxis() const;
	Quaternion getQuaternion() const;

	RotMat inv() const;	// Inverse computation redefinition.
};

#endif	// !defined(ROTMAT_INCLUDED)
#if !defined(QUATERNION_INCLUDED)
#define QUATERNION_INCLUDED


//class Vec;
class RotMat;

class Quaternion: public Vec
{
public:
	//Constructors
	Quaternion();
	Quaternion(const double constant);	//Initialize to constant value
	Quaternion(double const *values);// Initialize to values in C-style array a	
	Quaternion(char const *string);  //Initialize to values in string
  Quaternion(double const q0, Vec const v);  //Initialize with scalar and vector
	Quaternion(const Vec &origin_vector);	// Conversion constructor
	Quaternion(const Quaternion &origin_quaternion);  //Copy constructor

	//Operators
	//(Explicitely Inherited for preserving the output class label)
	Quaternion & operator =(const double constant);	//assign a to every element
	Quaternion & operator +=(const Quaternion &original);
	Quaternion & operator -=(const Quaternion &original);
	Quaternion operator +(const Quaternion &original) const;
	Quaternion operator -(const Quaternion &original) const;
	Quaternion operator -() const;
	Quaternion operator *(const double constant) const;
	Quaternion operator /(const double constant) const;
	//(New)
	Quaternion operator ^(const Quaternion &original) const; // Quaternion product.
	double operator *(const Quaternion &original) const;

	//Quaternion Algebra
	Quaternion conjugate() const;
	Quaternion inverse() const;
	double getScalar() const;
	Vec getVector() const;
	void setScalar(const double s);
	void setVector(const Vec &vector);
	Mat leftMat() const;
	Mat rightMat() const;

	//Transformation
	double getAngle() const;
	Vec getAxis() const;
	RotMat getRotMat() const;
};

#endif	// !defined(QUATERNION_INCLUDED)
#if !defined(HOMOGTRANSF_INCLUDED)
#define HOMOGTRANSF_INCLUDED

class HomogTransf : public Mat
{
public:
	//Constructors
	HomogTransf();	// Null translation and rotation.
	HomogTransf(double const *values);
	HomogTransf(char const *string);
	HomogTransf(const HomogTransf &original);		// Copy constructor.
	HomogTransf(const RotMat &rot, const Vec &trans);
	HomogTransf(const Mat &original); // Conversion constructor.
	
	//Operators
	//(Explicitely Inherited for preserving the output class label)
	HomogTransf& operator=(const double constant);
	HomogTransf& operator+=(const HomogTransf &original);
	HomogTransf& operator-=(const HomogTransf &original);
	HomogTransf& operator*=(const double constant);
	HomogTransf& operator/=(const double constant);
	HomogTransf operator +(const HomogTransf &original) const;
	HomogTransf operator -(const HomogTransf &original) const;
	HomogTransf operator *(const HomogTransf &original) const;
	HomogTransf operator *(const double constant) const;
	HomogTransf operator /(const double constant) const;
	//New Operator
	Vec operator *(const Vec &original) const;  //If original is a 4-vector: Standard product.
									//If original is a 3-vector: Transformation of a point in R3.
	//Interface
	void setTranslation(const Vec &trans);
	void setRotation(const RotMat &rot);
	RotMat getRotation() const;
	Vec getTranslation() const;

	//Transformations
	void setScrew(const Vec &point,const Vec &director_vector,const double displacement,const double angle);

	//Algebra
	HomogTransf inv() const;
	
};

#endif	// !defined(HOMOGTRANSF_INCLUDED)
#if !defined(POLYNOM_INCLUDED)
#define POLYNOM_INCLUDED

class Polynom : public Vec
{
public:

	//Constructors
	Polynom();
	Polynom(const int n); 		// Zero-based array
	Polynom(const double constant, const int n);	//Initialize to constant value
	Polynom(double const *values, const int n);// Initialize to values in C-style array a	
	Polynom(char const *string, const int n);  //Initialize to values in string
	Polynom(const Vec &origin_vector);	// Copy constructor
	Polynom(const Polynom &origin_polynom);  //Copy constructor

	//Operators
	//(Explicitely Inherited for preserving the output class label)
	Polynom & operator =(const double constant);	//assign a to every element
	Polynom operator -() const;
	Polynom operator *(const double constant);
	Polynom operator /(const double constant);
	//(New or redefined)
	Polynom operator *(const Polynom &original); // Polynom product.
	Polynom & operator +=(const Polynom &original);
	Polynom & operator -=(const Polynom &original);
	Polynom operator +(const Polynom &original); //Polynom Sum.
	Polynom operator -(const Polynom &original); //Polynom Sum.
	double operator()(const double x); //Evaluate Polynomial.
	
	int degree() const;
	void interpolate(const Vec &x, const Vec &y, const int n);
};

}

	
}

#endif	// !defined(POLYNOM_INCLUDED)
