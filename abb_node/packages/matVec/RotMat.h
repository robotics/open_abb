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
