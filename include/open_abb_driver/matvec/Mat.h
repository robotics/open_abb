#if !defined(MAT_INCLUDED)
#define MAT_INCLUDED

#include <iostream>
namespace open_abb_driver
{

namespace matvec
{
		
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

}

}

#endif	// !defined(MAT_INCLUDED)
