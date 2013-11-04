#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Vec.h"
#include "Mat.h"


/** \brief Default constructor.
 *         Zero-size matrix.
 *
 *  This constructor creates an empty matrix. No memory space is allocated, so
 *	accessing any element will cause an error.
 */
Mat::Mat()
{
	nn=0;
	mm=0;
	v=0;
}

/** \brief Constructs an NxM matrix.
 *	\param n number of rows.
 *	\param m number of columns.
 *
 *  This constructor creates a matrix of n rows by m columns. The matrix's elements
 *	are not initialized.
 */
Mat::Mat(const int n, const int m)
{
	if ((n>0) && (m>0))
	{
		nn = n;
		mm = m;
		v = new double*[n];
		v[0] = new double[m*n];
		for (int i=1; i<n; i++)
			v[i] = v[i-1] + m;
	}
	else Mat();
}

/** \brief Constructs an NxM matrix and initializes its elements to a constant value a.
 *	\param a initial value os each matrix element.
 *	\param n number of rows.
 *	\param m number of columns.
 *	\exception InvalidArgument one or both of the parameters are negative or zero.
 *
 *  This constructor creates a matrix of n rows by m columns, and sets all
 *	matrix's elements to a.
 */
Mat::Mat(const double constant, const int n, const int m)
{
	if ((n>0) && (m>0))
	{
		int i,j;
		nn = n;
		mm = m;
		v = new double*[n];
		v[0] = new double[m*n];
		for (i=1; i<n; i++)
			v[i] = v[i-1] + m;
		for (i=0; i<n; i++)
			for (j=0; j<m; j++)
				v[i][j] = constant;
	}
	else Mat();
}

/** \brief Constructs an NxM matrix from a C-style array a.
 *	\param a pointer to a double array with the matrix's elements value.
 *	\param n number of rows.
 *	\param m number of columns.
 *
 *  This constructor creates a matrix of n rows by m columns. The first m components
 *	of a are used to initialize the first row, the next m components are used to
 *	initialize the second row, ...
 */
Mat::Mat(double const * values, const int n, const int m)
{
	if ((n>0) && (m>0))
	{
		int i,j;
		nn = n;
		mm = m;
		v = new double*[n];
		v[0] = new double[m*n];
		for (i=1; i<n; i++)
			v[i] = v[i-1] + m;
		for (i=0; i<n; i++)
			for (j=0; j<m; j++)
			{
			  v[i][j] = *values;
			  values++;
			}
	}
	else Mat();
}

/** \brief Constructs an NxM matrix from a C-style string a.
 *	\param a pointer to a char array with the matrix's elements value.
 *	\param n number of rows.
 *	\param m number of columns.
 *
 *  This constructor creates a matrix of n rows by m columns. Values from the string
 *	are read until the null-character is reached or all matrix's elements are
 *	initialized. The fist m values read are used to
 *	initialize the fist row, the next m initialize the second row,...
 */
Mat::Mat(char const * string, const int n, const int m)
{
	if ((n>0) && (m>0))
	{
		int i,j,k,l,error;
		char aux[30];	// Stores one of the floats in string.

		nn = n;
		mm = m;
		v = new double*[n];
		v[0] = new double[m*n];
		for (i=1;i<n;i++)
			v[i] = v[i-1] + m;

		i=0;	// Index of the char array string.
		j=0;	// Index of the char array aux.
		k=0;	// Row index of v.
		l=0;	// Column index of v.

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
				error = sscanf(aux,"%lf",&v[k][l]);
				if (error > 0)
				{
					// Conversion ok.
					if ( (k==nn-1) && (l==mm-1) )
					{
						// More reals in string than element in matrix. Matrix filled.
						return;
					}
					// Go to next element of the matrix.
					l++;
					if (l==m)
					{
						l=0;
						k++;
					}
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
			sscanf(aux,"%lf",&v[k][l]);
		}
	}
	else Mat();
}

/** \brief Copy constructor.
 *	\param rhs matrix to be copied to the new one.
 *
 *  This constructor creates a copy of rhs. 
 */
Mat::Mat(const Mat &origin_matrix)
{
	nn=origin_matrix.nn;
	mm=origin_matrix.mm;
	v=new double*[nn];
	v[0] = new double[mm*nn];
	int i,j;
	for (i=1; i<nn; i++)
		v[i] = v[i-1] + mm;
	for (i=0; i<nn; i++)
		for (j=0; j<mm; j++)
			v[i][j] = origin_matrix.v[i][j];
}

/** \brief Destructor.
 *
 *  The destructor deallocates all memory space used by the object.
 */
Mat::~Mat()
{
	if (v != 0) {
		delete[] (v[0]);
		delete[] (v);
	}
}

/** \brief Subscripting operator [].
*	\param i number of row accessed.
 *	\return a pointer to row i.
 *	\exception InvalidArgument i is not a valid row number (i<0 or i>=number of rows).
 *
 *  Method that gives access to any induvidual element of the matrix. The following
 *	example shows how to change tha value of an individual element of a matrix:
 *	\code
 *	Mat_DP m("1 2 3 4", 2, 2);
 *	m[0][1] = 25; \endcode
 */
double* Mat::operator [](const int i) const
{
	if ((i>=0) && (i<nn))
		return v[i];
	else
		return 0;
}


/** \brief Copy assignment operator (cleanup and copy).
 *	\param rhs matrix to be copied
 *	\return reference to the new matrix.
 *	\post normal assignment via copying has ben performed;
 *	if matrix and rhs were diferent sizes, matrix has been resized
 *	to match the size of rhs.
 */
Mat& Mat::operator =(const Mat &original)
{
	if (this != &original)
	{
		int i,j;
		if (nn != original.nn || mm != original.mm)
		{
			if (v != 0) 
			{
				delete[] (v[0]);
				delete[] (v);
			}
			nn = original.nn;
			mm = original.mm;
			v = new double*[nn];
			v[0] = new double[mm*nn];
			for (i=1; i<nn; i++)
				v[i] = v[i-1] + mm;
		}
		for (i=0; i<nn; i++)
			for (j=0; j<mm; j++)
				v[i][j] = original[i][j];
	}
	return *this;
}

/** \brief Assign a to every element.
 *	\param a value to be assigned to every element.
 *	\return reference to the modified matrix.
 */
Mat& Mat::operator =(const double constant)
{
	for (int i=0; i<nn; i++)
		for (int j=0; j<mm; j++)
			v[i][j] = constant;
	return *this;
}

/** \brief Composite addition-assignment operator.
 *	\param rhs matrix to be added.
 *	\return reference to the modified matrix.
 *	
 *	Adds the matrix and rhs, and stores the result into the matrix.
 */
Mat& Mat::operator+=(const Mat &original)
{
	if ( (nn = original.nn) && (mm = original.mm) )
	{
		for (int i=0;i<nn;i++)
			for (int j=0;j<mm;j++)
				v[i][j] += original[i][j]; 
	}

	return *this;
}

/** \brief Composite subtraction-assignment operator.
 *	\param rhs matrix to be subtracted.
 *	\return reference to the modified matrix.
 *	
 *	Subtracts rhs to the matrix, and stores the result into the matrix.
 */
Mat& Mat::operator-=(const Mat &original)
{
	if ( (nn = original.nn) && (mm = original.mm) )
	{
		for (int i=0;i<nn;i++)
			for (int j=0;j<mm;j++) 
				v[i][j] -= original[i][j]; 
	}

	return *this;
}

/** \brief Composite constant value addition-assignment operator.
 *	\param constant value to be added to the matrix
 *	\return reference to the modified matrix.
 *	
 *	Adds the constant to each element, and stores the result into the matrix.
 */
Mat& Mat::operator+=(const double constant)
{
	for (int i=0; i<nn; i++)
		for (int j=0; j<mm; j++) 
			v[i][j] += constant;
	return *this;
}

/** \brief Composite constant value subtraction-assignment operator.
 *	\param constant value to be subtracted from the matrix
 *	\return reference to the modified matrix.
 *	
 *	Subtracts the constant to each element, and stores the result into the matrix.
 */
Mat& Mat::operator-=(const double constant)
{
	for (int i=0; i<nn; i++)
		for (int j=0; j<mm; j++) 
			v[i][j] -= constant;
	return *this;
}

/** \brief Composite constant value product-assignment operator.
 *	\param a value by which the matrix will be multiplied.
 *	\return reference to the modified matrix.
 *	
 *	Multiplies each element by a, and stores the result into the matrix.
 */
Mat& Mat::operator*=(const double constant)
{
	for (int i=0; i<nn; i++)
		for (int j=0; j<mm; j++) 
			v[i][j] *= constant;
	return *this;
}

/** \brief Composite constant value division-assignment operator.
 *	\param a value by which the matrix will be divided.
 *	\return reference to the modified matrix.
 *	\exception ZeroDivide thrown if a is 0.
 *	
 *	Divides each element by a and stores the result into the matrix.
 */
Mat& Mat::operator/=(const double constant)
{
	if (constant != 0)
	{
		for (int i=0; i<nn; i++)
			for (int j=0; j<mm; j++) 
				v[i][j] /= constant;
	}
	return *this;
}


/** \brief Matrix addition operator.
 *	\param rhs matrix to be added.
 *	\return the matrix resultant from the addition.
 *	\exception InvalidSize thrown when the matrices have different sizes.
 *	
 *	Adds two matrices and returns the resultant matrix. Both matrices must have
 *	the same number columns and rows.
 */
Mat Mat::operator+(const Mat &original) const
{
	Mat w(*this);
	w += original;
	return w;
}

/** \brief Matrix subtract operator.
 *	\param rhs matrix to subtract.
 *	\return the matrix resultant from the subtraction.
 *	\exception InvalidSize thrown when the matrices have different sizes.
 *	
 *	Subtracts two matrices and returns the resultant matrix. Both matrices must
 *	have the same number columns and rows.
 */
Mat Mat::operator-(const Mat &original) const
{
	Mat w(*this);
	w -= original;
	return w;
}

/** \brief Matrix negator operator.
 *	\return the matrix resultant from the negation.
 *	
 *	Negates the matrix and returns the resultant matrix. 
 */
Mat Mat::operator-() const
{
        Mat w(nn, mm);
	for (int i=0; i<nn; i++)
	  for(int j=0; j<mm; j++)
	    w[i][j]=-v[i][j];
	return w;
}

/** \brief Matrix product operator.
 *	\param rhs matrix to multiply.
 *	\return the matrix resultant from the product.
 *	\exception InvalidSize thrown when rhs's number of rows is different from the
 *	matrix's number of columns.
 *	
 *	Multiplies two matrices and returns the resultant matrix. The number of columns
 *	the first matrix must be the same as the number of rows of the second one.
 */
Mat Mat::operator*(const Mat &original) const
{
	if (mm == original.nn)
	{
		Mat w(0.0,nn,original.mm);
		int i,j,k;
		for (i=0; i<w.nn; i++)
			for (j=0; j<w.mm; j++)
				for (k=0; k<mm; k++) 
					w[i][j] += v[i][k]*original[k][j];
		return w;
	}
	else
	{
		Mat r;
		return r;
	}
}

/** \brief Vector product operator.
 *	\param rhs vector to multiply.
 *	\return the vector resultant from the product.
 *	\exception InvalidSize thrown when the vector's size is different from the
 *	matrix's number of columns.
 *	
 *	Multiplies a matrix by a vector and returns the resultant vector. The number of
 *	columns of the matrix must be the same as the vector's size.
 */
Vec Mat::operator*(const Vec &original) const
{
	Vec w;

	if (mm == original.nn)
	{
		w = Vec(0.0,nn);
		int i,j;
		for(i=0;i<nn;i++)
			for(j=0;j<mm;j++)
				w[i]+=v[i][j]*original[j]; 
	}
	return w;
}

/** \brief Constant value addition operator.
 *	\param a value to add to the matrix.
 *	\return the resultant matrix.
 *	
 *	Adds a constant to each element of the matrix and returns the
 *	resultant matrix.
 */
Mat Mat::operator +(const double constant) const
{
	Mat w(*this);

	return w += constant;
}

/** \brief Constant value subtraction operator.
 *	\param a value to subtract to the matrix.
 *	\return the resultant matrix.
 *	
 *	Subtracts a constant to each element of the matrix and returns the
 *	resultant matrix.
 */
Mat Mat::operator -(const double constant) const
{
	Mat w(*this);

	return w -= constant;
}

/** \brief Constant value product operator.
 *	\param a value to multiply the matrix.
 *	\return the matrix resultant from the product.
 *	
 *	Multiplies each element of the matrix by a constant value and returns the
 *	resultant matrix.
 */
Mat Mat::operator *(const double constant) const
{
	Mat w(*this);

	return w *= constant;
}

/** \brief Constant value division operator.
 *	\param rhs value to divide the matrix.
 *	\return the matrix resultant from the division.
 *	\exception ZeroDivide thrown when a is 0.
 *	
 *	Divides each element of the matrix by a constant value and returns the
 *	resultant matrix. This value must be different from 0.
 */
Mat Mat::operator /(const double constant) const
{
	Mat w(*this);
	w /= constant;
	return w;
}


std::ostream& operator<<(std::ostream &os, const Mat &orig)
{
  int i,j;

  os << "[ ";
  for (i=0; i<orig.nn;i++)
  {
    os << "[ ";
    for (j=0; j<orig.mm;j++)
    {
      os << orig[i][j] << " ";
    }
    os << "] ";
  }
  os << "]";

  return os;
}

Mat Mat::LDU(Vec &permutations, int &sign) const
{	
	int i,j,k,l,m,pivot;
	double max_pivot, aux;
	permutations= Vec(nn);
	for(i=0;i<nn;i++)
		permutations[i]=i;
	Mat w(*this);
	
	//We begin the loop
	sign=1;
	for(i=0;i<nn-1;i++)
	{
		//First we look for the maximum pivoting element in the given column
		max_pivot=fabs(w[i][i]);
		pivot=i;
		for(k=i+1;k<nn;k++)
		{
			if(fabs(w[k][i])>max_pivot)
			{
				max_pivot=fabs(w[k][i]);
				pivot=k;
			}
		}
		
		//If we have found a greater pivote that the original we make the permutation
		if(pivot!=i)
		{
			sign=sign*(-1);
			//We have to interchange rows i and pivot of matrix A
			for(j=0; j<mm; j++)
			{
				aux = w[i][j];
				w[i][j]=w[pivot][j];
				w[pivot][j]=aux;
			}
			//Interchange the indexes in the permutation vector
			m=(int)permutations[pivot];
			permutations[pivot]=permutations[i];
			permutations[i]=m;
		}
		//We operate in order to make column i of matrix U to have zeros below the diagonal. 
		//Besides we compute the elements of matrix L and store them in the same matrix A. 
		if(w[i][i]==0)
			break;
		else
		{
			for(l=i+1;l<nn;l++)
			{
				w[l][i]=w[l][i]/w[i][i];
				for(j=i+1;j<mm;j++)
				{
					w[l][j]=w[l][j] - w[l][i]*w[i][j];
				}
			}
		}
	}
	return w;
}

void Mat::LDU(Mat &L, Mat &D, Mat &U, Mat &P) const
{
	int i,j, sign;
	Vec perm;
	Mat w;
	sign=0;
	w=LDU(perm, sign);
	L=Mat(0.0,nn,nn);
	D=Mat(0.0,nn,nn);
	U=Mat(0.0,nn,mm);
	P=Mat(0.0,nn,nn);

	//We get L from the values of w
	for(i=0;i<w.nn;i++)
	{
		for(j=0;(j<i)&&(j<w.mm);j++)
			L[i][j]=w[i][j];
		L[i][i]=1;
	}
	
	//We get D and U from the values of w
	for(i=0;(i<w.nn)&&(i<w.mm);i++)
	{
		D[i][i]=w[i][i];
		U[i][i]=1;
		for(j=i+1;j<w.mm;j++)
			U[i][j]=w[i][j]/D[i][i];
	}

	//We construct P from the vector perm
	for(i=0;i<nn;i++)
		P[i][(int)perm[i]]=1;
}

Vec Mat::LDUsolve(const Vec &b) const
{
	Vec x;
	if((nn==mm)&&(nn==b.nn))
	{
		int i,j;
		Mat L,D,U,P;
		LDU(L,D,U,P);
		Vec b2=P*b;
		
		x=Vec(0.0,mm);
		Vec y(0.0,nn);
		
		y[0] = b2[0];
		for (i = 1 ; i < nn ; i++)
		{
			y[i] = b2[i];
			for(j = 0 ; j < i ; j++)
				y[i] = y[i] - L[i][j]*y[j];
		}
		
		for(i=0;i<nn;i++)
			y[i]=y[i]/D[i][i];
		
		x[nn-1] = y[nn-1]/U[nn-1][nn-1];
		for (i = nn-2 ; i >= 0 ; i--)
		{
			x[i] = y[i];
			for(j = i + 1 ; j < nn ; j++)
				x[i] = x[i] - U[i][j]*x[j];
			x[i] = x[i]/U[i][i];
		}
	}
	return x;
}

Vec Mat::LDUsolve(const Mat &L, const Mat &D, const Mat &U, const Mat &P, const Vec &b) const
{
	Vec x;
	if((nn==mm)&&(nn==b.nn))
	{
		int i,j;
		Vec b2=P*b;
		
		x=Vec(0.0,mm);
		Vec y(0.0,nn);
		
		y[0] = b2[0];
		for (i = 1 ; i < nn ; i++)
		{
			y[i] = b2[i];
			for(j = 0 ; j < i ; j++)
				y[i] = y[i] - L[i][j]*y[j];
		}

		for(i=0;i<nn;i++)
			y[i]=y[i]/D[i][i];
	
		x[nn-1] = y[nn-1]/U[nn-1][nn-1];
		for (i = nn-2 ; i >= 0 ; i--)
		{
			x[i] = y[i];
			for(j = i + 1 ; j < nn ; j++)
				x[i] = x[i] - U[i][j]*x[j];
			x[i] = x[i]/U[i][i];
		}
	}
	return x;
}

Mat Mat::LDUinverse() const
{
	Mat w(nn,mm);
	if(nn==mm)
	{
		Mat L, D, U, P;
		LDU(L,D,U,P);

		int i,j;
		Vec aux;
		Vec b(0.0,mm);
		for(j=0;j<mm;j++)
		{
			b[j]=1;
			aux=LDUsolve(L,D,U,P,b);
			for(i=0;i<nn;i++)
				w[i][j]=aux[i];
			b[j]=0.0;
		}
	}
	return(w);
}

double Mat::LDUdet() const
{
	double det=1;
	if(nn==mm)
	{
		Mat w(*this);
		int i,sign;
		Vec perm;
		w=LDU(perm,sign);
		det=sign;
		for(i=0;i<nn;i++)
			det=det*w[i][i];
	}
	return det;
}


Mat Mat::inv() const
{
	return LDUinverse();
}

Mat Mat::transp() const
{
	Mat w(mm,nn);
	int i,j;

	for(i=0;i<nn;i++)
		for(j=0;j<mm;j++) w[j][i] = v[i][j];
		
	return(w);
}

double Mat::det() const
{
	double det=0.0;
	if(nn==mm)
	{
		switch(nn)
		{
		case 1:
			det=v[0][0];
			break;
		case 2:
			det=v[0][0]*v[1][1] - v[0][1]*v[1][0];
			break;
		case 3:
			det= v[0][0]*v[1][1]*v[2][2]
		       + v[0][2]*v[1][0]*v[2][1]
		       + v[0][1]*v[1][2]*v[2][0]
	           - v[0][2]*v[1][1]*v[2][0]
		       - v[0][0]*v[1][2]*v[2][1]
		       - v[0][1]*v[1][0]*v[2][2];
			   break;
		default:
			det=LDUdet();
		}
	}
	return det;
}
				
Vec Mat::getRow(const int n) const
{
	Vec row(mm);
	if((n>=0)&&(n<nn))
	{
		int i;
		for(i=0;i<mm;i++)
			row[i]=v[n][i];
	}
	return row;
}

Vec Mat::getCol(const int m) const
{
	Vec col(nn);
	if((m>=0)&&(m<mm))
	{
		int i;
		for(i=0;i<nn;i++)
			col[i]=v[i][m];
	}
	return col;
}

void Mat::setRow(const int n, const Vec &row)
{
	if((n>=0)&&(n<nn)&&(row.nn==mm))
	{
		int i;
		for(i=0;i<mm;i++)
			v[n][i]=row[i];
	}
}

void Mat::setCol(const int m, const Vec &col)
{
	if((m>=0)&&(m<mm)&&(col.nn==nn))
	{
		int i;
		for(i=0;i<nn;i++)
			v[i][m]=col[i];
	}
}

Vec Mat::LSsolve(const Vec &b) const
{
	Vec a(mm);
	Mat X(*this);
	if(nn==b.nn)
	{
		a=((X.transp()*X).inv())*(X.transp()*b);
	}
	return a;
}


double Mat::mean() const
{
  double aux=0.0;

  for (int i=0; i<nn; i++)
    {
      double *vpointer = &v[i][0];
      for (int j=0; j<mm; j++)
	{
	  aux+=*vpointer;
	  vpointer++;
	}
    }
  return (aux/(double)(nn*mm));
}

double Mat::variance() const
{
  if((nn*mm)<=1)
    return 0;
  else
    {
      double aux=0;
      double m=mean();
      for (int i=0; i<nn; i++)
	{
	  double *vpointer = &v[i][0];
	  for (int j=0; j<mm; j++)
	    {
	      aux+=(*vpointer - m) * (*vpointer - m);
	      vpointer++;
	    }
	}
      return (aux/(double)((nn*mm)-1));
    }
}

double Mat::stdev() const
{
  return(sqrt(variance()));
}

double Mat::PYTHAG(double a, double b) const
{
    double at = fabs(a), bt = fabs(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrt(1.0 + ct * ct); }
    else if (bt > 0.0) { ct = at / bt; result = bt * sqrt(1.0 + ct * ct); }
    else result = 0.0;
    return(result);
}

int Mat::SVD(Mat &U, Vec &sigma, Mat &V) const
{
    int flag, i, its, j, jj, k, l, nm;
    double c, f, h, s, x, y, z;
    double anorm = 0.0, g = 0.0, scale = 0.0;
    double *rv1;
    if (nn < mm) 
    {
        fprintf(stderr, "#rows must be >= #cols \n");
        return(0);
    }
  
    U = Mat(*this);
    sigma = Vec(0.0,mm);
    V = Mat(0.0,mm,mm);

    rv1 = (double *)malloc((unsigned int) mm*sizeof(double));
    l=0;
/* Householder reduction to bidiagonal form */
    for (i = 0; i < mm; i++) 
    {
        /* left-hand reduction */
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < nn) 
        {
            for (k = i; k < nn; k++) 
                scale += fabs((double)U[k][i]);
            if (scale) 
            {
                for (k = i; k < nn; k++) 
                {
                    U[k][i] = (float)((double)U[k][i]/scale);
                    s += ((double)U[k][i] * (double)U[k][i]);
                }
                f = (double)U[i][i];
		g = -SIGN(f)*sqrt(s);
                h = f * g - s;
                U[i][i] = (float)(f - g);
                if (i != mm - 1) 
                {
                    for (j = l; j < mm; j++) 
                    {
                        for (s = 0.0, k = i; k < nn; k++) 
                            s += ((double)U[k][i] * (double)U[k][j]);
                        f = s / h;
                        for (k = i; k < nn; k++) 
                            U[k][j] += (float)(f * (double)U[k][i]);
                    }
                }
                for (k = i; k < nn; k++) 
                    U[k][i] = (float)((double)U[k][i]*scale);
            }
        }
        sigma[i] = (float)(scale * g);
    
        /* right-hand reduction */
        g = s = scale = 0.0;
        if (i < nn && i != mm - 1) 
        {
            for (k = l; k < mm; k++) 
                scale += fabs((double)U[i][k]);
            if (scale) 
            {
                for (k = l; k < mm; k++) 
                {
                    U[i][k] = (float)((double)U[i][k]/scale);
                    s += ((double)U[i][k] * (double)U[i][k]);
                }
                f = (double)U[i][l];
                g = -SIGN(f)*sqrt(s);
                h = f * g - s;
                U[i][l] = (float)(f - g);
                for (k = l; k < mm; k++) 
                    rv1[k] = (double)U[i][k] / h;
                if (i != nn - 1) 
                {
                    for (j = l; j < nn; j++) 
                    {
                        for (s = 0.0, k = l; k < mm; k++) 
                            s += ((double)U[j][k] * (double)U[i][k]);
                        for (k = l; k < mm; k++) 
                            U[j][k] += (float)(s * rv1[k]);
                    }
                }
                for (k = l; k < mm; k++) 
                    U[i][k] = (float)((double)U[i][k]*scale);
            }
        }
        anorm = MAX(anorm, (fabs((double)sigma[i]) + fabs(rv1[i])));
    }
  
    /* accumulate the right-hand transformation */
    for (i = mm - 1; i >= 0; i--) 
    {
        if (i < mm - 1) 
        {
            if (g) 
            {
                for (j = l; j < mm; j++)
                    V[j][i] = (float)(((double)U[i][j] / (double)U[i][l]) / g);
                    /* double division to avoid underflow */
                for (j = l; j < mm; j++) 
                {
                    for (s = 0.0, k = l; k < mm; k++) 
                        s += ((double)U[i][k] * (double)V[k][j]);
                    for (k = l; k < mm; k++) 
                        V[k][j] += (float)(s * (double)V[k][i]);
                }
            }
            for (j = l; j < mm; j++) 
                V[i][j] = V[j][i] = 0.0;
        }
        V[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }
  
    /* accumulate the left-hand transformation */
    for (i = mm - 1; i >= 0; i--) 
    {
        l = i + 1;
        g = (double)sigma[i];
        if (i < mm - 1) 
            for (j = l; j < mm; j++) 
                U[i][j] = 0.0;
        if (g) 
        {
            g = 1.0 / g;
            if (i != mm - 1) 
            {
                for (j = l; j < mm; j++) 
                {
                    for (s = 0.0, k = l; k < nn; k++) 
                        s += ((double)U[k][i] * (double)U[k][j]);
                    f = (s / (double)U[i][i]) * g;
                    for (k = i; k < nn; k++) 
                        U[k][j] += (float)(f * (double)U[k][i]);
                }
            }
            for (j = i; j < nn; j++) 
                U[j][i] = (float)((double)U[j][i]*g);
        }
        else 
        {
            for (j = i; j < nn; j++) 
                U[j][i] = 0.0;
        }
        ++U[i][i];
    }

    /* diagonalize the bidiagonal form */
    for (k = mm - 1; k >= 0; k--) 
    {                             /* loop over singular values */
        for (its = 0; its < 30; its++) 
        {                         /* loop over allowed iterations */
            flag = 1;
            for (l = k; l >= 0; l--) 
            {                     /* test for splitting */
                nm = l - 1;
                if (fabs(rv1[l]) + anorm == anorm) 
                {
                    flag = 0;
                    break;
                }
                if (fabs((double)sigma[nm]) + anorm == anorm) 
                    break;
            }
            if (flag) 
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++) 
                {
                    f = s * rv1[i];
                    if (fabs(f) + anorm != anorm) 
                    {
                        g = (double)sigma[i];
                        h = PYTHAG(f, g);
                        sigma[i] = (float)h; 
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < nn; j++) 
                        {
                            y = (double)U[j][nm];
                            z = (double)U[j][i];
                            U[j][nm] = (float)(y * c + z * s);
                            U[j][i] = (float)(z * c - y * s);
                        }
                    }
                }
            }
            z = (double)sigma[k];
            if (l == k) 
            {                  /* convergence */
                if (z < 0.0) 
                {              /* make singular value nommegative */
                    sigma[k] = (float)(-z);
                    for (j = 0; j < mm; j++) 
                        V[j][k] = (-V[j][k]);
                }
                break;
            }
            if (its >= 30) {
                free((void*) rv1);
                fprintf(stderr, "No convergence after 30,000! iterations \n");
                return(0);
            }
    
            /* shift from bottom 2 x 2 minor */
            x = (double)sigma[l];
            nm = k - 1;
            y = (double)sigma[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = PYTHAG(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(f)*fabs(g))) - h)) / x;
          
            /* next QR transformation */
            c = s = 1.0;
            for (j = l; j <= nm; j++) 
            {
                i = j + 1;
                g = rv1[i];
                y = (double)sigma[i];
                h = s * g;
                g = c * g;
                z = PYTHAG(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < mm; jj++) 
                {
                    x = (double)V[jj][j];
                    z = (double)V[jj][i];
                    V[jj][j] = (float)(x * c + z * s);
                    V[jj][i] = (float)(z * c - x * s);
                }
                z = PYTHAG(f, h);
                sigma[j] = (float)z;
                if (z) 
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < nn; jj++) 
                {
                    y = (double)U[jj][j];
                    z = (double)U[jj][i];
                    U[jj][j] = (float)(y * c + z * s);
                    U[jj][i] = (float)(z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            sigma[k] = (float)x;
        }
    }
    free((void*) rv1);
    return(1);
}
