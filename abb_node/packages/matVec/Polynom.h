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

#endif	// !defined(POLYNOM_INCLUDED)
