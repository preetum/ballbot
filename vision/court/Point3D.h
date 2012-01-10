/** A class to represent 3D points with double precision.
 *  @author : Ankush Gupta.
 *  @date   : 10th January 2012.*/

#include <string>

class Point3D {
    protected :
    double xCoor, yCoor, zCoor;

    public :
    /** Default Constructor.
     *  Initializes to (0, 0, 0).*/
    Point3D();

    /** Constructor. Initializes the point to (X, Y, Z).*/
    Point3D(double, double, double);

    /** Return the x-coordinate.*/
    double x();

    /** Set the x-coordinate to VAL.*/
    void x(double);

    /** Return the y-coordinate.*/
    double y();

    /** Set the y-coordinate to VAL.*/
    void y(double);

    /** Return the z-coordinate.*/
    double z();

    /** Set the z-coordinate to VAL.*/
    void z(double);

    /** Sets the point to (X, Y, Z).*/
    void set(double, double, double);

    /** Override : the equality operator.*/
    bool operator==(Point3D);

    /** Override : the + operator.
     *  Returns the result of VECTOR ADDITION of 2 points.
     *  e.g.: (1, 2, 3) + (3, 4, 5)  = (4, 6, 8)--> this is returned.*/
    Point3D operator+(Point3D);

    /** Override : the + operator.
     *  Returns the result of VECTOR SUBTRACTION of 2 points.
     *  e.g.: (1, 2, 3) - (3, 4, 5)  = (-2, -2, -2)--> this is returned.*/
    Point3D operator-(Point3D);

    /** Override : the * operator.
     *  Returns the result of SCALAR MULTIPLICATION.
     *  e.g.: (1, 2, 3) * 2  = (2, 4, 6)--> this is returned.*/
    Point3D operator*(double);
    Point3D operator*(int);

    /** Returns the dot product of 2 points.
     *  e.g. : (1,2,3).dot((3,4,5)) = 1*3 + 2*4 + 3*5 = 26.*/
    double dot(Point3D);

    /** Returns the cross (vector) product of 2 points.*/
    Point3D cross(Point3D pt);

    /** Returns the norm of the Point.
     *  where, norm = (x^2 + y^2 + z^2)^.5 **/
    double norm();

    /** Prints the point on standard output.*/
    void print();

    /** Retuns a string representation of the point.*/
    std::string toString();
};

/** Scalar mulplication. * is overloaded.*/
Point3D operator*(int , Point3D);
Point3D operator*(double , Point3D);
