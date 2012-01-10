/** A class to represent 2D points with double precision.
 *  @author : Ankush Gupta.
 *  @date   : 10th January 2012.*/

using namespace std;

class Point2D {
    protected :
    double xCoor, yCoor;

    public :
    /** Default Constructor.
     *  Initializes to (0, 0).*/
    Point2D();

    /** Constructor. Initializes the point to (X, Y).*/
    Point2D(double, double);

    /** Return the x-coordinate.*/
    double x();

    /** Set the x-coordinate to VAL.*/
    void x(double);

    /** Return the y-coordinate.*/
    double y();

    /** Set the y-coordinate to VAL.*/
    void y(double);

    /** The the point to (X, Y).*/
    void set(double, double);

    /** Override : the equality operator.*/
    bool operator==(Point2D);

    /** Override : the + operator.
     *  Returns the result of VECTOR ADDITION of 2 points.
     *  e.g.: (1, 2) + (3, 4)  = (4, 6)--> this is returned.*/
    Point2D operator+(Point2D);

    /** Override : the - operator.
     *  Returns the result of VECTOR SUBTRACTION of 2 points.
     *  e.g.: (1, 2) - (3, 4)  = (-2, -2)--> this is returned.*/
    Point2D operator-(Point2D);

    /** Override : the * operator.
     *  Returns the result of SCALAR MULTIPLICATION.
     *  e.g.: (1, 2) * 2  = (2, 4)--> this is returned.*/
    Point2D operator*(double);
    Point2D operator*(int);

    /** Returns the dot product of 2 points.
     *  e.g. : (1,2).dot((3,4)) = 1*3 + 2*4 = 11.*/
    double dot(Point2D);

    /** Returns the result of rotating by ANGLE, specified in
     *  RADIANS in the clockwise counterclockwise direction.*/
    Point2D rotate(double);

    /** Returns the norm of the Point.
     *  where, norm = (x^2 + y^2)^.5 **/
    double norm();

    /** Prints the point on standard output.*/
    void print();
};

/** Scalar mulplication. * is overloaded.*/
Point2D operator*(int , Point2D);
Point2D operator*(double , Point2D);
