/** A class to represent a line segment, defined by 2 points (in 3 dimensions).
 *  @author Ankush Gupta
 *  @date 10th January 2012.*/
#include "Point3D.h"

class LineSegment3D {
    protected :
    /** The 2 end points.*/
    Point3D pt1, pt2;

    public :    
    /** Initializes the ends points to (0,0,0).*/
    LineSegment3D();

    /** Initializes first end-point to P1 and the other to P2.*/
    LineSegment3D(Point3D, Point3D);

    /** Initializes the lineSegment with its end points as  (X1, Y1, Z1)
     *  and (X2, Y2, Z2).*/
    LineSegment3D(double x1, double y1, double z1, double x2, double y2,
                  double z2);

    /** Override the equality operator.*/
    bool operator==(LineSegment3D);

    /** Returns the first end-point.*/
    Point3D p1();

    /** Returns the second end-point.*/
    Point3D p2();

    /** Returns the mid-point of the segment.*/
    Point3D mid();

    /** Returns the length of the line-segment.*/
    double length();

    /** Prints the text representation on the standard output.*/
    void print();

    /** Returns a string representation of the line segment.*/
    std::string toString();
};
