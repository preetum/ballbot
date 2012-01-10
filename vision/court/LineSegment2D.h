/** A class to represent a line segment, defined by 2 points (in 2 dimensions).
 *  @author Ankush Gupta
 *  @date 10th January 2012.*/

#include "Point2D.h"

class LineSegment2D {
    protected :
    /** The 2 end points.*/
    Point2D pt1, pt2;

    public :    
    /** Initializes the ends points to (0,0).*/
    LineSegment2D();

    /** Initializes first end-point to P1 and the other to P2.*/
    LineSegment2D(Point2D, Point2D);

    /** Initializes the lineSegment with its end points as  (X1, Y1)
     *  and (X2, Y2).*/
    LineSegment2D(double x1, double y1, double x2, double y2);

    /** Override the equality operator.*/
    bool operator==(LineSegment2D);

    /** Returns the first end-point.*/
    Point2D p1();

    /** Returns the second end-point.*/
    Point2D p2();

    /** Returns the mid-point of the segment.*/
    Point2D mid();

    /** Returns the length of the line-segment.*/
    double length();

    /** Prints the text representation on the standard output.*/
    void print();

    /** Returns a string representation of the line segment.*/
    std::string toString();
};
