/** A class to represent a line segment, defined by 2 points (in 3 dimensions).
 *  @author Ankush Gupta
 *  @date 10th January 2012.*/

#include "LineSegment2D.h"
#include <iostream>

/** Initialize the end-points to (0,0).*/
LineSegment2D::LineSegment2D() {
    pt1 = Point2D();
    pt2 = Point2D();
}

/** Initializes first end-point to P1 and the other to P2.*/
LineSegment2D::LineSegment2D(Point2D p1, Point2D p2) {
    pt1 = p1;
    pt2 = p2;
}

/** Initializes the lineSegment with its end points as  (X1, Y1)
 *  and (X2, Y2).*/
LineSegment2D::LineSegment2D(double x1, double y1, double x2, double y2) {
    pt1 = Point2D(x1, y1);
    pt2 = Point2D(x2, y2);
}

/** Checks for equality of line segments.
 *  2 line segments are EQUAL iff their end-points are equal.*/
bool operator==(LineSegment2D l) {
    return ((pt1 == l.p1() && pt2 == l.p2())
	    || (pt1 == l.p2() && pt2 == l.p1()));
}

/** Returns the first end-point.*/
Point2D LineSegment2D::p1() {
	return pt1;
}

/** Returns the second end-point.*/
Point2D LineSegment2D::p2() {
    return pt2;
}

/** Returns the mid-point of the segment.*/
Point2D LineSegment2D::mid() {
    return (0.5 * (pt1 + pt2));
}

/** Returns the length of the line-segment.*/
double length() {
    return (pt1 - pt2).norm();
}

/** Prints the text representation on the standard output.*/
void LineSegment2D::print() {
    pt1.print(); std::cout<<"---"; pt2.print();
    std::cout<<std::endl;
}

/** Returns a string representation of the line segment.*/
std::string LineSegment2D::toString() {
    return (pt1.toString().append("---")).append(pt2.toString());
}
