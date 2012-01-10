/** A class to represent a line segment, defined by 2 points (in 3 dimensions).
 *  @author Ankush Gupta
 *  @date 10th January 2012.*/

#include "LineSegment3D.h"
#include <iostream>

/** Initialize the end-points to (0,0,0).*/
LineSegment3D::LineSegment3D() {
    pt1 = Point3D();
    pt2 = Point3D();
}

/** Initializes first end-point to P1 and the other to P2.*/
LineSegment3D::LineSegment3D(Point3D p1, Point3D p2) {
    pt1 = p1;
    pt2 = p2;
}

/** Initializes the lineSegment with its end points as  (X1, Y1, Z1)
 *  and (X2, Y2, Z2).*/
LineSegment3D::LineSegment3D(double x1, double y1, double z1, double x2,
			     double y2, double z2) {
    pt1 = Point3D(x1, y1, z1);
    pt2 = Point3D(x2, y2, z2);
}

/** Returns the first end-point.*/
Point3D LineSegment3D::p1() {
	return pt1;
}

/** Returns the second end-point.*/
Point3D LineSegment3D::p2() {
    return pt2;
}

/** Returns the mid-point of the segment.*/
Point3D LineSegment3D::mid() {
    return (0.5 * (pt1 + pt2));
}

/** Prints the text representation on the standard output.*/
void LineSegment3D::print() {
    pt1.print(); std::cout<<"---"; pt2.print();
    std::cout<<std::endl;
}

/** Returns a string representation of the line segment.*/
std::string LineSegment3D::toString() {
    return (pt1.toString().append("...")).append(pt2.toString());
}
