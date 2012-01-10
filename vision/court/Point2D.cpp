/** A class to represent 2D points with double precision.
 *  @author : Ankush Gupta.
 *  @date   : 10th January 2012.*/

#include "Point2D.h"
#include <stdio.h>
#include <math.h>

/** Constructor. Initializes to (0, 0).*/
Point2D::Point2D() {
    xCoor = 0.d;
    yCoor = 0.d;
}

/** Constructor. Initializes the point to (X, Y).*/
Point2D::Point2D(double x, double y) {
    xCoor = x;
    yCoor = y;
}

/** Return the x-coordinate.*/
double Point2D::x() {
    return xCoor;
}
 
/** Set the x-coordinate to VAL.*/
void Point2D::x(double val) {
    xCoor = val;
}

/** Return the y-coordinate.*/
double Point2D::y() {
    return yCoor;
}

/** Set the y-coordinate to VAL.*/
void Point2D::y(double val) {
    yCoor = val;
}

/** The the point to (X, Y).*/
void Point2D::set(double x, double y) {
    xCoor = x;
    yCoor = y;
}

/** Override : the equality operator.*/
bool Point2D::operator==(Point2D otherPt) {
    return (xCoor == otherPt.x() && yCoor == otherPt.y());
}

/** Override : the + operator.
 *  Returns the result of VECTOR ADDITION of 2 points.
 *  e.g.: (1, 2) + (3, 4)  = (4, 6)--> this is returned.*/
Point2D Point2D::operator+(Point2D pt) {
    Point2D result(xCoor + pt.x(), yCoor + pt.y());
    return result;
}

/** Override : the - operator.
 *  Returns the result of VECTOR SUBTRACTION of 2 points.
 *  e.g.: (1, 2) - (3, 4)  = (-2, -2)--> this is returned.*/
Point2D Point2D::operator-(Point2D pt) {
    Point2D result(xCoor - pt.x(), yCoor - pt.y());
    return result;
}

/** Override : the * operator.
 *  Returns the result of SCALAR MULTIPLICATION.
 *  e.g.: (1, 2) * 2  = (2, 4)--> this is returned.*/
Point2D Point2D::operator*(double k) {
    Point2D result(k * xCoor, k * yCoor);
    return result;
}
Point2D Point2D::operator*(int k ) {
    Point2D result(k * xCoor, k * yCoor);
    return result;
}
Point2D operator*(int k, Point2D pt) {
    Point2D result(k * pt.x(), k * pt.y());
    return result;
}
Point2D operator*(double k, Point2D pt) {
    Point2D result(k * pt.x(), k * pt.y());
    return result;
}

/** Returns the dot product of 2 points.
 *  e.g. : (1,2).dot((3,4)) = 1*3 + 2*4 = 11.*/
double Point2D::dot(Point2D pt) {
    double result = xCoor*pt.x() + yCoor*pt.y();
    return result;
}

/** Returns the result of rotating the point around the origin
 *  by ANGLE, specified in RADIANS in the counterclockwise direction.*/
Point2D Point2D::rotate(double angle) {
    Point2D result(xCoor * cos(angle) - yCoor * sin(angle),
		   xCoor * sin(angle) + yCoor * cos(angle));
    return result;
}

/** Returns the norm of the Point.
 *  where, norm = (x^2 + y^2)^.5 **/
double Point2D::norm() {
    double n = sqrt(pow(xCoor, 2.d) + pow(yCoor, 2.d));
    return n;
}

/** Prints the point on standard output.*/
void Point2D::print() {
    printf("(%.3f, %.3f)", xCoor, yCoor);
}
