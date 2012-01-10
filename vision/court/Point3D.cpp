/** A class to represent 3D points with double precision.
 *  @author : Ankush Gupta.
 *  @date   : 10th January 2012.*/

#include "Point3D.h"
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <string>

/** Constructor. Initializes to (0, 0, 0).*/
Point3D::Point3D() {
    xCoor = 0.d;
    yCoor = 0.d;
    zCoor = 0.d;
}

/** Constructor. Initializes the point to (X, Y, Z).*/
Point3D::Point3D(double x, double y, double z) {
    xCoor = x;
    yCoor = y;
    zCoor = z;
}

/** Return the x-coordinate.*/
double Point3D::x() {
    return xCoor;
}
 
/** Set the x-coordinate to VAL.*/
void Point3D::x(double val) {
    xCoor = val;
}

/** Return the y-coordinate.*/
double Point3D::y() {
    return yCoor;
}

/** Set the y-coordinate to VAL.*/
void Point3D::y(double val) {
    yCoor = val;
}

/** Return the z-coordinate.*/
double Point3D::z() {
    return zCoor;
}

/** Set the y-coordinate to VAL.*/
void Point3D::z(double val) {
    zCoor = val;
}

/** Set the point to (X, Y, Z).*/
void Point3D::set(double x, double y, double z) {
    xCoor = x;
    yCoor = y;
    zCoor = z;
}

/** Override : the equality operator.*/
bool Point3D::operator==(Point3D otherPt) {
    return (xCoor == otherPt.x()
	    && yCoor == otherPt.y()
	    && zCoor == otherPt.z());
}

/** Override : the + operator.
 *  Returns the result of VECTOR ADDITION of 2 points.
 *  e.g.: (1, 2, 3) + (3, 4, 5)  = (4, 6, 8)--> this is returned.*/
Point3D Point3D::operator+(Point3D pt) {
    Point3D result(xCoor + pt.x(), yCoor + pt.y(), zCoor + pt.z());
    return result;
}

/** Override : the - operator.
 *  Returns the result of VECTOR SUBTRACTION of 2 points.
 *  e.g.: (1, 2, 3) - (3, 4, 5)  = (-2, -2, -2)--> this is returned.*/
Point3D Point3D::operator-(Point3D pt) {
    Point3D result(xCoor - pt.x(), yCoor - pt.y(), zCoor - pt.z());
    return result;
}

/** Override : the * operator.
 *  Returns the result of SCALAR MULTIPLICATION. It is non-destrcutive.
 *  e.g.: (1, 2, 3) * 2  = (2, 4, 6)--> this is returned.*/
Point3D Point3D::operator*(double k) {
    Point3D result(k * xCoor, k * yCoor, k * zCoor);
    return result;
}
Point3D Point3D::operator*(int k ) {
    Point3D result(k * xCoor, k * yCoor, k * zCoor);
    return result;
}
Point3D operator*(int k, Point3D pt) {
    Point3D result(k * pt.x(), k * pt.y(), k * pt.z());
    return result;
}
Point3D operator*(double k, Point3D pt) {
    Point3D result(k * pt.x(), k * pt.y(), k * pt.z());
    return result;
}

/** Returns the dot product of 2 points.
 *  e.g. : (1,2,3).dot((3,4,5)) = 1*3 + 2*4 + 3*5= 26.*/
double Point3D::dot(Point3D pt) {
    double result = xCoor*pt.x() + yCoor*pt.y() + zCoor*pt.z();
    return result;
}

/** Returns the cross (vector) product of 2 points.*/
Point3D Point3D::cross(Point3D pt) {
    Point3D result(yCoor*pt.z() - zCoor*pt.y(),
		   zCoor*pt.x() - xCoor*pt.z(),
		   xCoor*pt.y() - yCoor*pt.x());
    return result;
}

/** Returns the norm of the Point.
 *  where, norm = (x^2 + y^2 + z^2)^.5 **/
double Point3D::norm() {
    double n = sqrt(pow(xCoor, 2.d) + pow(yCoor, 2.d) + pow(zCoor, 2.d));
    return n;
}

/** Prints the point on standard output.*/
void Point3D::print() {
    printf("(%.3f, %.3f, %.3f)", xCoor, yCoor, zCoor);
}

/** Retuns a string representation of the point.*/
std::string Point3D::toString() {
    char buffer[21];
    sprintf(buffer, "(%.3f, %.3f, %.3f)", xCoor, yCoor, zCoor);
    return std::string(buffer);
}
