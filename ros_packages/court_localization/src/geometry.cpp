// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#include <math.h>
#include "geometry.h"

using namespace cv;

// TODO store parameters
const double radians_per_px = 0.0016,
    frame_height = 480,
    frame_width = 640;

Vec4d cameraLineToRobot(const Vec4i &line, const camera &cam) {
    Point2d pt1 = cameraPointToRobot(Point2d(line[0], line[1]), cam),
        pt2 = cameraPointToRobot(Point2d(line[2], line[3]), cam);
    return Vec4d(pt1.x, pt1.y, pt2.x, pt2.y);
}

/* Converts a camera point to a point on the ground in the robot frame*/
Point2d cameraPointToRobot(const Point2d &pt, const camera &cam) {
    double theta = (pt.y - frame_height/2) * radians_per_px - cam.tilt,
        y = cam.position.z / tan(theta),
        phi = (pt.x - frame_width/2) * radians_per_px + cam.pan,
        x = y * tan(phi);

    return Point2d(x, y);
}

Vec4d pointsToLine(const Point2d &pt1, const Point2d &pt2) {
    return Vec4d(pt1.x, pt1.y, pt2.x, pt2.y);
}
Vec4d pointsToLine(const Point3d &pt1, const Point3d &pt2) {
    return Vec4d(pt1.x, pt1.y, pt2.x, pt2.y);
}

/* Normalize an angle to the range (-pi, pi] */
double normalizeRadians(double rad) {
    while (rad > CV_PI)
        rad -= 2*CV_PI;
    while (rad <= -CV_PI)
        rad += 2*CV_PI;
    return rad;
}

/* Returns the center of a line segment */
Vec2d lineCenter(const Vec4d &line) {
    return Vec2d((line[0]+line[2]) / 2, (line[1]+line[3]) / 2);
}

Vec2i lineCenter(const Vec4i &line) {
    return Vec2i((line[0]+line[2]) / 2, (line[1]+line[3]) / 2);
}

/* Returns the angle of the LINE which passes through
 * (x1, y1, x2, y2).
 *
 * Angle is given in radians counterclockwise from the x-axis,
 *  in the range (-pi/2, pi/2].
 */
double lineAngle(const Vec4d &line) {
    double y = line[3] - line[1],
        x = line[2] - line[0],
        theta = atan2(y, x);

    // Normalize answer to (-pi/2, pi/2]
    if (theta > CV_PI/2)
        theta -= CV_PI;
    else if (theta <= -CV_PI/2)
        theta += CV_PI;
    return theta;
}

double lineAngle(const Vec4i &line) {
    return lineAngle(Vec4d(line));
}

/* Returns the distance from POINT (x,y) to the LINE
 * which passes through points (x1, y1, x2, y2)
 *
 * Reference: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
 */
double pointLineDistance(const Vec2d &point, const Vec4d &line) {
    // Project point onto line
    // n is a unit normal vector of the line
    // v is a vector from point to line
    Vec2d n(line[1]-line[3], line[2]-line[0]);
    n *= 1.0/norm(n);
    Vec2d v(line[0]-point[0], line[1]-point[1]);

    return abs(n.dot(v));
}

double pointLineHeading(const Vec2d &point, const Vec4d &line) {
    // Project point onto line
    // n is a unit normal vector of the line
    // v is a vector from point to line
    Vec2d n(line[1]-line[3], line[2]-line[0]);
    n *= 1.0/norm(n);
    Vec2d v(line[0]-point[0], line[1]-point[1]);

    // Generate vector from point to the closest point on line
    Vec2d u = n.dot(v) * n;
    return atan2(u[1], u[0]);
}

/* Returns the distance from POINT (x,y) to line segment 
 * SEGMENT whose endpoints are (x1, y1, x2, y2)
 *
 * Reference: http://stackoverflow.com/questions/627563/702174#702174
 */
double pointLineSegmentDistance(const Vec2d &p, const Vec4d &segment) {
    Vec2d r(segment[0], segment[1]),
        s(segment[2], segment[3]);

    // Project point p onto line rs
    // n is a unit normal vector of line rs
    // v is a vector from p to line rs
    // d is the distance from p to line rs
    Vec2d n(r[1]-s[1], s[0]-r[0]);
    n *= 1.0/norm(n);
    Vec2d v = r - p;
    double d = n.dot(v);

    // We can parameterize the line rs as L(u) = r + (s-r)*u
    // then solve for the value of u where the projection of p lies
    // that is: L(u) = r + (s-r)* u = p + d*n
    // ||s-r||^2 * u = (s-r) * (p - r + d*n)
    //             u = (s-r) * (p - r + d*n) / ||s-r||^2
    Vec2d x = s - r;
    double u = x.dot(p - r + d*n) / x.dot(x);

    // If projection of p lies on line segment rs, return the distance
    if (0.0 <= u && u <= 1.0) {
        return abs(d);
    }
    // Otherwise return the distance to the closest endpoint
    else {
        double a = norm(p-s),
            b = norm(p-r);
        return min(a, b);
    }    
}
