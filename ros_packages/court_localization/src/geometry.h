// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#ifndef _geometry_h
#define _geometry_h

#include <cv.h>
#include "backproject.h"

// Conversion functions (camera plane -> robot frame on ground)
cv::Vec4d cameraLineToRobot(const cv::Vec4i &line, const camera &cam);
cv::Point2d cameraPointToRobot(const cv::Point2i &pt, const camera &cam);

// Misc utility functions
cv::Vec4d pointsToLine(const cv::Point2d &pt1, const cv::Point2d &pt2);
cv::Vec4d pointsToLine(const cv::Point3d &pt1, const cv::Point3d &pt2);
double normalizeRadians(double rad);

// Line functions
cv::Vec2i lineCenter(const cv::Vec4i &line);
cv::Vec2d lineCenter(const cv::Vec4d &line);
double lineAngle(const cv::Vec4i &line);
double lineAngle(const cv::Vec4d &line);
double pointLineDistance(const cv::Vec2d &point, const cv::Vec4d &line);
double pointLineHeading(const cv::Vec2d &point, const cv::Vec4d &line);
double pointLineSegmentDistance(const cv::Vec2d &point, 
				const cv::Vec4d &segment);

/*
// There's probably a better way to do this...
inline double pointLineDistance(const cv::Vec2i &point,
                                const cv::Vec4i &line) {
    return pointLineDistance(cv::Vec2d(point), cv::Vec4d(line));
}
inline double pointLineDistance(const cv::Vec2d &point,
                                const cv::Vec4i &line) {
    return pointLineDistance(point, cv::Vec4d(line));
}
inline double pointLineDistance(const cv::Vec2i &point,
                                const cv::Vec4d &line) {
    return pointLineDistance(cv::Vec2d(point), line);
}

inline double pointLineSegmentDistance(const cv::Vec2i &point,
                                       const cv::Vec4i &segment) {
    return pointLineSegmentDistance(cv::Vec2d(point), cv::Vec4d(segment));
}
inline double pointLineSegmentDistance(const cv::Vec2d &point,
                                       const cv::Vec4i &segment) {
    return pointLineSegmentDistance(point, cv::Vec4d(segment));
}
inline double pointLineSegmentDistance(const cv::Vec2i &point,
                                       const cv::Vec4d &segment) {
    return pointLineSegmentDistance(cv::Vec2d(point), segment);
}
*/
#endif
