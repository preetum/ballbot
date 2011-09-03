// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#ifndef _geometry_h
#define _geometry_h

#include <cv.h>

// Conversion functions
cv::Vec4i pointsToLine(const cv::Point2d &pt1, const cv::Point2d &pt2);

// Misc geometry functions
double normalizeRadians(double rad);

// Line functions
cv::Vec2i lineCenter(const cv::Vec4i &line);
double lineAngle(const cv::Vec4i &line);
double pointLineDistance(const cv::Vec2i &point, const cv::Vec4i &line);
double pointLineSegmentDistance(const cv::Vec2i &point, 
				const cv::Vec4i &segment);


#endif
