#ifndef _findlines_h
#define _findlines_h

#include <vector>
#include <cv.h>
#include "backproject.h"

void findLines(cv::Mat &frame, std::vector<cv::Vec4i> &groupedLines,
	       camera *cam);
void drawLines(cv::Mat &frame, const std::vector<cv::Vec4i> &lines);

#endif
