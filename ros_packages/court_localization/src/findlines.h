#ifndef _findlines_h
#define _findlines_h

#include <vector>
#include <cv.h>

void findLines(cv::Mat &frame, std::vector<cv::Vec4i> &groupedLines);

#endif
