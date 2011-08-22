/*
 * findlines_try.cpp
 *
 *  Created on: Aug 8, 2011
 *      Author: ankush
 */

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <list>

using namespace cv;
using namespace std;

const double pi  = 3.1415926535898;
const double inf =  numeric_limits<double>::infinity();

bool verbose = true;

double normalizeRadians(double );

struct line_r_theta
{
    double r;
    double theta;

    line_r_theta(double init_r, double init_theta)
        {
            r = init_r;
            theta = init_theta;
        }

    line_r_theta()
        {//absurd initialization
            r = inf;
            theta = inf;
        }

    bool operator==(const line_r_theta &other)
        {
            return (other.r == r && other.theta == theta);
        }

    void operator=(const line_r_theta &other)
        {
            r = other.r;
            theta = other.theta;
        }


};

struct lines_pair
{
    line_r_theta l1;
    line_r_theta l2;

    lines_pair()
        {
            l1.r = inf;
            l1.theta = inf;

            l2.r = inf;
            l2.theta  =inf;
        }
};

struct segments_intersections
{
    vector <vector <Point2d> > segments;
    vector <Point2d> intersections;

    segments_intersections(vector <vector <Point2d> > segments_other,
                           vector <Point2d> intersections_other)
        {
            segments = segments_other;
            intersections = intersections_other;
        }

    segments_intersections()
        {}
};

line_r_theta normalizeLine(line_r_theta line)
{
    /*
      Returns normalized line = (r, theta) such that
      r >=0
      0 <= theta < 2pi
    */
    double r = line.r;
    double theta = line.theta;
    if (r < 0)
    {
        r = -r;
        theta += pi;
        theta = normalizeRadians(theta);
    }
    return line_r_theta(r, theta);
}

line_r_theta avgLines(vector <line_r_theta> lines)
{
    /*
      Computes average of the lines (given as (r, theta) pairs)
      Returns average (r, theta)
    */
    double sum_r = 0, sum_sin = 0, sum_cos = 0;
    if(!lines.empty())
    {
        for(unsigned int i = 0; i< lines.size(); i++)
        {
            sum_r+= lines[i].r;
            sum_sin += sin(lines[i].theta);
            sum_cos += cos(lines[i].theta);
        }
        double r = sum_r/ lines.size();
        double theta = atan(sum_sin/sum_cos);
        return normalizeLine(line_r_theta(r, theta));
    }
}

Point2d toXY(line_r_theta line)
{    /*
       Converts (r, theta) to (x, y)
       (foot of the prependicular from origin to the line)
     */
    return Point2d(line.r*cos(line.theta), line.r*sin(line.theta));
}

void lineToSlopeIntercept(line_r_theta line, double &m, double &b)
{
    /*
      Converts a line in (r, theta) to slope-intercept (mx + b)
      Outputs in the reference variables m and b
    */
    Point2d p = toXY(line);

    if (p.y == 0.0)
    {//TODO: Get a better representation of infinity

        m = inf;
        b= inf;
    }

    m = -(p.x/p.y);
    b = -(m*p.x)+p.y;
}


unsigned char getPx(Mat &img, double x, double y)
{
    Size size = img.size();

    x = min(max(0.0, x), (double) size.width-1.0);
    y = min(max(0.0, y), (double) size.height-1.0);
    Point srchPt(cvRound(x),cvRound(y));

    unsigned char val = img.at<unsigned char>(srchPt); // assumes img is a grayscale single channel matrix
    return val;

}

Point2d lineIntersectionPoints(Point2d line1_p1, Point2d line1_p2, Point2d line2_p1, Point2d line2_p2)
{
    /*
      Finds the intersection point of two 2D lines given
      two points on each line.
      Returns intersection point as (x, y)
      See http://mathworld.wolfram.com/Line-LineIntersection.html
    */
    double x1 = line1_p1.x, y1 = line1_p1.y;
    double x2 = line1_p2.x, y2 = line1_p2.y;
    double x3 = line2_p1.x, y3 = line2_p1.y;
    double x4 = line2_p2.x, y4 = line2_p2.y;

    double a = x1*y2 - y1*x2;
    double b = x3*y4 - y3*x4;
    double d = (y3-y4)*(x1-x2) - (y1-y2)*(x3-x4);

    if(d !=0){
        double x = (a*(x3-x4)-b*(x1-x2)) / d;
        double y = (a*(y3-y4) - b*(y1-y2))/ d;
        return Point2d(x, y);
    }

    return Point2d(inf, inf);

}


vector<Point2d> lineToVisibleSegment(line_r_theta line, Mat &threshold_img, Mat & frame_img, double eps=1e-8)
{
    /*
      Returns the two endpoints of the line segment in the frame
      line is in (r, theta) form
      return value is two points in ((x1, y1) (x2, y2)), where x1 <= x2
    */

    Size size = threshold_img.size();

    //Get two points on the line
    Point2d p1a = toXY(line);
    Point2d p1b = Point2d(p1a.x-p1a.y, p1a.x+p1a.y);

    // Find top-left point a
    // First try to intersect with each edge of image
    vector <Point2d> candidates;
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d(0, 0), Point2d((double)size.width, 0)));
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d(0, 0), Point2d(0, (double)size.height)));
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d(0, (double) size.height), Point2d((double)size.width, (double) size.height)));
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d((double) size.width, 0), Point2d((double)size.width, (double) size.height)));

    /*There should be only 2 intersection points with the image rectangle,
      (or 1 if it's tangent to a corner, or inf if it lies on an edge, but
      we'll ignore those cases for now since they're improbable)
    */

    //Expand bounds by epsilon=1e-8 to account for floating point imprecision
    vector <Point2d> points;
    for(unsigned int k = 0; k< candidates.size(); k++)
    	if ((-eps <= candidates[k].x) && (candidates[k].x <= (double)size.width+eps)
            &&
            ((-eps <= candidates[k].y) && (candidates[k].y <=(double) size.height+eps)))
            points.push_back(candidates[k]);

    if (points.size() != 2)
    {
        cout<<"Line is not within in image bounds!"<<endl;
        vector <Point2d> empty_vec ;
        return empty_vec;
    }

    //Try to iterate over pixel values starting at points[0]
    double m, b;  // y= mx+b
    if(p1a.y != 0)
    	m = -(p1a.x / p1a.y);
    else
        m = inf;
    b = -(m * p1a.x) + p1a.y;

    Point2d startPt(inf, inf), endPt(inf, inf);

    //Line is more horizontal: iterate over x values
    if (m < 1)
    {
        cout<<"horizontal called"<<endl;
        //Ensure that point[0] is leftmost point
        if (points[0].x > points[1].x)
        {
            //swap
            Point2f temp = points[0];
            points[0] = points[1];
            points[1] = temp;
        }

        //Look for start point from leftmost point
        double x = points[0].x;
        double y = points[0].y;

        while (x < points[1].x && startPt.x == inf)
        {
            if (getPx(threshold_img, x, y) != 0)
                startPt = Point2d(x, y);

            x += 1;
            y = m * x + b;
        }

        // Look for end point from rightmost point
        x = points[1].x; y = points[1].y;
        while (x > points[0].x and endPt.x == inf)
        {
            if (getPx(threshold_img, x, y) != 0)
                endPt = Point2d(x, y);

            x -= 1;
            y = m * x + b;
        }

        cout<<"horizontal done!"<<endl;
    }
    //Line is more vertical: iterate over y values
    else
    {
        m = -(p1a.y / p1a.x);
        b = (m * -p1a.y + p1a.x);

        //Ensure that point[0] is topmost point
        if (points[0].y > points[1].y)
        {
            // swap
            Point2f temp = points[0];
            points[0] = points[1];
            points[1] = temp;
        }

        //Look for start point from topmost point
        double x = points[0].x, y = points[0].y;
        while (y < points[1].y && startPt.x == inf)
        {
            if (getPx(threshold_img, x, y) != 0)
                startPt = Point2d(x, y);

            y += 1;
            x =(m * y)+ b;
        }

        x = points[1].x;  y = points[1].y;
        while (y > points[0].y and endPt.x == inf)
        {
            if (getPx(threshold_img, x, y) != 0)
                endPt = Point2d(x, y);

            y -= 1;
            x = (m * y) + b;
        }
    }
    if (startPt.x== inf or endPt.x == inf)
    {
        cout <<"Line segment not found. Start point: "<< startPt.x<<" , "
             <<startPt.y<<" . End point: "<< endPt.x << " , "<< endPt.y
             <<endl;

        vector <Point2d> empty_vec;
        return empty_vec;
    }

    vector <Point2d> line_points;
    line_points.push_back(startPt);
    line_points.push_back(endPt);
    return line_points;
}
double normalizeRadians(double theta)
{
    /*
      Normalize an angle to the interval [-pi, pi)
    */
    double normalized_theta = (fmod ((theta + pi), 2*pi) - pi);
    return normalized_theta;
}

Point2d lineIntersection(line_r_theta line1, line_r_theta line2)
{
    /*
      Finds the intersection point of two 2D lines in (r, theta) form,
      where the line is parallel to the vector denoted by (r, theta)
    */
    Point2d p1a = toXY(line1);
    Point2d p1b(p1a.x-p1a.y, p1a.x+p1a.y);
    Point2d p2a = toXY(line2);
    Point2d p2b(p2a.x-p2a.y, p2a.x+p2a.y);

    return lineIntersectionPoints(p1a, p1b, p2a, p2b);
}

bool valid_line(line_r_theta line)
{
    return (line.r != inf && line.theta != inf);
}




void draw_line(Mat & frame, line_r_theta lin, Scalar & color)
{
    double eps = 1e-8;
    Size size = frame.size();
    //Get two points on the line
    Point2d p1a = toXY(lin);
    Point2d p1b = Point2d(p1a.x-p1a.y, p1a.x+p1a.y);

    // Find top-left point a
    // First try to intersect with each edge of image
    vector <Point2d> candidates;
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d(0, 0), Point2d((double)size.width, 0)));
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d(0, 0), Point2d(0, (double)size.height)));
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d(0, (double) size.height), Point2d((double)size.width, (double) size.height)));
    candidates.push_back( lineIntersectionPoints(p1a, p1b, Point2d((double) size.width, 0), Point2d((double)size.width, (double) size.height)));

    //Expand bounds by epsilon=1e-8 to account for floating point imprecision
    vector <Point2d> points;
    for(unsigned int k = 0; k< candidates.size(); k++)
        if ((-eps <= candidates[k].x) && (candidates[k].x <= (double)size.width+eps)
            &&
            ((-eps <= candidates[k].y) && (candidates[k].y <=(double) size.height+eps)))
            points.push_back(candidates[k]);

    if (points.size() == 2)
        line(frame, points[0], points[1], color, 2, 8);
}

segments_intersections find_lines(Mat &frame)
{
    // Resize to 640x480

    Size frame_size = frame.size();
    Mat frame_small = Mat(480, 640, CV_8UC3);

    if (frame_size.width != 640)
        resize(frame, frame_small, Size(480, 640));
    else
        frame.copyTo(frame_small);

    // Threshold by distance: blank out all top pixels
    rectangle(frame_small, Point(0,0), Point(640, 80), Scalar(0,0,0),
              CV_FILLED);

    //Convert to grayscale
    frame_size = frame_small.size();
    Mat frame_gray;
    cvtColor(frame_small,frame_gray,CV_RGB2GRAY); //convert it to grayscale

    // Use color thresholding to get white lines
    Mat frame_thresh;
    threshold(frame_gray, frame_thresh , 190, 255, THRESH_BINARY);

    // Morphological ops to reduce noise
    // TODO try to reduce sizes to increase detection of faraway lines

    Mat openElement = getStructuringElement( MORPH_RECT, Size(7, 7), Point( 3, 3));
    Mat closeElement = getStructuringElement( MORPH_RECT, Size(11, 11), Point( 5, 5));

    morphologyEx(frame_thresh, frame_thresh, MORPH_OPEN, openElement);
    morphologyEx(frame_thresh, frame_thresh, MORPH_CLOSE, closeElement);

    // Use Canny edge detection to find edges
    Mat edges;
    Canny(frame_thresh, edges, 50, 200);

    // Use Hough transform to find equations for lines
    vector <Vec2f> line_storage;
    HoughLines(edges, line_storage, 1, (float)pi/180.0, 120);

    // Create a "list" of lines
    list <line_r_theta> lines;
    if(!line_storage.empty())
    	for(unsigned int k = 0; k< line_storage.size(); k++)
        {
            line_r_theta l;
            l.r = (double)line_storage[k][0];
            l.theta = (double) line_storage[k][1];
            lines.push_back(l);
        }
    //else
    //{
    //segments_intersections seg_intersec;
    //return seg_intersec;
    //}

    // Remove spurious line from the black rectangle up top
    unsigned int counter = 0;
    for (list<line_r_theta>::iterator itr = lines.begin();
         itr != lines.end(); itr++, counter++)
    	if ((abs(80 - itr->r) < 10) &&
            (abs(normalizeRadians((pi / 2.0) - itr->theta)) < 0.01))
        {
            lines.erase(itr);
            counter -=1;

            itr = lines.begin();
            itr--;
            for(unsigned int j = 0; j < counter;j++)
                itr++;
        }

    // Group lines that are within r +/-30 and theta +/- 30 degrees
    list <line_r_theta> grouped_lines;
    double r_threshold = 30.0;  // in px
    double theta_threshold = ((pi* 35.0) / 180.0); // in radians

    while (lines.size() > 0)
    {

        line_r_theta line1 = normalizeLine(lines.back());
        lines.pop_back();

        line_r_theta avg_line = line1;

        vector <line_r_theta> matched_lines;
        matched_lines.push_back(line1);

        if(!lines.empty())
        {
            int count = 0;
            for (list<line_r_theta>::iterator itr = lines.begin();
                 itr != lines.end(); itr++, count++)
            {
                line_r_theta line2 = normalizeLine(*itr);

                if (verbose)
                {  //Print which criteria were matched
                    if (abs(avg_line.r - line2.r) < r_threshold)
                        cout<<"matched r"<<endl;
                    if (abs(normalizeRadians(avg_line.theta - line2.theta)) <
                        theta_threshold)
                        cout<<"matched theta"<<endl;

                    cout<<"Average Line: r = "<< avg_line.r <<" | theta = "<< avg_line.theta<<endl;
                    cout<<"Line: r = "<< line2.r <<" | theta = "<< line2.theta<<endl;
                }

                if ((abs(avg_line.r- line2.r) < r_threshold) &&
                    (abs(normalizeRadians(avg_line.theta - line2.theta)) <
                     theta_threshold))
                {
                    matched_lines.push_back(line2);
                    avg_line = avgLines(matched_lines);

                    lines.erase(itr); //the iterator becomes invalid after erasing
                    //re-initialize the iterator
                    count -= 1;
                    itr = lines.begin();
                    itr --;
                    for(int j = 0; j< count; j++ )
                        itr++;

                }
            }
        }
        if (verbose)
        {
            cout<<"Matched Lines: "<<endl;
            for(unsigned int k = 0; k < matched_lines.size(); k++)
                cout<<k<<". r = "<<matched_lines[k].r<<" theta = "
                    <<matched_lines[k].theta<<endl;
            cout<<endl;
        }
        grouped_lines.push_back(avg_line);
    }
    lines = grouped_lines;
    grouped_lines.clear();

    // Group possible pairs of lines by smallest angle difference
    vector <lines_pair> paired_lines; // in original code paired_lines is grouped_lines
    while (lines.size() > 0)
    {
        line_r_theta line1 = normalizeLine(lines.back());
        lines.pop_back();
        line_r_theta closest(inf, inf);

        for (list<line_r_theta>::reverse_iterator rev_itr = lines.rbegin();
             rev_itr != lines.rend(); rev_itr++)
        {
            line_r_theta line2 = normalizeLine(*rev_itr);
            if((closest.r == inf)
               //Make sure difference < pi/4 to reduce errors
               && (abs(normalizeRadians(line1.theta - line2.theta)) < pi/ 4))
                closest = line2;

            else if ((abs(normalizeRadians(line1.theta - line2.theta))
                      < abs(normalizeRadians(line1.theta - closest.theta))) &&
                     (abs(normalizeRadians(line1.theta - line2.theta)) < pi/ 4))
                closest = line2;
        }

        if (closest.r != inf && closest.theta != inf)
        {

            lines.remove(closest);
            //Sort list by line.r (radius)
            if (line1.r > closest.r)
            {
                // swap
                line_r_theta temp_line = line1;
                line1 = closest;
                closest = temp_line;
            }
            // Make a tuple (line1, line2) or (line,) if no match found
            lines_pair p;
            p.l1 = line1;
            p.l2 = closest;
            paired_lines.push_back(p);
        }
        else if (closest.r ==inf && closest.theta == inf)
        {
            lines_pair p;
            p.l1 = line1;
            // p.l2 has (r,theta) == (inf,inf) due to constructor
            paired_lines.push_back(p);
        }
    }//while

    // Print lines
    if (paired_lines.size() > 0)
    {
        cout<<endl<<"Paired Lines: "<<endl;
        for(unsigned int k = 0; k < paired_lines.size(); k++)
        {
            cout<<'\t'<<"Pair #"<<(k+1)<<" : "<<endl;
            cout<<'\t'<<'\t'<<"r: "<<paired_lines[k].l1.r <<" theta: "<<paired_lines[k].l1.theta<<endl;
            if(paired_lines[k].l2.r != inf && paired_lines[k].l2.theta != inf)
                cout<<'\t'<<'\t'<<"r: "<<paired_lines[k].l2.r <<" theta: "<<paired_lines[k].l2.theta;
            cout<<endl;

            //draw lines: same color for lines in the same pair
            //Scalar color = Scalar(100*(k+1)*((k+1)%2),100*(k+1)*((k+2)%2), 100*(k+1)*((k+2)%2));
            //draw_line(frame_small, paired_lines[k].l1, color);
            //if(paired_lines[k].l2.r != inf && paired_lines[k].l2.theta != inf)
            //draw_line(frame_small, paired_lines[k].l2, color);
        }
    }

    // If 2+ groups of lines, find corners (intersection point of lines)
    vector <Point2d> intersection_pts;

    if (paired_lines.size() > 1)
        for(unsigned int i = 0 ; i < paired_lines.size(); i++)
        {
            lines_pair pair1 = paired_lines[i];
            for (unsigned int j = i+1; j < paired_lines.size(); j++)
            {
                lines_pair pair2 = paired_lines[j];

                // Make sure their angles differ by more than 30 deg to
                //  reduce errors
                if (abs(normalizeRadians(pair1.l1.theta - pair2.l1.theta)) <
                    (double)pi*30.0/180.0)
                {
                    cout<<"Invalid Pairs!: The angle difference is less than 30 degrees."<<endl;
                    break;
                }

                // Enumerate intersection points
                vector <Point2d> pts;

                if(valid_line(pair1.l1) && valid_line(pair2.l1))
                    pts.push_back(lineIntersection(pair1.l1, pair2.l1));

                if(valid_line(pair1.l1) && valid_line(pair2.l2))
                    pts.push_back(lineIntersection(pair1.l1, pair2.l2));

                if(valid_line(pair1.l2) && valid_line(pair2.l1))
                    pts.push_back(lineIntersection(pair1.l2, pair2.l1));

                if(valid_line(pair1.l2) && valid_line(pair2.l2))
                    pts.push_back(lineIntersection(pair1.l2, pair2.l2));

                // Find average of intersection points
                double sum_x = 0.00, sum_y = 0.00;


                if(!pts.empty())
                {
                    for(unsigned int k = 0 ; k < pts.size(); k++)
                    {
                        sum_x += pts[k].x;
                        sum_y += pts[k].y;
                    }

                    double x = sum_x/((double)pts.size()), y = sum_y/((double)pts.size());
                    Point2d pt(x,y);

                    //Print the intersections coordinates
                    cout<<"Intersection: "<<endl<<"x: "<<pt.x<<" y: "<<pt.y<<endl;
                    intersection_pts.push_back(pt);

                    //Draw the intersections
                    Point pt_pixels(cvRound(pt.x), cvRound(pt.y));
                    circle(frame_small, pt_pixels, 4, Scalar(0,100,0), CV_FILLED);

                    // Find direction of intersection by following each line
                    // (direction is defined as the point of the T)
                    vector <float> angles;
                    for(unsigned int k = 0; k < paired_lines.size(); k++)
                    {
                        angles.push_back(paired_lines[k].l1.theta + (pi/2.0));
                        angles.push_back(paired_lines[k].l1.theta - (pi/2.0));
                    }

                    for(unsigned int k = 0; k < angles.size(); k++)
                    {
                        // Look 50px down the line for white pixels
                        // TODO look a variable amount

                        float angle = angles[k];
                        float x1, y1;

                        int number_of_successes = 0;
                        cout<<"Number of angles: "<< angles.size();
                        for(unsigned int d = 30; d< 70; d+= 5)
                        {
                            x1 = x + d*cos(angle);
                            y1 = y + d*sin(angle);

                            // Enforce limits
                            // TODO handle when intersection is off the bounds of the image
                            //  Currently orientation of the intersection is not being used
                            //  by the particle filter
                            x1 = min(max((float) 0, x1), (float)(frame_size.width-1));
                            y1 = min(max((float) 0, y1), (float)(frame_size.height-1));
                            Point srchPt(cvRound(x1), cvRound(y1));

                            circle(frame_small, srchPt, 3, Scalar(0,0,100), CV_FILLED);
                            if(frame_thresh.at<unsigned char>(srchPt) == 0) //error in accessing the Matrix elements with indices
                                number_of_successes ++;
                        }

                        if(number_of_successes >= 5)
                        {
                            x1 = x + 50*cos(angle + (float) pi);
                            y1 = y + 50*sin(angle + (float) pi);
                            Point invSrchPt(cvRound(x1), cvRound(y1));
                            line(frame_small, pt, invSrchPt, Scalar(0,200,0), 10, 8);
                            cout<<"Angle: "<< angle+(float)pi<<endl;

                        }

                    }
                }
            }
        }
    // Convert line equations into line segments
    vector < vector <Point2d> > line_segments;
    line_r_theta lin;
    for(unsigned int k = 0; k < paired_lines.size(); k++)
    {

        lines_pair pair = paired_lines[k];

        if (pair.l2.r != inf && pair.l2.theta != inf)
        {
            // Get the average of the lines in a pair
            lin = line_r_theta( (pair.l1.r + pair.l2.r) / 2.0,
                                (pair.l1.theta+pair.l2.theta) / 2.0);
        }
        else
            lin = pair.l1;

        // Look down the line for the endpoints of the white region
        vector <Point2d> line_segment = lineToVisibleSegment(lin, frame_thresh, frame_small);
        if (!line_segment.empty())
            line_segments.push_back(line_segment);
    }

    if(!line_segments.empty())
    {
        //Print the line segments
        cout<<"Line Segments:"<<endl;

        RNG rng; //random number generator
        for(unsigned int k = 0; k < line_segments.size(); k++)
        {
            vector <Point2d> line_seg = line_segments[k];
            if(!line_seg.empty())
            {
                cout<<"Segment #"<<k<<" : Pt1 = "<<"("<<line_seg[0].x<<", "<<line_seg[0].y<<")"
                    <<" : Pt2 = "<<"("<<line_seg[1].x<<", "<<line_seg[1].y<<")"
                    <<endl;

                Point2d pt1 = line_seg[0], pt2 = line_seg[1];
                Point p1 (cvRound(pt1.x), cvRound(pt1.y));
                Point p2 (cvRound(pt2.x), cvRound(pt2.y));
                Scalar random_color((unsigned int)(255*rng.uniform(0.f, 1.f)),
                                    (unsigned int)	255*rng.uniform(0.f, 1.f),
                                    (unsigned int) 255*rng.uniform(0.f, 1.f));

                line(frame_small, p1, p2, random_color, 2, 8, 0);
            }
        }
    }
    imshow("frame", frame_small);
    imshow("edges", frame_thresh);

    return segments_intersections(line_segments, intersection_pts);
}

void CreateDisplayWindows()
{
    //first window
    namedWindow("frame", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("frame", 10, 10); // Position window

    //second window
    namedWindow("edges", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("edges", 600, 10); // Position window

}

/*
  int main(int argc, char** argv)
  {
  std::string arg = argv[1];
  VideoCapture capture(arg); //try to open string, this will attempt to open it as a video file
  if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
  capture.open(atoi(arg.c_str()));

  if (!capture.isOpened())  // if both video and camera fail to open
  {
  cerr << "Failed to open a video device or video file!\n" << endl;
  return -1;
  }

  CreateDisplayWindows(); //creates the windows for display
  verbose = true;

  Mat frame;
  capture.read(frame);
  capture.set(CV_CAP_PROP_POS_FRAMES, 2500);

  while(true)
  {

  capture.read(frame);
  find_lines(frame);

  waitKey(0);

  }

  capture.release();
  return 0;
  }
*/
