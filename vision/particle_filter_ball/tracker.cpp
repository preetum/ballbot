/*
 * frame-by-frame.cpp
 *
 *  Created on: Jul 14, 2011
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

#include <pthread.h>
#include <unistd.h>

#include <mgl/mgl.h>
#include <mgl/mgl_data.h>
#include <mgl/mgl_fltk.h>
#include "p-by-p.h"

#define pi 3.1415926535897

using namespace cv;
using namespace std;

//------------------------* Structures *-------------------------------

struct candidate
{
	Point2f position;
	bool seed;
	int prev, next;

	candidate()  //initializer constructor
	{
		position.x = -1.0;
		position.y = -1.0;
		seed = false;
		prev = -1;
		next = -1;
	}
};

struct bgr
{float b,g,r;};

//--------------------------------x---x---x--------------------------------------

//--------------------- Globals-------------------------------------------------
Mat frame1, frame2, frame_p3, frame_p2, frame_p1,frame_n1, frame_n2,frame_n3, frame_now;
Mat tmp, dst;
vector <blob_observations> blob_obs;
observations obsvrs; // holds camera potion and blobs(3D) cooridnates


//------------------- Morphology variables --------------------------------------
int morph_elem = 0;
int morph_size = 5; //window size
int morph_operator = 3; //closing
int morph_iters = 1;    //number of morphology iterations
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size_m = 21;
int const max_kernel_size_d = 21;
int operation = 3; //closing
int dilation_elem = 0;
int dilation_size = 1;
Point2f invalid_point(-1,-1);

int frame_num = 0, candidate_num = 0;  //counters for candidate 2D array
vector <ball_particle> ball_p;



/** Function Headers */
void Morphology_Operations( int, void* );
void Morphology_trackbars(char*);
void Dilation_trackbars(char*);
void CreateDisplayWindows();
void Initial_Capture(VideoCapture &);
void Rotate_Get_New_Capture(VideoCapture &);
void Threshold_AND();
void Do_Contours(Mat &, Mat &);
void Plot_Rays(mglGraph *gr);
//-------------------------------------------------------------

Mat Morphology_element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
Mat Dilation_element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );


void Morphology_Operations(int, void* )  //opening, closing, top-hat, black-hat
{
  // Since MORPH_X : 2,3,4,5 and 6
  operation = morph_operator + 2;
  Morphology_element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  morphologyEx( tmp,dst, operation, Morphology_element, Point(-1,-1), morph_iters);
  imshow("Tracker",dst);
  }

void Dilation( int, void* )
{
	int dilation_type;
	if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
	else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
	else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
	Dilation_element = getStructuringElement( dilation_type,
	                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
	                                       Point( dilation_size, dilation_size ) );
	/// Apply the dilation operation
	dilate( tmp, dst, Dilation_element );
	imshow("Tracker", dst );
}

void Morphology_trackbars(char* wind)
{
  createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", wind, &morph_operator, max_operator, Morphology_Operations );
  createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", wind, &morph_elem, max_elem, Morphology_Operations );
  createTrackbar( "Morphology Kernel size:\n 2n +1", wind, &morph_size, max_kernel_size_m,Morphology_Operations );
}

void Dilation_trackbars(char* wind)
{	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", wind, &dilation_elem, max_elem, Dilation );
	createTrackbar( "Dilation Kernel size:\n 2n +1", wind, &dilation_size, max_kernel_size_d, Dilation );
}

void CreateDisplayWindows()
{
	//first window
	namedWindow("Tracker", CV_WINDOW_KEEPRATIO); //resizable
	cvMoveWindow("Tracker", 0, 0); // Position window
	cvResizeWindow("Tracker", 675, 700); // Resize window
}

void Initial_Capture(VideoCapture & capt)
{
	capt.read(tmp); //capture the i-3 th frame
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_p3);

	capt.read(tmp); //capture the i-2 th frame
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_p2);

	capt.read(tmp); //capture the i-1 th frame
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_p1);

	capt.read(tmp); //capture the i th frame
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_now);

	capt.read(tmp); //capture the i+1 th frame
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_n1);

	capt.read(tmp); //capture the i+2 th frame
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_n2);

	capt.read(tmp); //capture the i+3 th frame
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_n3);

}

void Rotate_Get_New_Capture(VideoCapture & capt)
{
	//shift the frames up by one:   p3 <--p2 <--p1 <--NOW <--n1 <--n2 <-- n3 <---NEW
	frame_p2.copyTo(frame_p3);
	frame_p1.copyTo(frame_p2);
	frame_now.copyTo(frame_p1);
	frame_n1.copyTo(frame_now);
	frame_n2.copyTo(frame_n1);
	frame_n3.copyTo(frame_n2);

	capt.read(tmp);  // get the new frame
	cvtColor(tmp,dst,CV_RGB2GRAY); //convert it to grayscale
	dst.copyTo(frame_n3);
}


void Threshold_AND()
{
	Mat tmp1, dst1, dst2, dst3, dst4;

	absdiff(frame_now, frame_p3, tmp);
	threshold(tmp,dst1 ,10.0 , 255, THRESH_BINARY);

	absdiff(frame_now, frame_p2, tmp);
	threshold(tmp,dst2 ,10.0 , 255, THRESH_BINARY);

	absdiff(frame_now, frame_n2, tmp);
	threshold(tmp,dst3 ,10.0 , 255, THRESH_BINARY);

	absdiff(frame_now, frame_n3, tmp);
	threshold(tmp,dst4 ,10.0 , 255, THRESH_BINARY);

	bitwise_and(dst1,dst2,tmp);
	bitwise_and(tmp,dst3,tmp1);
	bitwise_and(tmp1,dst4,tmp);
}


int Graph_Data(mglGraph *gr,  void *) // ticks, grid, axis
{
	//gr->SetTicks('x', 100); gr->SetTicks('y', 100);
	//gr->SetTicks('z', 100); // too low step of ticks


	//    	gr->Zoom(-100,1000,1200,-1000);

    mglData d(2);
    d.a[0] = -1000;
    d.a[1] = 4500;
    gr->XRange(d);

    d.a[0] = -500;
    d.a[1] = 3000;
    gr->YRange(d);

    d.a[0] = -50;
    d.a[1] = 3000;
    gr->ZRange(d);

	gr->Rotate(75,37);
	gr->Zoom(0,0,1500,-1000);
	gr->RecalcBorder();

    gr->Light(true);

    gr->FaceZ(0,0,0,3658, 1829,"g"); //Court Base
    gr->FaceX (1829,366, 0, 1097, 91, "e"); // Net

    // OutLines
    gr->Line (mglPoint(640,366,0), mglPoint(640+2378,366,0), "w");
    gr->Line (mglPoint(640,366,0), mglPoint(640,366+1097,0), "w");
    gr->Line (mglPoint(640,366+1097,0),mglPoint(640+2378,366+1097,0),  "w");
    gr->Line (mglPoint(640+2378,366+1097,0),mglPoint(640+2378,366,0),  "w");

    // Singles Out
    gr->Line (mglPoint(640,503,0), mglPoint(3018,503,0), "w");
    gr->Line (mglPoint(640,1326,0),mglPoint(3018,1326,0),  "w");

	// Internal Lines
    gr->Line (mglPoint(1189,366+137,0), mglPoint(1189,1326,0), "w");
    gr->Line (mglPoint(2469,503,0), mglPoint(2469,1326,0), "w");
    gr->Line (mglPoint(1189,914.5,0), mglPoint(2469,914.5,0), "w");

    // Particles
    for (unsigned int k = 0; k<ball_p.size(); k++) //origin of the plot and the points is not the same=> translate in x-y plane
	{
    	gr->Ball(mglPoint(ball_p[k].pos.x+640,ball_p[k].pos.y+366,ball_p[k].pos.z), 'b');
    }

    //Camera
	gr->Ball(mglPoint(cam.coors.x+640,cam.coors.y+366,cam.coors.z), 'r');

	Plot_Rays(gr); // Plot rays from camera to blobs
    return 0;
}

int process(VideoCapture& capture)
{
	//Dilation_trackbars("Tracker");   //Create trackbars for Dilation
	//Morphology_trackbars("Tracker");

	Rotate_Get_New_Capture(capture);
	Threshold_AND();
	morphologyEx( tmp,dst, operation, Morphology_element, Point(-1,-1), morph_iters);
	Do_Contours(dst,tmp); //stores the result in tmp

	imshow("Tracker", tmp);
	waitKey(200);

		//imshow("Thresholded", tmp);

		//GaussianBlur(tmp, dst, Size(3,5), 2.0, 2.0);  //other things we might want to try to smooth
		//threshold(dst,tmp,10.0 , 255, THRESH_BINARY);
		//dilate(tmp, dst, Dilation_element, Point(-1,-1), 1);


  return 0;
}

void Do_Contours(Mat & input, Mat & output)
{
	vector<vector<Point> > contours; // needed for finding contours and connected components
	vector<Vec4i> hierarchy; //needed for finding contours and connected components
	Mat tmp_cont = Mat::zeros(frame_now.size(), CV_8UC3); // get a new matrix for blobifying

	// clear the blob_observations vector
	blob_obs.clear();

	findContours(input, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE ); // find contours

	if( !contours.empty() && !hierarchy.empty() )  // draws contours
	 {
	        int idx = 0, mass_limit = 150 ;
	        for( ;idx >= 0; idx = hierarchy[idx][0] )
	        {
	        	if ((contourArea(contours[idx]) <= mass_limit) && (contourArea(contours[idx])>5)) //only pick those contours which have a small area
	            {
			  	    blob_observations obv;

	        		// parameters for ellipse
	        	    RotatedRect ellipse_bounding_box;
	        	    float Major_axis = 0.0, Minor_axis = 0.0, ellipse_area = 0.0;

	        	    ellipse_bounding_box = fitEllipse(contours[idx]); //fit an ellipse to the contour
	        	    Major_axis = ellipse_bounding_box.size.height;
	        	    Minor_axis = ellipse_bounding_box.size.width;
	        	    ellipse_area = (Major_axis*Minor_axis*pi)/4;

	        	    /*filter based on shape:
	        	     * 1. calculate delta = |contour_area - ellipse_area|/contour_area
	        	     * 2. r = Major_axis/Minor_axis
	        	     *
	        	     * for circular ball, we expect: delta -->0(+) && r --> 1(+)
	        	     */

	        	    float delta = (abs(contourArea(contours[idx]) - ellipse_area))/contourArea(contours[idx]);
	        	    float r = Major_axis/Minor_axis;

	        	    obv.center = ellipse_bounding_box.center;
	        	    obv.delta_area = delta;
	        	    obv.major_by_minor = r;
	        	    blob_obs.push_back(obv); //add the blob to the blob_observation array

        	    	drawContours( tmp_cont, contours, idx, Scalar(255,255,255), CV_FILLED, CV_AA, hierarchy,0 ); //draw the contour
        	    	ellipse( tmp_cont, ellipse_bounding_box.center, ellipse_bounding_box.size, 0, 0, 360, Scalar(0,0,255), 1, 8, 0 ); //draw the approx ellipse


	        	 // }
	            }
	        }}
	tmp_cont.copyTo(output);  //'returns' value to output (a Mat type array)
}

void Init_bounds(bounds &b) // Bounds for initialization of particles
{
	//position
	b.xmax = 3700;
	b.xmin = -1000;

	b.ymax = 2000;
	b.ymin = -500;

	b.zmax = 2000;
	b.zmin = -50;

	//velocity
	b.vxmax = 3000; //20 meter/s
	b.vxmin = -3000;

	b.vymax = 3000;
	b.vymin = -3000;

	b.vzmax = 3000;
	b.vzmin = -3000;
}

VideoCapture Init_Video(int argc, char** argv)
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
	capture.set(CV_CAP_PROP_CONVERT_RGB,1); //tell the capture to convert frames to RGB format
	Initial_Capture(capture);
	return capture;
}

void Process_Blobs() // Converts blob pixel xy to real world coordinates and stores it in global variable obsvrs
{

	obsvrs.camera = cam; // update the camera position
	obsvrs.blobs_pos.clear(); // clear blobs of the previous frame
	Point3d cam_world_coor, world_coor;

	for(unsigned int k = 0; k< blob_obs.size(); k++)
	{

		cam_world_coor = PixelXY_To_CamWorldCoor(blob_obs[k].center);
		world_coor = Get_world_coor(cam_world_coor, obsvrs.camera.coors, obsvrs.camera.theta, obsvrs.camera.pan, obsvrs.camera.tilt);

		obsvrs.blobs_pos.push_back(world_coor);
	}

}

void Plot_Rays(mglGraph *gr)
{
	mglPoint cam;
	cam.x = (float) obsvrs.camera.coors.x+640;
	cam.y = (float) obsvrs.camera.coors.y+366;
	cam.z = (float) obsvrs.camera.coors.z;

	for(unsigned int k = 0; k< obsvrs.blobs_pos.size(); k++)
	{
		mglPoint blob_k;
		blob_k.x = (float) obsvrs.blobs_pos[k].x+640;
		blob_k.y = (float) obsvrs.blobs_pos[k].y+366;
		blob_k.z = (float) obsvrs.blobs_pos[k].z;

		gr->Line(cam, blob_k,"kAO");
	}

}

void *mgl_fltk_thrd_tmp(void *)
{
	mglFlRun();
	return 0;
}



int main(int argc, char** argv)
{
	VideoCapture capture = Init_Video(argc, argv); // Initialize video
	CreateDisplayWindows(); //creates the windows for display

	bounds b;
	Init_bounds(b); //bounds for particle initialization
	ball_p = sample_uniformly(1000, b);  // sample 1000 particle uniformly in the bounds b

	mglGraphFLTK gr;
	gr.Window(argc,argv,Graph_Data,"ParticleBB");
	static pthread_t thrd; //create a thread for plotting
    pthread_create(&thrd, 0, mgl_fltk_thrd_tmp, 0);
//    pthread_detach(thrd);    // run window handling in the separate thread


	for(;;)
	{
		waitKey(5000);
		process(capture); // process the video
		Process_Blobs();
		gr.Update();
	}

	return 0;
}



/*
color masking:
    Scalar low = Scalar(25, 50, 50);
    Scalar upper = Scalar(30, 150, 150);

    Mat mask;
    inRange(frame1, low, upper, mask);
    imshow(window_name, mask);

 */
