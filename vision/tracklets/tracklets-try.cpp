/*
 * tracklets-try.cpp
 *
 *  Created on: Aug 1, 2011
 *      Author: Ankush Gupta
 */

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

//------------------------* Structures *-------------------------------

struct Mat_Color_Gray
{
	Mat col;
	Mat bin;
};

struct Supports
{
	int frame_index;
	int blob_index;
};


struct Velocity_Model
{
	Point2f v; // Velocity vector
	Point2f a; // Acceleration vector
	Point2f ref; // Reference point for model
	Velocity_Model()
	{
		v.x = 0;
		v.y = 0;
		a.x = 0;
		a.y = 0;
		ref.x = 0;
		ref.y = 0;
	}
};


struct 	Trajectory
{
Velocity_Model model;
vector<Supports> support;
float cost;
};

struct Seed_Triplet
{	int ref;
	vector <int> prev;
	vector <int> nxt;};

struct Idx_Dist // To store the index of blob and its distance from the reference blob
{
	int idx;
	float dist;

	Idx_Dist()
	{
		idx = -1;
		dist = -1.0;
	}

};


//--------------------------------x---x---x--------------------------------------

//--------------------- Globals-------------------------------------------------
Mat frame_p3, frame_p2, frame_p1,frame_n1, frame_n2,frame_n3, frame_now;
Mat frame_p3c, frame_p2c, frame_p1c,frame_n1c, frame_n2c,frame_n3c, frame_nowc;
Mat tmp, dst;
Mat_Color_Gray wnd[15];
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
bool initialized = false;

const double pi  = 3.1415926535897;
const int MAX_DIST = 30;


/** Function Headers */
void CreateDisplayWindows();
void Initial_Capture(VideoCapture &);
void Rotate_Get_New_Capture(VideoCapture &);
Mat Threshold_AND();
vector <Point2f> Do_Contours(Mat);
vector <Point2f> Blobify(Mat);
vector <Seed_Triplet> Find_Seed_Triplet(vector <Point2f>, vector <Point2f>, vector <Point2f>);
Velocity_Model Fit_Model(int, int, int, Point2f, Point2f, Point2f);
Point2f Position_Estimate(Velocity_Model, Point2f, float, float);
vector <Supports> Find_Support(vector <vector <Point2f> >, Velocity_Model, Point2f, int, float);
float Get_Cost(vector <vector <Point2f> >, Velocity_Model, int, Point2f, float);
void Get_Support_Frames(vector <Supports> S, int &k_min, int &blob_num_min, int &k_max, int &blob_num_max, int &k_mid, int &blob_num_mid);
//-------------------------------------------------------------

Mat Morphology_element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
Mat Dilation_element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

void CreateDisplayWindows()
{
	//first window
	namedWindow("Tracker", CV_WINDOW_KEEPRATIO); //resizable
	cvMoveWindow("Tracker", 675, 0); // Position window
	cvResizeWindow("Tracker", 675, 700); // Resize window
}

void Initial_Capture(VideoCapture & capt)
{
	capt.read(tmp); //capture the i-3 th frame
	tmp.copyTo(frame_p3c);
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_p3);

	capt.read(tmp); //capture the i-2 th frame
	tmp.copyTo(frame_p2c);
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_p2);

	capt.read(tmp); //capture the i-1 th frame
	tmp.copyTo(frame_p1c);
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_p1);

	capt.read(tmp); //capture the i th frame
	tmp.copyTo(frame_nowc);
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_now);

	capt.read(tmp); //capture the i+1 th frame
	tmp.copyTo(frame_n1c);
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_n1);

	capt.read(tmp); //capture the i+2 th frame
	tmp.copyTo(frame_n2c);
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_n2);

	capt.read(tmp); //capture the i+3 th frame
	tmp.copyTo(frame_n3c);
	cvtColor(tmp,dst,CV_RGB2GRAY);
	dst.copyTo(frame_n3);

	initialized = true;
}

void Rotate_Get_New_Capture(VideoCapture & capt)
{
	//shift the frames up by one:   p3 <--p2 <--p1 <--NOW <--n1 <--n2 <-- n3 <---NEW

	//grayscale
	frame_p2.copyTo(frame_p3);
	frame_p1.copyTo(frame_p2);
	frame_now.copyTo(frame_p1);
	frame_n1.copyTo(frame_now);
	frame_n2.copyTo(frame_n1);
	frame_n3.copyTo(frame_n2);

	//color frames
	frame_p2c.copyTo(frame_p3c);
	frame_p1c.copyTo(frame_p2c);
	frame_nowc.copyTo(frame_p1c);
	frame_n1c.copyTo(frame_nowc);
	frame_n2c.copyTo(frame_n1c);
	frame_n3c.copyTo(frame_n2c);

	capt.read(tmp);  // get the new frame
	tmp.copyTo(frame_n3c);
	cvtColor(tmp,dst,CV_RGB2GRAY); //convert it to grayscale
	dst.copyTo(frame_n3);
}

Mat Threshold_AND(Mat fr[5])
{
	/*fr[2] is the working frame
	 * fr[0], fr[1] are the i-3, i-2 frames
	 * fr[3], fr[4] are the i+2, i+3 frames
	 */

	Mat tmp1, tmp2, dst[4];
	int j = 0;

	for(int i = 0; i<5; i++)
	{
		if (i==2)
			continue;

		absdiff(fr[2], fr[i], tmp1);
		threshold(tmp1,dst[j] ,10.0 , 255, THRESH_BINARY);
		j++;
	}

	bitwise_and(dst[0],dst[1],tmp1);
	bitwise_and(tmp1,dst[2],tmp2);
	bitwise_and(tmp2,dst[3],tmp1);

	return tmp1;
}

void Init_Window(VideoCapture capture)
{
	Initial_Capture(capture);

	Mat th[] = {frame_p3, frame_p2, frame_now, frame_n2, frame_n3 }; // pack the frames into an array for thresholding

	Threshold_AND(th).copyTo(wnd[0].bin); // store the  ANDed and thresholded frame
	frame_nowc.copyTo(wnd[0].col); //store the color frame

	for(int i = 1; i<15; i++) // size of window is 15
	{
		Rotate_Get_New_Capture(capture);
		Mat thr[] = {frame_p3, frame_p2, frame_now, frame_n2, frame_n3 }; // pack the frames into an array for thresholding
		Threshold_AND(thr).copyTo(wnd[i].bin);
		frame_nowc.copyTo(wnd[i].col);
	}
}

void Get_Next_Window(VideoCapture capture)
{
	for(int i = 0; i<14; i++) //Push up
	{
		wnd[i+1].col.copyTo(wnd[i].col);
		wnd[i+1].bin.copyTo(wnd[i].bin);
	}

	Rotate_Get_New_Capture(capture); // get a new frame
	Mat th[] = {frame_p3, frame_p2, frame_now, frame_n2, frame_n3 }; // pack the frames into an array for thresholding

	Threshold_AND(th).copyTo(wnd[14].bin);
	frame_nowc.copyTo(wnd[14].col); //store the color frame

}

vector <Seed_Triplet> Find_Seed_Triplet(vector <Point2f> p, vector <Point2f> curr, vector <Point2f> n, float d_thresh)
{
	/* INPUT: Vector of blob positions from 3 consecutive frames p->curr->n
	 * OUTPUT: Vector of Seed triplet corresponding to each blob in curr
	 *
	 */
	vector <Seed_Triplet> triplets;
	triplets.resize(curr.size());

	if(!curr.empty())
		for(unsigned int i = 0; i< curr.size(); i++)
		{

			triplets[i].ref = i;

			if(!p.empty())
				for(unsigned int k = 0; k< p.size(); k++) // check in the previous frame
				{
					if(norm(p[k] - curr[i]) <= d_thresh && norm(p[k] - curr[i]) >= 2.0 )
						triplets[i].prev.push_back(k);
				}

			if(!n.empty())
				for(unsigned int k = 0; k< n.size(); k++) // check in the next frame
				{
					if(norm(n[k] - curr[i]) <= d_thresh && norm(n[k] - curr[i]) >= 2.0)
						triplets[i].nxt.push_back(k);

				}

			if (triplets[i].nxt.size()<1 || triplets[i].prev.size()<1) // Invalidate the blob if no triplet corresponding to that blob is found
				triplets[i].ref = -1;

		}
	return triplets;
}

vector <Supports> Find_Support(vector <vector <Point2f> > blob_pos, Velocity_Model m, Point2f p0, int k0, float d_thresh = 5.0)
{
	/*Input: 1. blob_pos: vector of vector of all blob positions in a window
	 * 		 2. m       : velocity model to be used for finding supports
	 *       3. p0      : reference position used in velocity model
	 *       4. k0		: frame number of the reference point used in velocity model
	 *       5. d_thresh: Max distance for accepting a blob as a support for the model
	 *
	 *Output: vector of type Supports, which wraps the frame number and the index of blob position in that frame number
	 *		  which is a support for the model
	 */



	vector <Supports> spprts;

	for(unsigned int i = 0; i< blob_pos.size(); i++) // for all frames in the window
	{
		float min_dist = 100000; //Arbitarily large value
		float min_blob_index = -100; //absurd index



		if(!blob_pos.empty())
		{for(unsigned int j = 0; j< blob_pos[i].size(); j++) // for all blob position in window frame
		{

			Point2f est = Position_Estimate(m, p0, k0, i); // Estimate the position in ith frame using velocity model
			float d = norm(est-blob_pos[i][j]);   // get the distance of actual blob from the estimate
			if (d <= d_thresh)
				if(d <= min_dist) // incase of multiple supports found in the same frame, choose the one with smallest distance
				{
					min_dist = d;
					min_blob_index = j;
				}
		}}

		if(min_dist != 100000 && min_blob_index >=0)
		{	Supports s;
			s.blob_index = min_blob_index;
			s.frame_index = i;
			spprts.push_back(s);
		}

	}
	return spprts;
}

void Get_Support_Frames(vector <Supports> S, int &k_min, int &blob_num_min, int &k_max, int &blob_num_max, int &k_mid, int &blob_num_mid)
{
	/* INPUT : Support vector S
	 * OUTPUT: k_min, k_max, k_mid (values returned by reference)
	 */


	int min = 10000, max = -1000, mid = 7;
	if(!S.empty()) //Note Size of support is guanranteed to be >= 3 as the seed triplet(3 frames) is always a support
	{
		for(unsigned int k = 0; k< S.size(); k++)
		{
			if(S[k].frame_index < min)
				{
				min = S[k].frame_index;
				blob_num_min = S[k].blob_index;
				}
			if(S[k].frame_index > max)
				{
				max = S[k].frame_index;
				blob_num_max = S[k].blob_index;
				}
		}

		int v = 100; //absurd initialization
		for(unsigned int k = 0; k< S.size(); k++)
		{
			int val = abs(abs(max - S[k].frame_index)-abs(S[k].frame_index - min));
			if(val < v)
				{
				mid = S[k].frame_index;
				blob_num_mid = S[k].blob_index;
				}
		}
	}

	k_min = min;
	k_max = max;
	k_mid = mid;
}



Velocity_Model Fit_Model(int k1, int k2, int k3, Point2f p1, Point2f p2, Point2f p3)  // k1<k2<k3, p1-->p2-->p3 (time succession)
{
	Velocity_Model m;

	m.a = 2*(((k2-k1)*(p3-p2))-((k3-k2)*(p2-p1)))*(1.0/(float)((k2-k1)*(k3-k2)*(k3-k1)));
	m.v = (((p2-p1)*(1.0/(float)(k2-k1))) - ((k2-k1)*(m.a)*0.5));
	m.ref = p2;
	return m;
}

Point2f Position_Estimate(Velocity_Model m, Point2f p1, float k1, float k) // Returns estimated position given initial position and velocity model
{
	Point2f est;
	est = p1 + m.v*(k-k1) + m.a*(k-k1)*(k-k1)*0.5;

	return est;
}

float Get_Cost(vector <vector <Point2f> > blobs_pos, Velocity_Model m, int ref_frame_index, Point2f ref_pos, float d_th)
{
	float C = -100; // initialize to absurd value
	if(!blobs_pos.empty())
	{	C = 0.0;
		for(unsigned int k = 0; k < blobs_pos.size(); k++)
		{
			if(!blobs_pos[k].empty())
			{
			for(unsigned int j = 0; j< blobs_pos[k].size(); j++)
				{
				Point2f est = Position_Estimate(m, ref_pos, ref_frame_index, k);
				float d = norm(est - blobs_pos[k][j]);
				if(d < d_th)
					C+= d;
				else C+= d_th;
				}
			}
		}

	}

	return C;

}


Trajectory Get_Optimised_Trajectory(vector <vector <Point2f> > blobs_pos, Point2f prev, Point2f ref, Point2f nxt, float d_th)
{
	int k_min_prev = 6, k_mid_prev = 7, k_max_prev = 8;
	int k_min_curr = k_min_prev, k_max_curr = k_max_prev, k_mid_curr = k_mid_prev;
	int b_min, b_mid, b_max;
	float cost_prev = 0, cost_curr = -1;


	Velocity_Model m = Fit_Model( 6, 7, 8, prev, ref, nxt), m_prev;
	cost_curr = Get_Cost(blobs_pos, m, 7, ref, d_th);


	vector <Supports> S = Find_Support(blobs_pos, m, m.ref, k_mid_prev, 5.0);
	vector<Supports> S_prev;


	do
	{
		//Find new seed triplets
		k_min_prev = k_min_curr;   k_max_prev = k_max_curr,  k_mid_prev  = k_mid_curr;
		Get_Support_Frames(S, k_min_curr, b_min, k_max_curr, b_max, k_mid_curr, b_mid);

		//Fit a model to new seed triplets
		m_prev = m;
		m = Fit_Model(k_min_curr, k_mid_curr, k_max_curr, blobs_pos[k_min_curr][b_min], blobs_pos[k_mid_curr][b_mid], blobs_pos[k_max_curr][b_max]);

		// Get the cost of the new model
		cost_prev = cost_curr;
		cost_curr = Get_Cost(blobs_pos, m, k_mid_curr, blobs_pos[k_mid_curr][b_mid], d_th);

		S_prev = S;
		S =Find_Support(blobs_pos, m, blobs_pos[k_mid_curr][b_mid], k_mid_curr);

	}while((cost_curr < cost_prev)&&((k_min_prev != k_min_curr)||(k_max_prev != k_max_curr)));

	// use "prev" : they represent the optimized parameters
	Trajectory T;
	T.model = m_prev;
	T.support = S_prev;
	T.cost = cost_prev;

	return T;

}


Trajectory Get_Best_Trajectory(vector <vector <Point2f> > wnd_blobs, vector <Seed_Triplet> seed_trip)
{
	// seed_trip contains the index of blobs of wnd_blobs in the frames 6 and 8

	vector <Trajectory> trajs;
	trajs.resize(seed_trip.size());

	float min_cost = 1000000;
	int idx = 0;

	if (!seed_trip.empty())
	{
		for(unsigned int k = 0; k < seed_trip.size(); k++)
		{
			if(seed_trip[k].ref >= 0) // do only for valid seeds
				if(!seed_trip[k].prev.empty())
					for(unsigned int j = 0; j < seed_trip[k].prev.size(); j++)
						if(!seed_trip[k].nxt.empty())
							for(unsigned int i = 0; i< seed_trip[k].nxt.size(); i++)
							{
								Trajectory t = Get_Optimised_Trajectory(wnd_blobs, wnd_blobs[6][seed_trip[k].prev[j]], wnd_blobs[7][seed_trip[k].ref], wnd_blobs[8][seed_trip[k].nxt[i]], 5.0);
								trajs[k] = t;

								if(t.cost < min_cost)
								{
									idx = k;
									min_cost = t.cost;
								}
						    }
			}}

	if(!trajs.empty())
		return trajs[idx];
	else
	{
		Trajectory trj;
		trj.cost = -1000;
		return trj;
	}



}

void Process_Window(Mat_Color_Gray wnd[15])
{
	/*
	 * Finds (x,y) for the blobs for all frames in the given window
	 * returns 1 if a seed triplet is found in the frame 6,7,8 of the window
	 * otherwise returns -1 (indicating failure)
	 */

	// Central frame of any given window is frame number 7
	vector <vector <Point2f> > wind_blobs(15);

	for(int k = 0; k<15; k++) // get the x,y position of blobs in each frame in the window
	{
		vector <Point2f> b = Blobify(wnd[k].bin);
		if(!b.empty())
			wind_blobs[k] = b;
		else
			wind_blobs[k].clear();
	}


	vector <Seed_Triplet> s_t = Find_Seed_Triplet(wind_blobs[6], wind_blobs[7], wind_blobs[8],30.0); //find the seed triplets
	vector <Seed_Triplet> seed_trip;
	if (!s_t.empty())
		seed_trip = s_t;
	else
		seed_trip.clear();

	Trajectory t = Get_Best_Trajectory(wind_blobs, seed_trip);
	if(t.cost == -1000)
		return;


	/****************** Debug ************************************
	 for(int k = 0; k<15; k++) // Print the x,y position of blobs
	{
		cout<<"Frame # "<<k<<endl;
		for(unsigned int j = 0; j< wind_blobs[k].size(); j++)
				cout<< "x: "<< wind_blobs[k][j].x<<" y: "<<wind_blobs[k][j].y<<endl;
	}
	************************************************************/
	// Display the blobs superposed on the frame

	Mat tmp_cont = Mat::zeros(frame_now.size(), CV_8UC3);
	Mat Mat_ones = Mat::ones(frame_now.size(), CV_8UC1);

	Mat temp1, temp2, temp3, dst;

	for(unsigned int j = 0; j< wind_blobs[7].size(); j++) // Paint the blobs
	{	ellipse( tmp_cont, wind_blobs[7][j], Size(3,3), 0, 0, 360, Scalar(255,255,0), -1, 8, 0 );
	}

	for(float n = -3; n<3; n+= 0.1) // Paint the trajectory
		ellipse( tmp_cont,Position_Estimate(t.model, t.model.ref, 7, 7+n) , Size(1,1), 0, 0, 360, Scalar(0,0,255), -1, 8, 0 );


	cvtColor(tmp_cont, temp1, CV_RGB2GRAY); //convert it to grayscale
	threshold (temp1, temp2, 10.0 , 255, THRESH_BINARY);
	threshold (Mat_ones, temp1, 0.5 , 255, THRESH_BINARY);
	absdiff( temp1, temp2, dst);

	wnd[7].col.copyTo(temp3, dst);

	imshow("Tracker", temp3+tmp_cont);
	waitKey(100);
	/*********************************************************************/

}

vector <Point2f> Blobify(Mat input) // returns a vector of blob positions found in the given frame
{
	Mat tmp0, tmp1;
	input.copyTo(tmp0);

	morphologyEx( tmp0,tmp1, operation, Morphology_element, Point(-1,-1), morph_iters);
	return Do_Contours(tmp1);
}

vector <Point2f> Do_Contours(Mat input)
{
	vector<vector<Point> > contours; vector<Vec4i> hierarchy;
	vector <Point2f> blobs;

	Mat tmp_cont = Mat::zeros(frame_now.size(), CV_8UC3); // get a new matrix for blobifying
	Mat inp;	input.copyTo(inp);

	findContours(inp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE ); // find contours

	if( !contours.empty() && !hierarchy.empty() )  // draws contours
	 {
	        for( int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
	        {
	        	if ((contourArea(contours[idx])>0) && (contourArea(contours[idx]) < 150)) //only pick those contours which have a meaningful area
	            {
			  	    // parameters for ellipse
	        	    RotatedRect ellipse_bounding_box;
	        	    float Major_axis = 0.0, Minor_axis = 0.0, ellipse_area = 0.0;

	        	    try
						{ellipse_bounding_box = fitEllipse(contours[idx]);} //fit an ellipse to the contour
	        	    catch(...)
						{continue;}

	        	    Major_axis = ellipse_bounding_box.size.height;
	        	    Minor_axis = ellipse_bounding_box.size.width;
	        	    ellipse_area = (Major_axis*Minor_axis*pi)/4;

	        	    float delta = (abs(contourArea(contours[idx]) - ellipse_area))/contourArea(contours[idx]);
	        	    float r = Major_axis/Minor_axis;

	        	    if (r < 2) // filter based on shape and fullness
	        	    {
	        	    	blobs.push_back(ellipse_bounding_box.center); //store the center
	        	    }
	            } // Size filtering
	        }}
	return blobs;
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
	cout<<"Frame Rate: "<<capture.get(CV_CAP_PROP_FPS)<<endl;
	return capture;
}

int main(int argc, char** argv)
{

	VideoCapture capture = Init_Video(argc, argv); // Initialize video
	CreateDisplayWindows(); //creates the windows for display

	cout<<"Total Number of frames: "<<capture.get(CV_CAP_PROP_FRAME_COUNT)<<endl;

	Init_Window(capture);

	for(;;)
	{
	Process_Window(wnd);
	Get_Next_Window(capture);
	}
	return 0;
}


/*int process(VideoCapture& capture)
{
	Mat fr_sum;

	Rotate_Get_New_Capture(capture);
	Threshold_AND();
	morphologyEx( tmp,dst, operation, Morphology_element, Point(-1,-1), morph_iters);
	Do_Contours(dst,tmp); //stores the result in tmp

	fr_sum = tmp+ frame_nowc;

	imshow("Tracker", fr_sum);
	waitKey(200);
	return 0;
}*/




/*
color masking:
    Scalar low = Scalar(25, 50, 50);
    Scalar upper = Scalar(30, 150, 150);

    Mat mask;
    inRange(frame1, low, upper, mask);
    imshow(window_name, mask);

 */
