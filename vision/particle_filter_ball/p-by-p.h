/*
 * p-by-p.cpp
 *
 *  Created on: Jul 20, 2011
 *      Author: Ankush Gupta
 */


#include <math.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>



using namespace std; //needed for cout, cin to work
using namespace cv;

#define NUM_PARTICLES 1000
#define pi 3.1415926535897

Point3d cross(Point3d, Point3d);
double dist (Point3d, Point3d, Point3d);
const double fps  = 60.0;  // 60 frames per second

vector<double> x;
vector<double> y;
vector<double> z;



//-------------------------Structures-----------------------------------------
struct ball_particle
{
	Point3d pos; // position
	Point3d v; // velocity
};

struct cam_pos
{ /******** Published by Localization *********************
   *x,y,z are in centimeters
   *theta, pan, tilt are in radians
   ********************************************************/
	Point3d coors;
	float  theta, pan, tilt;
	cam_pos() // Initialized for capture1...backcorner.avi
	{

		coors.x = 2316;
		coors.y = -30.5;
		coors.z = 33; // Camera is 33cm above the ground
		theta = (120*pi)/180;
		pan = 0;
		tilt = 0;
	}

};
cam_pos cam;

struct weighed_particle{		//encapsulate ball_particle and weight in one unit
	ball_particle part;
	double w;
};

struct bounds  //defines the initialization bounds
{
	float xmin, xmax, ymin, ymax, zmin, zmax, vxmin, vxmax, vymin, vymax, vzmin, vzmax;
};

struct blob_observations //structure for returning ball candidates
{
	Point2f center;
	float delta_area;
	float major_by_minor;
};

struct observations // Holds camera position and blobs position x,y,z
{
	cam_pos camera;
	vector <Point3d> blobs_pos;
};


struct camera_intrinsics  //stores camera intrinsics
{	// TODO: Add undisortion parameters
	double fx, fy, cx, cy;

	camera_intrinsics()
	{
	fx = 522.1019521747;
	cx = 299.0472018951;
	fy = 524.4051288723;
	cy = 242.6572277189;
	}
} cam_intr;

/*void Init_Plot() // Gnuplot Initialization
{
	//Reset
	rgb_plot.reset_plot();

	// Set the labels
	rgb_plot.set_xlabel("X");
	rgb_plot.set_ylabel("Y");
	rgb_plot.set_zlabel("UP = Z");

	// Set the range
	rgb_plot.set_xrange(-500,2500);
	rgb_plot.set_yrange(-50,1500);
	rgb_plot.set_zrange(-100,10000);

	rgb_plot.set_style("points");
}*/
//------------------------x-Structures-x---------------------------------------

Point3d PixelXY_To_CamWorldCoor(Point2f pixel_pos)
{
	/***************************  Important Note *************************
		 * Since we want the world coordinates (3 dimensional), from 2D pixel position
		 * information, we can only determine the line where the corresponding point can be.
		 *
		 * Hence, I arbitarily chose z_cam_world = 100 (cm). hence, this function returns the point
		 * [x_cam_world, y_cam_world, 100] corresponding to a given pixel postion on the image plane.
		 *
		 * To get x_cam_world or y_cam_world for any other z_cam_world, just scale:
		 * x_new = x_cam_world(for z=100)/100*new_z
		 * y_new = y_cam_world(for z=100)/100*new_z
		 *
		 *
		 * ----------------- Coordinates Axes --------------------------------
		 * 			   ____
		 *  Camera--> |	   O---------> z-axes (Front)
		 * 			  |____|\
		 * 				|| | \
		 * 				|| |  \
		 * 				|| V   V x-axis
		 *               y-axis
		 *               (Down)
		 *
		 ***********************************************************************/
	Point3d cam_world_coor;
	cam_world_coor.z = 100.00;  //arbitary; 100 centimeters
	cam_world_coor.y = (double)((pixel_pos.y -cam_intr.cy)*(cam_intr.fy/cam_world_coor.z));
	cam_world_coor.x = (double)((pixel_pos.x -cam_intr.cx)*(cam_intr.fx/cam_world_coor.z));

	return cam_world_coor;
}

Point3d Get_world_coor(Point3d cam_world_coor, Point3d camera_pos, double heading, double pan, double tilt)
{
	/*Returns position of a point in World Frame
	 *
	 *Assuming:
	 *
	 * cam_coor is the position of a point in camera's frame
	 * camera_pos is the positi6on of the camera in the world frame
	 *
	 * linear units: Centimeters
	 * angluar units: radians
	 *
	 * 	 1. that height of the camera in the world is fixed = 33 above the gound.
	 * 	 2. that roll angle = 0
	 */

	Point3d w_coor;
	double phi = heading + pan;
	double theta = tilt;

	double cos_phi = cos(phi), sin_phi = sin(phi), cos_t = cos(theta), sin_t = sin(theta);


	w_coor.x = sin_phi*cam_world_coor.x + cos_phi*sin_t*cam_world_coor.y + cos_phi*cos_t*cam_world_coor.z + camera_pos.x;
	w_coor.y = (-1)*cos_phi*cam_world_coor.x + sin_phi*sin_t*cam_world_coor.y + sin_phi*cos_t*cam_world_coor.z + camera_pos.y;
	w_coor.z  = (-1)*cos_t*cam_world_coor.y + sin_t*cam_world_coor.z + camera_pos.z;

	return w_coor;
}

vector <ball_particle> sample_uniformly(int n, bounds b ) //Returns a vector of size n, with uniformly sampled "ball_particles"
{
	vector <ball_particle> vect_ball_parts;
	RNG r;
    double rnd;


	for(int i = 0; i< n; i++) //for all particles in p
	{

		ball_particle prtcl;

		rnd = r.uniform((double)0, (double)1);
		prtcl.pos.x = b.xmin + (rnd*(b.xmax - b.xmin));

		rnd = r.uniform((double)0, (double)1);
		prtcl.pos.y = b.ymin + (rnd*(b.ymax - b.ymin));

		rnd = r.uniform((double)0, (double)1);
		prtcl.pos.z = b.zmin + (rnd*(b.zmax - b.zmin));

		rnd = r.uniform((double)0, (double)1);
		prtcl.v.x = b.vxmin + (rnd*(b.vxmax - b.vxmin));

		rnd = r.uniform((double)0, (double)1);
		prtcl.v.y = b.vymin + (rnd*(b.vymax - b.vymin));

		rnd = r.uniform((double)0, (double)1);
		prtcl.v.z = b.vzmin + (rnd*(b.vzmax - b.vzmin));

		vect_ball_parts.push_back(prtcl); //add the particle to the vector
	}

	return vect_ball_parts;
}


//----------------------------------------- Particle Class --------------------------------------------
class particles
{
	  /* Provides convenience methods over a set of particles
	   *	1. particles is a 1D array of type ball_particles
	   */

public:
	vector <weighed_particle> prtcls; //vector for particles

	particles() // overloaded constructor
	{	}

	particles(vector<ball_particle> vect_ball_partcls)		//costructor: Expects a vector of ball_particles as input
	{
		prtcls.resize(vect_ball_partcls.size());
		fill_weighedparts_with_ballparts(vect_ball_partcls);
		weigh_uniformly(prtcls);
	}

	void fill_weighedparts_with_ballparts(vector<ball_particle> vect_ball_partcls)
	{														// fills the weighed_particle vector with ball_particles recieved in initialization
		for(unsigned int i = 0; i< vect_ball_partcls.size(); i++)
			prtcls[i].part = vect_ball_partcls[i];
	}

	void weigh_uniformly(vector <weighed_particle> & p)		// Assigns uniform weight to each particls in the vector prtcls
	{
		int num_particles = p.size();
		double w = 1/(double)num_particles;
		for(int i = 0; i< num_particles; i++)
			p[i].w = w;
	}

	double get_weight_sum()  // Returns the sum of the weights
	{
		double w_sum = 0.0;
		for(unsigned int i = 0; i<prtcls.size() ; i++) //find the sum of the weights
			w_sum+= prtcls[i].w;
		return w_sum;
	}

	void normalize()		// Normalizes the weights of the particles in the vector prtcls
	{
		double w_sum = get_weight_sum();
		for(unsigned int i = 0; i<prtcls.size() ; i++) //normalize...assign
			prtcls[i].w = prtcls[i].w/w_sum;
	}


	vector <weighed_particle> resample() //Probabilistic Robotics, Thrun et. all, pg: 110
	{
		normalize(); //  normalize the weights of prtcls

		double weights_sum = get_weight_sum();
		if (weights_sum > 1.0001 || weights_sum < 0.9999)
			fprintf(stderr, "Bad Distribution: Sum of weights exceeds tolerances \n");

		vector <weighed_particle> X;
		X.clear();

		int M = prtcls.size();

		RNG rng;
		float r = (rng.uniform((double)0, (double)1))*(1/(float)M); //r is a random number between 0 and (1/number_of_particles)
		double c = prtcls[0].w; //initialize sum of weights to the weight of the first particle
		int i = 0; //is a counter

		for(int m = 0; m < M; m++)
		{
			float U = r + (m-1)*(1/(float)M);
			while(U > c)
			{
				i = i+1;
				c = c + prtcls[i].w;
			}
			X.push_back(prtcls[i]);
		}

		weigh_uniformly(X); //  weigh uniformly the resampled particles
		return X;
	}

};



//------------------------------------------ Particle Filter Class ----------------------------------
class ParticleFilter
{
	unsigned int numParticles;
	int t;


public:

	particles p;

	ParticleFilter(bounds init_bounds, unsigned int n = 1000)  //n is the number of particles
	{
		t = 0; // start time at 0
		numParticles = n;
		vector <ball_particle> vect_ball_partcls = sample_uniformly(n, init_bounds); // sample uniformly
		p = particles(vect_ball_partcls); // initialize the particles uniformly
	}

	void Move_Particles(void (*Motion)(particles &, float t))
	{
		Motion(p, (1.0/fps)); //moves each particle by fixed amount + noise| particles changed in place
		t += 1;  // t passes by one
	}

	particles get_beliefs()
	{
		return p;
	}

	void observe(observations obsv, double (*prob_func)(observations, ball_particle))
	{
		// update beliefs:  P(X_t | e_1:t, e') ~ P(e'|X_t) * P(X_t | e_1:t)
		// Changes the particle distribution, by updating the particles according to the observation

		for(unsigned int i = 0; i <p.prtcls.size(); i++)
		{
			p.prtcls[i].w = prob_func(obsv, p.prtcls[i].part);

		}
		vector <weighed_particle> resampled = p.resample(); //resample based on the new weights
		p.prtcls = resampled; // see: http://www.cplusplus.com/reference/stl/vector/operator=/ for '=' operator for <vectors>
	}

};

Point3d cross(Point3d v1, Point3d v2) // Calculates the cross product of 2 vectors
{
	Point3d u;
	u.x = (v1.y*v2.z - v1.z*v2.y);
	u.y = (v1.z*v2.x - v1.x*v2.z);
	u.z = (v1.x*v2.y - v1.y*v2.x);

	return u;
}


double dist (Point3d p1, Point3d p2, Point3d p3)
{
	/* Calculates the distance of p3 from line joining p1 and p2:
	 *
	 *				  p3
	 *				  |
	 *				  |--> this  distance = D
	 *			p1------------------p2
	 *
	 * D = |(p3- p1)X (p3-p2)| / |p2-p1|
	 */

	double D = -1; // initialize to absurd value
	D = norm(cross((p3-p1),(p3-p2)))/norm(p2-p1);

	return D;
}



double probDistNormal(observations obs, ball_particle p)
{
	double prob = 0.0;

	for(unsigned int i = 0; i < obs.blobs_pos.size(); i++ )
	{
		prob+= 1.0/dist(obs.camera.coors, obs.blobs_pos[i], p.pos);
	}

	return prob;
}


void MoveParticlesConstantAcceleration(particles & p, float t)
{
	/* Changes particles to reflect their motion
	 * t: is the time to be used in the calculations
	 */

	RNG r; // opencv random number generator
	double g = -980.7; //acceleration = 9.807 m/s (source: wolframalpha.com)
	for(unsigned int i =0; i< p.prtcls.size(); i++)
	{
		// Deterministic: Physical Model + Gaussian Noise
		if (abs(p.prtcls[i].part.pos.z) >10)
			p.prtcls[i].part.v.z += g*t; // accelerate only if far from gorund

		p.prtcls[i].part.v.z += r.gaussian((double) 30);
		p.prtcls[i].part.v.y += r.gaussian((double) 20);
		p.prtcls[i].part.v.x += r.gaussian((double) 20);

		p.prtcls[i].part.pos.x += p.prtcls[i].part.v.x*t + r.gaussian((double) 10.0);
		p.prtcls[i].part.pos.y += p.prtcls[i].part.v.y*t +  r.gaussian((double) 10.0);
		p.prtcls[i].part.pos.z += p.prtcls[i].part.v.z*t + (0.5*(g*t)*t)+ r.gaussian((double) 10.0); // Downwards accelerated motion (in the -Z direction)
    }
}

