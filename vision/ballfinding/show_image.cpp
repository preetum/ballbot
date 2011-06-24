#include <math.h>
#include <sys/timeb.h>

#include <cv.h>
#include <highgui.h>

using namespace cv;

const int slider_max = 200;
int brightness_slider = 100,
     contrast_slider = 100;
double gain = 1.0;
int bias = 0;
Mat resized_img, display_img;

void print_time(string label)
{
	static struct timeb prev = {0,0};
	struct timeb cur;
	double diff = 0;
	ftime(&cur);
	if (prev.time) {
		diff  =  cur.time    - prev.time;
		diff += (cur.millitm - prev.millitm)/1000.0;
	}
	fprintf(stderr, "%30s  start = %d.%-3hu (+%5.3f)\n",
		label.c_str(), (int)cur.time, cur.millitm, diff);
	prev = cur;
}

void refresh_display (void)
{
     print_time("computing image");

/* Test 1 - slowest, ~120ms
     for( int y = 0; y < display_img.rows; y++ )
     { for( int x = 0; x < display_img.cols; x++ )
	  { for( int c = 0; c < display_img.channels(); c++ )
	       { display_img.at<Vec3b>(y,x)[c] = 
			 saturate_cast<uchar>(
			      gain * resized_img.at<Vec3b>(y,x)[c] + bias);
	       }
	  }
     }
//*/     
//* Test 2 - fastest, ~20ms
     resized_img.convertTo(display_img, -1, gain, bias);
//*/
/* Test 3 - slower, ~90ms
     Size size = display_img.size();
     int channels = display_img.channels();
     if (display_img.isContinuous())
     {
	  size.width *= size.height;
	  size.height = 1;
     }
     size.width *= channels;

     for (int i = 0; i < size.height; i += 1)
     {
	  const uchar* sptr = resized_img.ptr<uchar>(i);
	  uchar* dptr = display_img.ptr<uchar>(i);

	  for (int j = 0; j < size.width; j += 1)
	  {
	       dptr[j] = saturate_cast<uchar>(gain * sptr[j] + bias);
	  }
     }
//*/
     print_time("displaying image");
  
     imshow( "Display Image", display_img );
     print_time("end display image");
}

void on_trackbar (int value, void*)
{
     bias = 3*(brightness_slider - 100)/2;
     gain = pow(10, (double)(contrast_slider - 100)/slider_max);
     refresh_display();
}


int main( int argc, char** argv )
{
     Mat image;
     image = imread( argv[1], 1 );
  
     if( argc != 2 || !image.data )
     {
	  printf( "No image data \n" );
	  return -1;
     }

     resize(image, resized_img, Size(), 0.8, 0.8);
  
     namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
     createTrackbar("Brightness", "Display Image", &brightness_slider,
		    slider_max, on_trackbar);
     createTrackbar("Contrast", "Display Image", &contrast_slider,
		    slider_max, on_trackbar);
     refresh_display();
     imshow( "Display Image", display_img );
  
     waitKey(0);
  
     return 0;
}
