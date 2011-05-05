import cv
import numpy as np

import robot

def get_pixel_center(frame, threshold=254):
	'''
	Returns the average pixel location of all pixels that are greater
	than threshold.
	
	frame is a 1-channel image
	'''
	xsum = 0
	ysum = 0
	count = 0
	
	height = frame.height
	width = frame.width
	#pxPerRow = frame.widthStep
	#data = frame.imageData;

	for i in xrange(0, height):
		for j in xrange(0, width):
			#a = np.asarray(frame)
			pixel = 0#a[i,j]

			if (pixel > threshold):
				xsum += j
				ysum += i
				count += 1
	if count > 0:
		x = xsum/count
		y = ysum/count
		return (x, y)
	
	return None


def process_frame(frame):
	size = cv.GetSize(frame)
	hsv = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
	cv.CvtColor(frame, hsv, cv.CV_BGR2HSV)
	
	mask = cv.CreateImage(size, 8, 1)
	cv.InRangeS(hsv, cv.Scalar(0.10*256, 0.45*256, 0.20*256, 0),
		    cv.Scalar(0.15*256, 1.00*256, 1.00*256, 0), mask);
	return mask

def main():
	cv.NamedWindow("img", 0)
	cv.MoveWindow("img", 200, 200)
	cv.NamedWindow("out", 0)
	cv.MoveWindow("out", 200, 600)

	cam = cv.CreateCameraCapture(0);

	while True:
		frame = cv.QueryFrame(cam)
		cv.ShowImage("img", frame)
		out = process_frame(frame)
		cv.ShowImage("out", out)

		print get_pixel_center(out)

		if (cv.WaitKey(23) == 'q'):
			break

if __name__ == '__main__':
	main()
