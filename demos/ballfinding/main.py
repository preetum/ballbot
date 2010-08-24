import cv


def main():
	cv.NamedWindow("out", 0)
	cv.MoveWindow("out", 200, 200)

	cam = cv.CreateCameraCapture(0);

	while True:
		frame = cv.QueryFrame(cam)
		cv.ShowImage("out", frame)

		if (cv.WaitKey(23) == 'q'):
			break

if __name__ == '__main__':
	main()
