# Builds the cpp using opencv libraries

all:
	g++ -o depth stereo_match.cpp `pkg-config --cflags --libs opencv-2.3.1` -L/usr/lib -lopencv_core

traj:
	g++ -o traj trajectory_generator.cpp backproject.cpp `pkg-config --cflags --libs opencv-2.3.1` -L /usr/lib -L/usr/local/lib -L/opt/lib -l opencv_core

depth: 
	g++ -o depth stereo_match.cpp `pkg-config --cflags --libs opencv-2.3.1` -L/usr/lib -lopencv_core

example:
	g++ -o example /home/ankush/Sources/gnuplot-cpp/gnuplot_i.cc example.cc -I/home/ankush/Sources/gnuplot-cpp

filter:
	g++ -o filter trajectory_simulator.cpp particle_filter.cpp backproject.cpp ball_detector.cpp gnuplot-cpp/gnuplot_i.cc -Ignuplot-cpp `pkg-config --cflags --libs opencv-2.3.1` -L /usr/lib -L/usr/local/lib -L/opt/lib -l opencv_core -l opencv_highgui


clean:
	rm -f depth, traj, example
	rm -f *.dat
	rm filter
