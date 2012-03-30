cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#include_directories(. ./src)

find_package(OpenCV REQUIRED)

rosbuild_add_executable(particle_filter src/particle_filter.cpp src/backproject.cpp src/findlines.cpp src/differentiation.cpp src/geometry.cpp)
target_link_libraries(particle_filter opencv_core opencv_imgproc opencv_highgui opencv_objdetect)

rosbuild_add_executable(matlab_export src/matlab_export.cpp src/backproject.cpp src/findlines.cpp src/differentiation.cpp src/geometry.cpp)
target_link_libraries(matlab_export opencv_core opencv_imgproc opencv_objdetect)

rosbuild_add_executable(findlines_gui src/findlines_gui.cpp src/geometry.cpp)
target_link_libraries(findlines_gui opencv_core opencv_imgproc opencv_highgui opencv_objdetect)


#rosbuild_add_executable(ekf_localization src/ekf_localization.cpp src/backproject.cpp src/findlines.cpp src/differentiation.cpp src/geometry.cpp)
#target_link_libraries(ekf_localization opencv_core opencv_imgproc opencv_objdetect)
