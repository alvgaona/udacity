# 3D Object Tracking

This repository contains the final project for the camera course. It builds upon the knowledge gained from previous lessons on keypoint detectors, descriptors, methods for matching keypoints across images, object detection using YOLO, and associating camera image regions with Lidar data in 3D space. The program schematic illustrates the existing components and the missing parts to be implemented.

<img src="images/course_code_structure.png" width="779" height="414"  alt="Course Code Structure"/>

The final project comprises four main tasks:

1. Developing a method to match 3D objects over time using keypoint correspondences.
2. Computing the Time-To-Collision (TTC) based on LiDAR measurements.
3. Performing the same TTC computation using the camera by associating keypoint matches with regions of interest.
4. Conducting tests to identify the most suitable detector/descriptor combination for accurate TTC estimation and investigating potential issues with camera or Lidar measurements.
5. While the Kalman filter technique for combining independent TTC measurements will be covered in the subsequent course, this project focuses on the culminating tasks related to the camera course.

The repository contains the necessary code and resources to complete the project successfully.

## Dependencies for Running Locally

* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, macOS), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * macOS: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * macOS: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Build `cd /path/to/3d-object-tracking && make build`
3. Run it: `./object_tracking`.
