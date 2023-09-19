# 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project.
As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts.
Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies

* `cmake >= 2.8`.
  * All OSes: [click here for installation instructions][CMake].
* `make >= 4.1` (Linux, macOS), 3.81 (Windows).
  * Linux: make is installed by default on most Linux distros.
  * macOS: [install Xcode command line tools to get make][Xcode].
  * Windows: [Click here for installation instructions][Make].
* `OpenCV >= 4.1`.
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here][OpenCV].
* `gcc/g++ >= 5.4`.
  * Linux: gcc / g++ is installed by default on most Linux distros.
  * macOS: same deal as make - [install Xcode command line tools][Xcode].
  * Windows: recommend using [MinGW].

## Usage

1. Clone the repository.

```bash
git clone https://github.com/alvgaona/2d-feature-tracking.git
```

2. Build the project.

```bash
cd 2d-feature-tracking
make build
```

3. Run it.

```bash
./feature_tracking
```

[Make]: http://gnuwin32.sourceforge.net/packages/make.htm
[CMake]: https://cmake.org/install
[Xcode]: https://developer.apple.com/xcode/features
[OpenCV]: https://github.com/opencv/opencv/tree/4.1.0
[MinGW]: http://www.mingw.org
