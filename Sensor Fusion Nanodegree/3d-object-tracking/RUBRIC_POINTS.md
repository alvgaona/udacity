# Rubric points

This document gathers information about all the rubric points met for this project.

## FP.1 Match 3D Objects

**CRITERIA**

Implement the method `MatchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the `id` property).
Matches must be the ones with the highest number of keypoint correspondences.

Implementation can be found [here][MatchBoundingBoxes].

## FP.2 Compute Lidar-based TTC

**CRITERIA**

Compute the time-to-collision in seconds for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

Implementation can be found [here][ComputeTTCLidar].

## FP.3 Associate Keypoint Correspondences with Bounding Boxes

**CRITERIA**

Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them.
All matches which satisfy this condition must be added to a vector in the respective bounding box.

Implementation can be found [here][ClusterKptsMatchesWithROI].
   
## FP.4 Compute Camera-based TTC

**CRITERIA**

Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
   
Implementation can be found [here][ComputeTTCCamera].
   
## FP.5 Performance Evaluation 1

**CRITERIA**

Find examples where the TTC estimate of the Lidar sensor does not seem plausible.
Describe your observations and provide a sound argumentation why you think this happened.

![TTC Comparison]

The picture depicts small fluctuations between the initial frame and frame 48.
At frame 7, the TTC lidar measurement deflects from the real TTC and this can be cause by
many factors such as noise in the lidar points measurements and outliers.
The noise can be seen frame-by-frame where jumps of ~3 seconds can be spotted when analyzing
a sequence of 3 frames.
After frame 48, the cars seem to be stopping and the measurements start to diverge since the denominator
tends to 0 for both TTC Lidar and TTC Camera.   
   
## FP.6 Performance Evaluation 2

**CRITERIA**

Run several detector-descriptor combinations and look at the differences in TTC estimation.
Find out which methods perform best and also include several examples where camera-based TTC estimation is way off.
As with Lidar, describe your observations again and also look into potential reasons.

To check for detector-description pair sheets, follow this link to the [results][Results].

![Detector-Descriptor TTC Peformance]

There are a lot of erroneous measurements due to outliers and small number of keypoints.
The small number of keypoints can be visualized if you run, for instance a `HARRIS-BRIEF` combination.
Harris detector provides a small amount of keypoints and in most of the measurements will diverge or will not 
even be able to compute it.

Lastly, one of the best combinations to fuse with the lidar measurements is `AKAZE-BRIEF`.

[MatchBoundingBoxes]: src/camera_fusion.cpp#L221
[ComputeTTCLidar]: src/camera_fusion.cpp#L208
[ClusterKptsMatchesWithROI]: src/camera_fusion.cpp#L115
[ComputeTTCCamera]: src/camera_fusion.cpp#L137
[Results]: results/
[TTC Comparison]: images/ttc-comparison.png
[Detector-Descriptor TTC Peformance]: images/detector-descriptor-ttc-performance.png


