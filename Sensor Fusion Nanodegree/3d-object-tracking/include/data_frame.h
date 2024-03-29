#ifndef CAMERA_FUSION_DATA_FRAME_H
#define CAMERA_FUSION_DATA_FRAME_H

#include <map>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

#include "bounding_box.h"
#include "lidar_point.h"

struct DataFrame {  // represents the available sensor information at the same time instance

  cv::Mat camera_image;  // camera image

  std::vector<cv::KeyPoint> keypoints;        // 2D keypoints within camera image
  cv::Mat descriptors;                        // keypoint descriptors
  std::vector<cv::DMatch> keypoints_matches;  // keypoint matches between previous and current frame
  std::vector<LidarPoint> lidar_points;

  std::vector<BoundingBox> bounding_boxes;  // ROI around detected objects in 2D image coordinates
  std::map<int, int> bounding_box_matches;  // bounding box matches between previous and current frame
};

#endif
