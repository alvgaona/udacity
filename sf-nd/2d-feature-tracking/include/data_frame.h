#ifndef CAMERA_FUSION_DATA_FRAME_H
#define CAMERA_FUSION_DATA_FRAME_H

#include <opencv2/core.hpp>
#include <vector>

struct DataFrame {
  cv::Mat image;  // camera image

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<cv::DMatch> keypoints_matches;  // Keypoints matches between previous and current frame
};

#endif /* dataStructures_h */
