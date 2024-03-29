#ifndef CAMERA_FUSION_BOUNDING_BOX_H
#define CAMERA_FUSION_BOUNDING_BOX_H

#include "lidar_point.h"

struct BoundingBox {  // bounding box around a classified object (contains both 2D and 3D data)

  int id;        // unique identifier for this bounding box
  int track_id;  // unique identifier for the track to which this bounding box belongs

  cv::Rect roi;       // 2D region-of-interest in image coordinates
  int class_id;       // ID based on class file provided to YOLO framework
  double confidence;  // classification trust

  std::vector<LidarPoint> lidar_points;       // Lidar 3D points which project into 2D image roi
  std::vector<cv::KeyPoint> keypoints;        // keypoints enclosed by 2D roi
  std::vector<cv::DMatch> keypoints_matches;  // keypoint matches enclosed by 2D roi
};

#endif
