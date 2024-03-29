#ifndef CAMERA_FUSION_CAMERA_FUSION_H
#define CAMERA_FUSION_CAMERA_FUSION_H

#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>
#include <vector>

#include "data_frame.h"

void ClusterLidarWithROI(
    std::vector<BoundingBox>& bounding_boxes,
    std::vector<LidarPoint>& lidar_points,
    float shrink_factor,
    cv::Mat& P_rect_xx,
    cv::Mat& R_rect_xx,
    cv::Mat& RT);

void ClusterKptMatchesWithROI(BoundingBox& bounding_box, std::vector<cv::KeyPoint>& keypoints_current, std::vector<cv::DMatch>& keypoints_matches);

void MatchBoundingBoxes(
    const std::vector<cv::DMatch>& matches, std::map<int, int>& bounding_box_best_matches, DataFrame& prev_frame, DataFrame& current_frame);

void Show3DObjects(std::vector<BoundingBox>& bounding_boxes, const cv::Size& world_size, const cv::Size& image_size, bool wait = true);

double ComputeTTCCamera(
    std::vector<cv::KeyPoint>& keypoints_prev,
    std::vector<cv::KeyPoint>& keypoints_current,
    std::vector<cv::DMatch>& keypoints_matches,
    double frame_rate);

double ComputeTTCLidar(const std::vector<LidarPoint>& lidar_points_prev, const std::vector<LidarPoint>& lidar_points_current, double frame_rate);

pcl::PointCloud<pcl::PointXYZ>::Ptr RemoveLidarOutliers(const std::vector<LidarPoint>& lidar_points);

#endif
