#ifndef CAMERA_FUSION_LIDAR_DATA_H
#define CAMERA_FUSION_LIDAR_DATA_H

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "data_frame.h"

void CropLidarPoints(std::vector<LidarPoint> &lidar_points, float min_x, float max_x, float max_y, float min_z, float max_z, float min_r);

void LoadLidarFromFile(std::vector<LidarPoint> &lidar_points, const std::string &filename);

void ShowLidarTopview(std::vector<LidarPoint> &lidar_points, const cv::Size &world_size, const cv::Size &image_size, bool wait = true);

void ShowLidarImgOverlay(cv::Mat &img, std::vector<LidarPoint> &lidar_points, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT,
                         cv::Mat *ext_vis_img = nullptr);
#endif
