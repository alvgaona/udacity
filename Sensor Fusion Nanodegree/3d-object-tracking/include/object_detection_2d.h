#ifndef CAMERA_FUSION_OBJECT_DETECTION_H
#define CAMERA_FUSION_OBJECT_DETECTION_H

#include <stdio.h>

#include <opencv2/core.hpp>

#include "data_frame.h"

void DetectObjects(cv::Mat& img, std::vector<BoundingBox>& bounding_boxes, float confidence_threshold, float nms_threshold,
                   const std::string& base_path, const std::string& classes_file, const std::string& model_configuration, const std::string& model_weights,
                   bool visible);

#endif
