#include "camera_fusion.h"

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void ClusterLidarWithROI(
    std::vector<BoundingBox>& bounding_boxes,
    std::vector<LidarPoint>& lidar_points,
    float shrink_factor,
    cv::Mat& P_rect_xx,
    cv::Mat& R_rect_xx,
    cv::Mat& RT) {
  // loop over all Lidar points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto& lidar_point : lidar_points) {
    // assemble vector for matrix-vector-multiplication
    X.at<double>(0, 0) = lidar_point.x;
    X.at<double>(1, 0) = lidar_point.y;
    X.at<double>(2, 0) = lidar_point.z;
    X.at<double>(3, 0) = 1;

    // project Lidar point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);  // pixel coordinates
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    std::vector<std::vector<BoundingBox>::iterator> enclosing_boxes;  // pointers to all bounding boxes which enclose the current Lidar point
    for (auto it2 = bounding_boxes.begin(); it2 != bounding_boxes.end(); ++it2) {
      // shrink current bounding box slightly to avoid having too many outlier points around the edges
      cv::Rect smaller_box;
      smaller_box.x = (*it2).roi.x + shrink_factor * (*it2).roi.width / 2.0;
      smaller_box.y = (*it2).roi.y + shrink_factor * (*it2).roi.height / 2.0;
      smaller_box.width = (*it2).roi.width * (1 - shrink_factor);
      smaller_box.height = (*it2).roi.height * (1 - shrink_factor);

      // check whether point is within current bounding box
      if (smaller_box.contains(pt)) {
        enclosing_boxes.push_back(it2);
      }
    }

    // check whether point has been enclosed by one or by multiple boxes
    if (enclosing_boxes.size() == 1) {
      // add Lidar point to bounding box
      enclosing_boxes[0]->lidar_points.emplace_back(lidar_point);
    }
  }
}

void Show3DObjects(std::vector<BoundingBox>& bounding_boxes, const cv::Size& world_size, const cv::Size& image_size, bool wait) {
  // create topview image
  cv::Mat top_view_img(image_size, CV_8UC3, cv::Scalar(255, 255, 255));

  for (auto& bounding_box : bounding_boxes) {
    // create randomized color for current 3D object
    cv::RNG rng(bounding_box.id);
    cv::Scalar current_color = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

    // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xw_min = 1e8, y_min = 1e8, y_max = -1e8;
    for (auto& lidar_point : bounding_box.lidar_points) {
      // world coordinates
      float xw = lidar_point.x;  // world position in m with x facing forward from sensor
      float yw = lidar_point.y;  // world position in m with y facing left from sensor
      xw_min = xw_min < xw ? xw_min : xw;
      y_min = y_min < yw ? y_min : yw;
      y_max = y_max > yw ? y_max : yw;

      // top-view coordinates
      int y = (-xw * image_size.height / world_size.height) + image_size.height;
      int x = (-yw * image_size.width / world_size.width) + image_size.width / 2;

      // find enclosing rectangle
      top = top < y ? top : y;
      left = left < x ? left : x;
      bottom = bottom > y ? bottom : y;
      right = right > x ? right : x;

      // draw individual point
      cv::circle(top_view_img, cv::Point(x, y), 4, current_color, -1);
    }

    // draw enclosing rectangle
    cv::rectangle(top_view_img, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

    // augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", bounding_box.id, (int)bounding_box.lidar_points.size());
    putText(top_view_img, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, current_color);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xw_min, y_max - y_min);
    putText(top_view_img, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, current_color);
  }

  // plot distance markers
  float line_spacing = 2.0;  // gap between distance markers
  int n_markers = floor(world_size.height / line_spacing);
  for (size_t i = 0; i < n_markers; ++i) {
    int y = (-(i * line_spacing) * image_size.height / world_size.height) + image_size.height;
    cv::line(top_view_img, cv::Point(0, y), cv::Point(image_size.width, y), cv::Scalar(255, 0, 0));
  }

  // display image
  std::string window_name = "3D Objects";
  cv::namedWindow(window_name, 1);
  cv::imshow(window_name, top_view_img);

  if (wait) {
    cv::waitKey(0);  // wait for key to be pressed
  }
}

// associate a given bounding box with the keypoints it contains
void ClusterKptMatchesWithROI(BoundingBox& bounding_box, std::vector<cv::KeyPoint>& keypoints_current, std::vector<cv::DMatch>& keypoints_matches) {
  double distance_accumulator = 0.0L;
  int matches_in_roi_counter = 0;
  for (auto& match : keypoints_matches) {
    cv::KeyPoint keypoint = keypoints_current.at(match.trainIdx);
    if (bounding_box.roi.contains(keypoint.pt)) {
      bounding_box.keypoints_matches.emplace_back(match);
      distance_accumulator += match.distance;
      matches_in_roi_counter++;
    }
  }

  double mean_distance = distance_accumulator / matches_in_roi_counter;

  bounding_box.keypoints_matches.erase(
      std::remove_if(
          bounding_box.keypoints_matches.begin(),
          bounding_box.keypoints_matches.end(),
          [&mean_distance](cv::DMatch& match) { return match.distance < 0.7 * mean_distance; }),
      bounding_box.keypoints_matches.end());
}

double ComputeTTCCamera(
    std::vector<cv::KeyPoint>& keypoints_prev,
    std::vector<cv::KeyPoint>& keypoints_current,
    std::vector<cv::DMatch>& keypoints_matches,
    double frame_rate) {
  std::vector<double> distance_ratios;
  for (auto& match : keypoints_matches) {
    cv::KeyPoint keypoint_outer_curr = keypoints_current.at(match.trainIdx);
    cv::KeyPoint keypoint_outer_prev = keypoints_prev.at(match.queryIdx);

    for (auto& match2 : keypoints_matches) {
      double min_dist = 100.0;
      cv::KeyPoint keypoint_inner_curr = keypoints_current.at(match2.trainIdx);
      cv::KeyPoint keypoint_inner_prev = keypoints_prev.at(match2.queryIdx);

      double distCurr = cv::norm(keypoint_outer_curr.pt - keypoint_inner_curr.pt);
      double distPrev = cv::norm(keypoint_outer_prev.pt - keypoint_inner_prev.pt);

      if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= min_dist) {
        double distance_ratio = distCurr / distPrev;
        distance_ratios.emplace_back(distance_ratio);
      }
    }
  }

  if (distance_ratios.empty()) {
    return -1;
  }

  std::sort(distance_ratios.begin(), distance_ratios.end());
  long med_index = floor(distance_ratios.size() / 2.0);
  double med_dist_ratio = distance_ratios.size() % 2 == 0 ? (distance_ratios[med_index - 1] + distance_ratios[med_index]) / 2.0
                                                          : distance_ratios[med_index];  // compute median dist. ratio to remove outlier influence
  double dt = 1 / frame_rate;
  return -dt / (1 - med_dist_ratio);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RemoveLidarOutliers(const std::vector<LidarPoint>& lidar_points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& p : lidar_points) {
    cloud->push_back(pcl::PointXYZ((float)p.x, (float)p.y, 0.0f));
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.05);
  ec.setSearchMethod(tree);
  ec.setMinClusterSize(3);
  ec.setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

  if (cluster_indices.empty()) {
    return result;
  }

  for (auto& cluster : cluster_indices) {
    for (auto& idx : cluster.indices) {
      result->points.push_back(cloud->points.at(idx));
    }
  }
  return result;
}

double ComputeTTCLidar(const std::vector<LidarPoint>& lidar_points_prev, const std::vector<LidarPoint>& lidar_points_current, double frame_rate) {
  double dt = 1 / frame_rate;
  double min_x_prev = 1e9L;
  double min_x_curr = 1e9L;

  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_obstacle_cloud = RemoveLidarOutliers(lidar_points_prev);
  pcl::PointCloud<pcl::PointXYZ>::Ptr curr_obstacle_cloud = RemoveLidarOutliers(lidar_points_current);

  for (const auto& p : prev_obstacle_cloud->points) {
    min_x_prev = min_x_prev > p.x ? p.x : min_x_prev;
  }
  for (const auto& p : curr_obstacle_cloud->points) {
    min_x_curr = min_x_curr > p.x ? p.x : min_x_curr;
  }
  return min_x_curr * dt / (min_x_prev - min_x_curr);
}

void MatchBoundingBoxes(
    const std::vector<cv::DMatch>& matches, std::map<int, int>& bounding_box_best_matches, DataFrame& prev_frame, DataFrame& current_frame) {
  std::map<std::pair<int, int>, int> bounding_box_matches_counter;

  for (auto& match : matches) {
    cv::Point previous_point = prev_frame.keypoints.at(match.queryIdx).pt;
    cv::Point current_point = current_frame.keypoints.at(match.trainIdx).pt;

    for (auto& bounding_box_in_prev : prev_frame.bounding_boxes) {
      if (!bounding_box_in_prev.roi.contains(previous_point)) {
        continue;
      }

      for (auto& bounding_box_in_curr : current_frame.bounding_boxes) {
        if (!bounding_box_in_curr.roi.contains(current_point)) {
          continue;
        }
        bounding_box_matches_counter[std::make_pair(bounding_box_in_prev.id, bounding_box_in_curr.id)]++;
      }
    }

    std::vector<std::tuple<int, int, int>> bounding_box_matches;
    std::for_each(
        bounding_box_matches_counter.begin(), bounding_box_matches_counter.end(), [&bounding_box_matches](std::pair<std::pair<int, int>, int> pair) {
          bounding_box_matches.emplace_back(pair.first.first, pair.first.second, pair.second);
        });

    std::sort(bounding_box_matches.begin(), bounding_box_matches.end(), [](const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b) {
      return std::get<2>(a) > std::get<2>(b);
    });

    std::set<int> matched_previous_boxes;
    bounding_box_best_matches.clear();
    for (auto& tup : bounding_box_matches) {
      if (matched_previous_boxes.count(std::get<0>(tup))) {
        continue;
      }
      matched_previous_boxes.insert(std::get<0>(tup));
      bounding_box_best_matches.insert(std::make_pair(std::get<0>(tup), std::get<1>(tup)));
    }
  }
}
