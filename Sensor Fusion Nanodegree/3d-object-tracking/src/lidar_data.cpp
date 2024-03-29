#include "lidar_data.h"

// remove Lidar points based on min. and max distance in X, Y and Z
void CropLidarPoints(std::vector<LidarPoint> &lidar_points, float min_x, float max_x, float max_y, float min_z, float max_z, float min_r) {
  std::vector<LidarPoint> new_lidar_points;
  for (auto it = lidar_points.begin(); it != lidar_points.end(); ++it) {
    if ((*it).x >= min_x && (*it).x <= max_x && (*it).z >= min_z && (*it).z <= max_z && (*it).z <= 0.0 && abs((*it).y) <= max_y &&
        (*it).r >= min_r)  // Check if Lidar point is outside of boundaries
    {
      new_lidar_points.push_back(*it);
    }
  }

  lidar_points = new_lidar_points;
}

// Load Lidar points from a given location and store them in a vector
void LoadLidarFromFile(std::vector<LidarPoint> &lidar_points, const std::string &filename) {
  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  unsigned long num = 1000000;
  auto *data = (float *)malloc(num * sizeof(float));

  // pointers
  float *px = data + 0;
  float *py = data + 1;
  float *pz = data + 2;
  float *pr = data + 3;

  // load point cloud
  FILE *stream;
  stream = fopen(filename.c_str(), "rb");
  num = fread(data, sizeof(float), num, stream) / 4;

  for (int32_t i = 0; i < num; i++) {
    LidarPoint lpt(*px, *py, *pz, *pr);
    lidar_points.push_back(lpt);
    px += 4;
    py += 4;
    pz += 4;
    pr += 4;
  }
  fclose(stream);
}

void ShowLidarTopview(std::vector<LidarPoint> &lidar_points, const cv::Size &world_size, const cv::Size &image_size, bool wait) {
  // create topview image
  cv::Mat top_view_img(image_size, CV_8UC3, cv::Scalar(0, 0, 0));

  // plot Lidar points into image
  for (auto &lidar_point : lidar_points) {
    float xw = lidar_point.x;  // world position in m with x facing forward from sensor
    float yw = lidar_point.y;  // world position in m with y facing left from sensor

    int y = (-xw * image_size.height / world_size.height) + image_size.height;
    int x = (-yw * image_size.height / world_size.height) + image_size.width / 2;

    cv::circle(top_view_img, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
  }

  // plot distance markers
  float lineSpacing = 2.0;  // gap between distance markers
  int nMarkers = floor(world_size.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * image_size.height / world_size.height) + image_size.height;
    cv::line(top_view_img, cv::Point(0, y), cv::Point(image_size.width, y), cv::Scalar(255, 0, 0));
  }

  // display image
  std::string window_name = "Top-View Perspective of LiDAR data";
  cv::namedWindow(window_name, 2);
  cv::imshow(window_name, top_view_img);
  if (wait) {
    cv::waitKey(0);  // wait for key to be pressed
  }
}

void ShowLidarImgOverlay(cv::Mat &img, std::vector<LidarPoint> &lidar_points, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT,
                         cv::Mat *ext_vis_img) {
  // init image for visualization
  cv::Mat visible_image;
  if (ext_vis_img == nullptr) {
    visible_image = img.clone();
  } else {
    visible_image = *ext_vis_img;
  }

  cv::Mat overlay = visible_image.clone();

  // find max. x-value
  double max_val = 0.0;
  for (auto &lidar_point : lidar_points) {
    max_val = max_val < lidar_point.x ? lidar_point.x : max_val;
  }

  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);
  for (auto &lidar_point : lidar_points) {
    X.at<double>(0, 0) = lidar_point.x;
    X.at<double>(1, 0) = lidar_point.y;
    X.at<double>(2, 0) = lidar_point.z;
    X.at<double>(3, 0) = 1;

    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    float val = lidar_point.x;
    int red = std::min(255, (int)(255 * abs((val - max_val) / max_val)));
    int green = std::min(255, (int)(255 * (1 - abs((val - max_val) / max_val))));
    cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1);
  }

  float opacity = 0.6;
  cv::addWeighted(overlay, opacity, visible_image, 1 - opacity, 0, visible_image);

  // return augmented image or wait if no image has been provided
  if (ext_vis_img == nullptr) {
    std::string windowName = "LiDAR data on image overlay";
    cv::namedWindow(windowName, 3);
    cv::imshow(windowName, visible_image);
    cv::waitKey(0);  // wait for key to be pressed
  } else {
    ext_vis_img = &visible_image;
  }
}
