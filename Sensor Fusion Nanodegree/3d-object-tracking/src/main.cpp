#include <iomanip>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <fstream>

#include "camera_fusion.h"
#include "data_frame.h"
#include "lidar_data.h"
#include "matching_2d.h"
#include "object_detection_2d.h"
#include "ring_buffer.h"

int main(int argc, const char* argv[]) {
  std::cout << "### INITIALIZING ### " << std::endl;

  std::string data_path = "../";

  // Camera
  std::string image_base_path = data_path + "images/";
  std::string image_prefix = "KITTI/2011_09_26/image_02/data/000000";  // left camera, color
  std::string image_file_type = ".png";
  int image_start_index = 0;  // first file index to load (assumes Lidar and camera names have identical naming convention)
  int image_end_index = 77;   // last file index to load
  int image_step_width = 1;
  int image_fill_width = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

  // object detection
  std::string yolo_base_path = data_path + "dat/yolo/";
  std::string yolo_classes_file = yolo_base_path + "coco.names";
  std::string yolo_model_configuration = yolo_base_path + "yolov3.cfg";
  std::string yolo_model_weights = yolo_base_path + "yolov3.weights";

  // Lidar
  std::string lidar_prefix = "KITTI/2011_09_26/velodyne_points/data/000000";
  std::string lidar_file_type = ".bin";

  // Calibration data for camera and lidar
  cv::Mat P_rect_00(3, 4, cv::DataType<double>::type);  // 3x4 projection matrix after rectification
  cv::Mat R_rect_00(4, 4, cv::DataType<double>::type);  // 3x3 rectifying rotation to make image planes co-planar
  cv::Mat RT(4, 4, cv::DataType<double>::type);         // rotation matrix and translation vector

  RT.at<double>(0, 0) = 7.533745e-03;
  RT.at<double>(0, 1) = -9.999714e-01;
  RT.at<double>(0, 2) = -6.166020e-04;
  RT.at<double>(0, 3) = -4.069766e-03;
  RT.at<double>(1, 0) = 1.480249e-02;
  RT.at<double>(1, 1) = 7.280733e-04;
  RT.at<double>(1, 2) = -9.998902e-01;
  RT.at<double>(1, 3) = -7.631618e-02;
  RT.at<double>(2, 0) = 9.998621e-01;
  RT.at<double>(2, 1) = 7.523790e-03;
  RT.at<double>(2, 2) = 1.480755e-02;
  RT.at<double>(2, 3) = -2.717806e-01;
  RT.at<double>(3, 0) = 0.0;
  RT.at<double>(3, 1) = 0.0;
  RT.at<double>(3, 2) = 0.0;
  RT.at<double>(3, 3) = 1.0;

  R_rect_00.at<double>(0, 0) = 9.999239e-01;
  R_rect_00.at<double>(0, 1) = 9.837760e-03;
  R_rect_00.at<double>(0, 2) = -7.445048e-03;
  R_rect_00.at<double>(0, 3) = 0.0;
  R_rect_00.at<double>(1, 0) = -9.869795e-03;
  R_rect_00.at<double>(1, 1) = 9.999421e-01;
  R_rect_00.at<double>(1, 2) = -4.278459e-03;
  R_rect_00.at<double>(1, 3) = 0.0;
  R_rect_00.at<double>(2, 0) = 7.402527e-03;
  R_rect_00.at<double>(2, 1) = 4.351614e-03;
  R_rect_00.at<double>(2, 2) = 9.999631e-01;
  R_rect_00.at<double>(2, 3) = 0.0;
  R_rect_00.at<double>(3, 0) = 0;
  R_rect_00.at<double>(3, 1) = 0;
  R_rect_00.at<double>(3, 2) = 0;
  R_rect_00.at<double>(3, 3) = 1;

  P_rect_00.at<double>(0, 0) = 7.215377e+02;
  P_rect_00.at<double>(0, 1) = 0.000000e+00;
  P_rect_00.at<double>(0, 2) = 6.095593e+02;
  P_rect_00.at<double>(0, 3) = 0.000000e+00;
  P_rect_00.at<double>(1, 0) = 0.000000e+00;
  P_rect_00.at<double>(1, 1) = 7.215377e+02;
  P_rect_00.at<double>(1, 2) = 1.728540e+02;
  P_rect_00.at<double>(1, 3) = 0.000000e+00;
  P_rect_00.at<double>(2, 0) = 0.000000e+00;
  P_rect_00.at<double>(2, 1) = 0.000000e+00;
  P_rect_00.at<double>(2, 2) = 1.000000e+00;
  P_rect_00.at<double>(2, 3) = 0.000000e+00;

  /* Configuration */
  // TODO: Move configuration parameters to a YAML file.
  std::string detector_type = "AKAZE";
  std::string descriptor_type = "AKAZE";
  std::string matcher_type = "MAT_BF";
  std::string selector_type = "SEL_KNN";

  /* Miscellaneous */
  double sensor_frame_rate = 10.0 / image_step_width;

  bool show_classification_result = false;
  bool show_3d_objects = false;
  bool visualize_results = false;

  RingBuffer<DataFrame> ring_buffer(2);

  std::cout << "Detector-Descriptor Combination: " << detector_type << "-" << descriptor_type << std::endl;

  std::ofstream file("/Users/alvaro/github.com/3d-object-tracking/results/" + detector_type + "_" + descriptor_type + ".csv");
  file << "Frame,TTC Lidar,TTC Camera" << std::endl;

  for (auto image_index = 0; image_index <= image_end_index - image_start_index; image_index += image_step_width) {
    // assemble filenames for current index
    std::stringstream image_number;
    image_number << std::setfill('0') << std::setw(image_fill_width) << image_start_index + image_index;
    std::stringstream image_full_filename;
    image_full_filename << image_base_path << image_prefix << image_number.str() << image_file_type;

    // push image into data frame buffer
    DataFrame frame;
    frame.camera_image = cv::imread(image_full_filename.str());
    ring_buffer.PushFront(frame);

    /* DETECT & CLASSIFY OBJECTS */

    float confidence_threshold = 0.2;
    float nms_threshold = 0.4;
    DetectObjects(
        ring_buffer.begin()->camera_image,
        ring_buffer.begin()->bounding_boxes,
        confidence_threshold,
        nms_threshold,
        yolo_base_path,
        yolo_classes_file,
        yolo_model_configuration,
        yolo_model_weights,
        show_classification_result);

    /* CROP LIDAR POINTS */

    // load 3D Lidar points from file
    std::stringstream lidar_full_filename;
    lidar_full_filename << image_base_path << lidar_prefix << image_number.str() + lidar_file_type;
    std::vector<LidarPoint> lidar_points;
    LoadLidarFromFile(lidar_points, lidar_full_filename.str());

    // remove Lidar points based on distance properties
    float min_z = -1.5;
    float max_z = -0.9;
    float min_x = 2.0;
    float max_x = 20.0;
    float max_y = 2.0;
    float min_r = 0.1;  // focus on ego lane
    CropLidarPoints(lidar_points, min_x, max_x, max_y, min_z, max_z, min_r);

    ring_buffer.begin()->lidar_points = lidar_points;

    /* CLUSTER LIDAR POINT CLOUD */

    // associate Lidar points with camera-based ROI
    float shrink_factor = 0.1;  // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
    ClusterLidarWithROI(ring_buffer.begin()->bounding_boxes, ring_buffer.begin()->lidar_points, shrink_factor, P_rect_00, R_rect_00, RT);

    // Visualize 3D objects
    if (show_3d_objects) {
      Show3DObjects(ring_buffer.begin()->bounding_boxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
    }

    /* DETECT IMAGE KEYPOINTS */

    // convert current image to grayscale
    cv::Mat img_gray;
    cv::cvtColor(ring_buffer.begin()->camera_image, img_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;

    // TODO: Move this string based detection to a Method Factory
    if (detector_type == "SHITOMASI") {
      DetectKeypointsShiTomasi(keypoints, img_gray, false);
    } else if (detector_type == "HARRIS") {
      DetectKeypointsHarris(keypoints, img_gray, false);
    } else {
      DetectKeypointsModern(keypoints, img_gray, detector_type, false);
    }

     // push keypoints and descriptor for current frame to end of data buffer
    ring_buffer.begin()->keypoints = keypoints;

    cv::Mat descriptors;
    DescribeKeypoints(ring_buffer.begin()->keypoints, ring_buffer.begin()->camera_image, descriptors, descriptor_type);

    ring_buffer.begin()->descriptors = descriptors;

    if (ring_buffer.size() > 1) {
      /* MATCH KEYPOINT DESCRIPTORS */

      std::vector<cv::DMatch> matches;
      MatchDescriptors(
          (ring_buffer.begin() + 1)->keypoints,
          ring_buffer.begin()->keypoints,
          (ring_buffer.begin() + 1)->descriptors,
          ring_buffer.begin()->descriptors,
          matches,
          matcher_type,
          selector_type);

      ring_buffer.begin()->keypoints_matches = matches;

      /* TRACK 3D OBJECT BOUNDING BOXES */

      std::map<int, int> bounding_box_best_matches;
      MatchBoundingBoxes(matches, bounding_box_best_matches, *(ring_buffer.begin() + 1), *ring_buffer.begin());

      ring_buffer.begin()->bounding_box_matches = bounding_box_best_matches;

      /* COMPUTE TTC ON OBJECT IN FRONT */

      for (auto it1 = ring_buffer.begin()->bounding_box_matches.begin(); it1 != ring_buffer.begin()->bounding_box_matches.end(); ++it1) {
        // find bounding boxes associates with current match
        BoundingBox* previous_bounding_box = nullptr;
        BoundingBox* current_bounding_box = nullptr;
        for (auto& bounding_box : ring_buffer.begin()->bounding_boxes) {
          if (it1->second == bounding_box.id) {
            current_bounding_box = &bounding_box;
            break;
          }
        }

        for (auto& bounding_boxe : (ring_buffer.begin() + 1)->bounding_boxes) {
          if (it1->first == bounding_boxe.id) {
            previous_bounding_box = &bounding_boxe;
            break;
          }
        }

        if (current_bounding_box->lidar_points.size() > 100 && previous_bounding_box->lidar_points.size() > 100) {
          double ttc_lidar = ComputeTTCLidar(previous_bounding_box->lidar_points, current_bounding_box->lidar_points, sensor_frame_rate);

          ClusterKptMatchesWithROI(*current_bounding_box, ring_buffer.begin()->keypoints, ring_buffer.begin()->keypoints_matches);
          double ttc_camera = ComputeTTCCamera(
              (ring_buffer.begin() + 1)->keypoints, ring_buffer.begin()->keypoints, current_bounding_box->keypoints_matches, sensor_frame_rate);

//          std::cout << "Image: " << image_number.str() << " Frame " << image_index << " - TTC Lidar: " << ttc_lidar << " TTC Camera: " << ttc_camera << std::endl;

          file << image_index  << "," << ttc_lidar << "," << ttc_camera << std::endl;
          std::cout << image_index  << "," << ttc_lidar << "," << ttc_camera << std::endl;

          if (visualize_results) {
            cv::Mat visualized_image = ring_buffer.begin()->camera_image.clone();
            ShowLidarImgOverlay(visualized_image, current_bounding_box->lidar_points, P_rect_00, R_rect_00, RT, &visualized_image);
            cv::rectangle(
                visualized_image,
                cv::Point(current_bounding_box->roi.x, current_bounding_box->roi.y),
                cv::Point(
                    current_bounding_box->roi.x + current_bounding_box->roi.width, current_bounding_box->roi.y + current_bounding_box->roi.height),
                cv::Scalar(0, 255, 0),
                2);

            char str[200];
            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttc_lidar, ttc_camera);
            putText(visualized_image, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

            std::string window_name = "Final Results" + image_number.str();
            cv::namedWindow(window_name, 4);
            cv::imshow(window_name, visualized_image);
            std::cout << "Press key to continue to next frame" << std::endl;
            cv::waitKey(0);
          }
        }
      }
    }
  }
  file.close();
  return 0;
}
