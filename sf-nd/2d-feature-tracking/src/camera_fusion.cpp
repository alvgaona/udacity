#include <cmath>
#include <iomanip>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "data_frame.h"
#include "matching_2d.h"
#include "ring_buffer.h"

int main(int argc, const char *argv[]) {
  std::cout << "### INITIALIZING ### " << std::endl;

  std::string data_path = "../";
  std::string img_base_path = data_path + "images/";
  std::string img_prefix = "KITTI/2011_09_26/image_00/data/000000";  // left camera, color
  std::string img_file_type = ".png";
  int img_start_index = 0;
  int img_end_index = 9;
  int img_fill_width = 4;

  RingBuffer<DataFrame> ring_buffer(2);

  /* Configuration */
  // TODO: Move configuration parameters to a YAML file.
  std::string detector_type = "SIFT";
  std::string descriptor_type = "SIFT";
  std::string matcher_type = "MAT_FLANN";
  std::string selector_type = "SEL_KNN";

  /* Miscellaneous */
  bool limit_keypoints = false;
  bool focus_on_vehicle = true;
  bool visualize_matches = false;

  /* Counters */
  int total_keypoints = 0;
  int total_matches = 0;

  for (size_t img_index = 0; img_index <= img_end_index - img_start_index; img_index++) {
    // TODO: Create ImageGenerator class responsible of providing an image at a time.
    std::stringstream img_number;
    img_number << std::setfill('0') << std::setw(img_fill_width) << img_start_index + img_index;
    std::stringstream img_full_filename;
    img_full_filename << img_base_path << img_prefix << img_number.str() << img_file_type;

    std::cout << "Converting image to grayscale." << std::endl;

    // TODO: Add image grayscale conversion logic in ImageGenerator
    cv::Mat img, img_gray;
    img = cv::imread(img_full_filename.str());
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    std::cout << "Loading image into ring buffer." << std::endl;

    DataFrame frame;
    frame.image = img_gray;
    ring_buffer.PushFront(frame);

    std::cout << "Detecting image keypoints" << std::endl;

    std::vector<cv::KeyPoint> keypoints;  // create empty feature list for current image

    // TODO: Move this string based detection to a Method Factory
    if (detector_type == "SHITOMASI") {
      DetectKeypointsShiTomasi(keypoints, img_gray, false);
    } else if (detector_type == "HARRIS") {
      DetectKeypointsHarris(keypoints, img_gray, false);
    } else {
      DetectKeypointsModern(keypoints, img_gray, detector_type, false);
    }

    // TODO: Read focus_on_vehicle value from a file.
    //    This is a configuration parameter.
    cv::Rect vehicle_frame(535, 180, 180, 150);
    if (focus_on_vehicle) {
      keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                                     [&vehicle_frame](const cv::KeyPoint &kp) { return !vehicle_frame.contains(kp.pt); }),
                      keypoints.end());
    }

    // TODO: Same with limit_keypoints.
    if (limit_keypoints) {
      int max_keypoints = 50;

      if (detector_type == "SHITOMASI") {  // there is no response info, so keep the first 50 as they are sorted in descending quality order
        keypoints.erase(keypoints.begin() + max_keypoints, keypoints.end());
      }
      cv::KeyPointsFilter::retainBest(keypoints, max_keypoints);
      std::cout << " NOTE: Keypoints have been limited!" << std::endl;
    }

    ring_buffer.begin()->keypoints = keypoints;

    total_keypoints += keypoints.size();

    std::cout << "Extracting keypoint descriptors" << std::endl;

    cv::Mat descriptors;
    // TODO: Move this function to a factory.
    DescribeKeypoints(ring_buffer.begin()->keypoints, ring_buffer.begin()->image, descriptors, descriptor_type);

    // push descriptors for current frame to end of data buffer
    ring_buffer.begin()->descriptors = descriptors;

    if (ring_buffer.size() > 1) {
      std::cout << "Matching keypoint descriptors" << std::endl;

      // TODO: Encapsulate matching procedure in a custom Matcher class.
      std::vector<cv::DMatch> matches;
      MatchDescriptors((ring_buffer.begin() + 1)->keypoints, ring_buffer.begin()->keypoints, (ring_buffer.begin() + 1)->descriptors,
                       ring_buffer.begin()->descriptors, matches, matcher_type, selector_type);

      total_matches += matches.size();

      ring_buffer.begin()->keypoints_matches = matches;

      if (visualize_matches) {
        cv::Mat match_img = (ring_buffer.begin()->image).clone();
        cv::drawMatches((ring_buffer.begin() + 1)->image, (ring_buffer.begin() + 1)->keypoints, ring_buffer.begin()->image,
                        ring_buffer.begin()->keypoints, matches, match_img, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        std::string window_name = "Matching keypoints between two camera images";
        cv::namedWindow(window_name, 7);
        cv::imshow(window_name, match_img);
        std::cout << "Press key to continue to next image" << std::endl;
        cv::waitKey(0);  // wait for key to be pressed
      }
    }
  }

  std::cout << "Total keypoints: " << total_keypoints << std::endl;
  std::cout << "Total matches: " << total_matches << std::endl;

  return 0;
}
