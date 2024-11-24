#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/encoded_image.hpp>
#include <rerun/archetypes/points2d.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <rerun/recording_stream.hpp>
#include <vector>

#include "collection_adapters.h"
#include "data_frame.h"
#include "frame_buffer.h"
#include "matching_2d.h"

namespace fs = std::filesystem;

int main() {
  FrameBuffer<DataFrame> frame_buffer(2);

  /* Configuration */
  std::string detector_type = "SIFT";
  std::string descriptor_type = "SIFT";
  std::string matcher_type = "MAT_FLANN";
  std::string selector_type = "SEL_KNN";

  std::vector<fs::path> img_paths;

  const auto rec = rerun::RecordingStream("2d_feature_image");
  rec.spawn().exit_on_failure();

  for (const auto& entry :
       fs::directory_iterator("./images/KITTI/2011_09_26/image_00/data")) {
    img_paths.push_back(entry.path());
  }

  std::sort(img_paths.begin(), img_paths.end());

  for (const auto& img_path : img_paths) {
    cv::Mat img = cv::imread(img_path.string(), cv::IMREAD_GRAYSCALE);

    DataFrame frame;
    frame.image = img;
    frame_buffer.append(frame);

    std::vector<cv::KeyPoint> keypoints;

    if (detector_type == "SHITOMASI") {
      DetectKeypointsShiTomasi(keypoints, img);
    } else if (detector_type == "HARRIS") {
      DetectKeypointsHarris(keypoints, img);
    } else {
      DetectKeypointsModern(keypoints, img, detector_type);
    }

    frame_buffer.begin()->keypoints = keypoints;

    cv::Mat descriptors;

    DescribeKeypoints(
      frame_buffer.begin()->keypoints,
      frame_buffer.begin()->image,
      descriptors,
      descriptor_type
    );

    frame_buffer.begin()->descriptors = descriptors;

    if (frame_buffer.size() > 1) {
      std::vector<cv::DMatch> matches;
      MatchDescriptors(
        (frame_buffer.begin() + 1)->descriptors,
        frame_buffer.begin()->descriptors,
        matches,
        matcher_type,
        selector_type
      );

      frame_buffer.begin()->keypoints_matches = matches;

      // Logging in Rerun
      auto prev_frame = frame_buffer.at(0);
      auto curr_frame = frame_buffer.at(1);

      rec.log(
        "logs",
        rerun::TextLog(
          "Number of matches found: " + std::to_string(matches.size())
        )
          .with_level(rerun::TextLogLevel::Info)
      );

      std::vector<rerun::Position2D> points_2d_curr(matches.size());
      std::vector<rerun::Position2D> points_2d_prev(matches.size());

      // Print out matched keypoint pairs
      for (const auto& match : matches) {
        const auto& prev_kp = prev_frame.keypoints[match.trainIdx];
        const auto& curr_kp = curr_frame.keypoints[match.queryIdx];

        points_2d_prev.emplace_back(prev_kp.pt.x, prev_kp.pt.y);
        points_2d_curr.emplace_back(curr_kp.pt.x, curr_kp.pt.y);
      }

      uint32_t width = prev_frame.image.cols;
      uint32_t height = prev_frame.image.rows;

      rec.log("image", rerun::Image::from_greyscale8(img, {width, height}));

      // rec.log(
      //   "prev/image",
      //   rerun::Image::from_greyscale8(prev_frame.image, {width, height})
      // );

      // rec.log(
      //   "curr/image",
      //   rerun::Image::from_greyscale8(curr_frame.image, {width, height})
      // );

      // rec.log(
      //   "prev/keypoints",
      //   rerun::Points2D(points_2d_prev).with_colors(rerun::Color(0, 0, 255))
      // );
      // rec.log(
      //   "curr/keypoints",
      //   rerun::Points2D(points_2d_curr).with_colors(rerun::Color(0, 255, 0))
      // );

      // cv::Mat concat_image;
      // cv::hconcat(
      //   frame_buffer.begin()->image,
      //   (frame_buffer.begin() + 1)->image,
      //   concat_image
      // );
      // rec.log(
      //   "matches",
      //   rerun::Image::from_greyscale8(concat_image, {width * 2, height})
      // );
    }
  }

  //   // TODO: Read focus_on_vehicle value from a file.
  //   //    This is a configuration parameter.
  //   cv::Rect vehicle_frame(535, 180, 180, 150);
  //   if (focus_on_vehicle) {
  //     keypoints.erase(
  //       std::remove_if(
  //         keypoints.begin(),
  //         keypoints.end(),
  //         [&vehicle_frame](const cv::KeyPoint& kp) {
  //           return !vehicle_frame.contains(kp.pt);
  //         }
  //       ),
  //       keypoints.end()
  //     );
  //   }

  //   // TODO: Same with limit_keypoints.
  //   if (limit_keypoints) {
  //     int max_keypoints = 50;

  //     if (detector_type ==
  //         "SHITOMASI") {  // there is no response info, so keep the first
  //                         // 50 as they are sorted in descending quality
  //                         order
  //       keypoints.erase(keypoints.begin() + max_keypoints,
  //       keypoints.end());
  //     }
  //     cv::KeyPointsFilter::retainBest(keypoints, max_keypoints);
  //     std::cout << " NOTE: Keypoints have been limited!" << std::endl;
  //   }

  //   ring_buffer.begin()->keypoints = keypoints;

  //   total_keypoints += static_cast<int>(keypoints.size());

  //   std::cout << "Extracting keypoint descriptors" << std::endl;

  //   cv::Mat descriptors;
  //   // TODO: Move this function to a factory.
  //   DescribeKeypoints(
  //     ring_buffer.begin()->keypoints,
  //     ring_buffer.begin()->image,
  //     descriptors,
  //     descriptor_type
  //   );

  //   // push descriptors for current frame to end of data buffer
  //   ring_buffer.begin()->descriptors = descriptors;

  //   if (ring_buffer.size() > 1) {
  //     std::cout << "Matching keypoint descriptors" << std::endl;

  //     // TODO: Encapsulate matching procedure in a custom Matcher class.
  //     std::vector<cv::DMatch> matches;
  //     MatchDescriptors(
  //       (ring_buffer.begin() + 1)->descriptors,
  //       ring_buffer.begin()->descriptors,
  //       matches,
  //       matcher_type,
  //       selector_type
  //     );

  //     total_matches += static_cast<int>(matches.size());

  //     ring_buffer.begin()->keypoints_matches = matches;

  //     if (visualize_matches) {
  //       cv::Mat match_img = (ring_buffer.begin()->image).clone();
  //       cv::drawMatches(
  //         (ring_buffer.begin() + 1)->image,
  //         (ring_buffer.begin() + 1)->keypoints,
  //         ring_buffer.begin()->image,
  //         ring_buffer.begin()->keypoints,
  //         matches,
  //         match_img,
  //         cv::Scalar::all(-1),
  //         cv::Scalar::all(-1),
  //         std::vector<char>(),
  //         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
  //       );

  //       std::string window_name =
  //         "Matching keypoints between two camera images";
  //       cv::namedWindow(window_name, 7);
  //       cv::imshow(window_name, match_img);
  //       std::cout << "Press key to continue to next image" << std::endl;
  //       cv::waitKey(0);  // wait for key to be pressed
  //     }
  //   }
  // }

  // std::cout << "Total keypoints: " << total_keypoints << std::endl;
  // std::cout << "Total matches: " << total_matches << std::endl;

  return 0;
}
