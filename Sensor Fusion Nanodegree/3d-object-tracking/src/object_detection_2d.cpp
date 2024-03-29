#include "object_detection_2d.h"

#include <fstream>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

void DetectObjects(
    cv::Mat& img,
    std::vector<BoundingBox>& bounding_boxes,
    float confidence_threshold,
    float nms_threshold,
    const std::string& base_path,
    const std::string& classes_file,
    const std::string& model_configuration,
    const std::string& model_weights,
    bool visible) {
  std::vector<std::string> classes;
  std::ifstream ifs(classes_file.c_str());
  std::string line;

  while (getline(ifs, line)) {
    classes.push_back(line);
  }

  cv::dnn::Net net = cv::dnn::readNetFromDarknet(model_configuration, model_weights);
  net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  cv::Mat blob;
  std::vector<cv::Mat> netOutput;
  double scale_factor = 1 / 255.0;
  cv::Size size = cv::Size(416, 416);
  cv::Scalar mean = cv::Scalar(0, 0, 0);
  bool swapRB = false;
  bool crop = false;
  cv::dnn::blobFromImage(img, blob, scale_factor, size, mean, swapRB, crop);

  std::vector<cv::String> names;
  std::vector<int> outLayers = net.getUnconnectedOutLayers();
  std::vector<cv::String> layersNames = net.getLayerNames();

  names.resize(outLayers.size());
  for (size_t i = 0; i < outLayers.size(); ++i) {
    names[i] = layersNames[outLayers[i] - 1];
  }

  net.setInput(blob);
  net.forward(netOutput, names);

  // Scan through all bounding boxes and keep only the ones with high confidence
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  for (auto& i : netOutput) {
    auto data = (float*)i.data;
    for (int j = 0; j < i.rows; ++j, data += i.cols) {
      cv::Mat scores = i.row(j).colRange(5, i.cols);
      cv::Point class_id;
      double confidence;

      // Get the value and location of the maximum score
      cv::minMaxLoc(scores, 0, &confidence, 0, &class_id);
      if (confidence > confidence_threshold) {
        cv::Rect box;
        int cx, cy;
        cx = static_cast<int>((data[0] * img.cols));
        cy = static_cast<int>((data[1] * img.rows));
        box.width = static_cast<int>((data[2] * img.cols));
        box.height = static_cast<int>((data[3] * img.rows));
        box.x = cx - box.width / 2;   // left
        box.y = cy - box.height / 2;  // top

        boxes.push_back(box);
        class_ids.push_back(class_id.x);
        confidences.push_back(static_cast<float>(confidence));
      }
    }
  }

  // perform non-maxima suppression
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold, indices);
  for (int& index : indices) {
    BoundingBox bounding_box;
    bounding_box.roi = boxes[index];
    bounding_box.class_id = class_ids[index];
    bounding_box.confidence = confidences[index];
    bounding_box.id = static_cast<int>(bounding_boxes.size());  // zero-based unique identifier for this bounding box

    bounding_boxes.push_back(bounding_box);
  }

  if (visible) {
    cv::Mat visImg = img.clone();
    for (auto& bounding_box : bounding_boxes) {
      int top;
      int left;
      int width;
      int height;
      top = bounding_box.roi.y;
      left = bounding_box.roi.x;
      width = bounding_box.roi.width;
      height = bounding_box.roi.height;
      cv::rectangle(visImg, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);

      std::string label = cv::format("%.2f", bounding_box.confidence);
      label.append(classes[(bounding_box.class_id)]);
      label.append(":");
      label.append(label);

      // Display label at the top of the bounding box
      int base_line;
      cv::Size label_size = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &base_line);
      top = std::max(top, label_size.height);
      cv::rectangle(
          visImg,
          cv::Point(left, top - round(1.5 * label_size.height)),
          cv::Point(left + round(1.5 * label_size.width), top + base_line),
          cv::Scalar(255, 255, 255),
          cv::FILLED);
      cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
    }

    std::string window_name = "Object classification";
    cv::namedWindow(window_name, 1);
    cv::imshow(window_name, visImg);
    cv::waitKey(0);  // wait for key to be pressed
  }
}
