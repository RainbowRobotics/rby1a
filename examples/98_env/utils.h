#pragma once

#include "opencv2/opencv.hpp"
#include "nlohmann/json.hpp"

static nlohmann::json MatToJson(const cv::Mat& mat) {
  nlohmann::json j;
  j["rows"] = mat.rows;
  j["cols"] = mat.cols;
  j["type"] = mat.type();

  std::vector<uint8_t> data;

  if (mat.isContinuous()) {
    data.resize(mat.total() * mat.elemSize());
    std::memcpy(data.data(), mat.data, mat.total() * mat.elemSize());
  } else {
    throw std::runtime_error("cv::Mat is not continuous");
  }

  // Add data to JSON object as a Base64 encoded string or raw data vector
  j["data"] = data;

  return j;
}