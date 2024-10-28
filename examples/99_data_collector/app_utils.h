#pragma once

#include "opencv2/opencv.hpp"
#include "highfive/H5Easy.hpp"
#include "nlohmann/json.hpp"
#include "zmq_addon.hpp"

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

template <typename T>
static void AddDataIntoDataSet(std::unique_ptr<HighFive::DataSet>& dataset, const std::vector<std::size_t>& shape, T* data) {
  auto current_dims = dataset->getSpace().getDimensions();
  size_t current_rows = current_dims[0];
  size_t new_rows = current_rows + 1;
  std::vector<std::size_t> resize = {new_rows}, offset = {current_rows}, count = {1};
  for (const auto& l : shape) {
    resize.push_back(l);
    offset.push_back(0);
    count.push_back(l);
  }
  dataset->resize(resize);
  dataset->select(offset, count).write_raw(data);
}

template <typename T>
static HighFive::DataSet CreateDataSet(HighFive::File& file, const std::string& name, const std::vector<std::size_t>& shape,
                                int compression_level = 0) {
  std::vector<size_t> dims = {0};
  std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED};
  std::vector<hsize_t> chunk_size = {1};
  for (const auto& l : shape) {
    dims.push_back(l);
    max_dims.push_back(l);
    chunk_size.push_back(l);
  }
  HighFive::DataSetCreateProps props;
  props.add(HighFive::Chunking(chunk_size));
  if (compression_level != 0) {
    props.add(HighFive::Deflate(compression_level));
  }
  return file.createDataSet<T>(name, HighFive::DataSpace(dims, max_dims), props);
}