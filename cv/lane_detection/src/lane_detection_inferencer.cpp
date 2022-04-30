#include "lane_detection/lane_detection_inferencer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "onnxruntime_c_api.h"
#include "onnxruntime_cxx_api.h"
#include "opencv2/opencv.hpp"

namespace
{

constexpr int NETWORK_INPUT_WIDTH{330};
constexpr int NETWORK_INPUT_HEIGHT{180};
constexpr int NETWORK_INPUT_CHANNEL{5};
constexpr int NETWORK_OUTPUT_WIDTH{330};
constexpr int NETWORK_OUTPUT_HEIGHT{180};

template <typename T> T vectorProduct(const std::vector<T> &v)
{
    return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
}

uint8_t getMedian(std::vector<uint8_t> input)
{
    if (input.empty()) {
        throw std::invalid_argument("Received empty input vector!");
    }

    std::nth_element(input.begin(), input.begin() + input.size() / 2,
                     input.end());
    return input[input.size() / 2];
}

std::vector<uint8_t> convertToQuadrant(cv::Mat input, const int r_min,
                                       const int r_max, const int c_min,
                                       const int c_max)
{
    std::vector<uint8_t> output;
    for (int r = r_min; r < r_max; ++r) {
        for (int c = c_min; c < c_max; ++c) {
            output.emplace_back(input.at<uchar>(r, c));
        }
    }
    return output;
}

cv::Mat getQuadrantEdges(cv::Mat grayscaled_image, const int row_start,
                         const int row_end, const int col_start,
                         const int col_end)
{
    const std::vector<uint8_t> quad{convertToQuadrant(
        grayscaled_image, row_start, row_end, col_start, col_end)};

    const double median{static_cast<double>(getMedian(quad))};
    const double lower_bound{std::max(0.0, std::floor((1 - 0.205) * median))};
    const double upper_bound{std::min(255.0, std::floor((1 + 0.205) * median))};
    cv::Mat quad_cv_mat{
        cv::Mat(quad, CV_8UC1)
            .reshape(1, static_cast<int>(grayscaled_image.rows / 2))};
    cv::Mat edges;
    cv::Canny(quad_cv_mat, edges, lower_bound, upper_bound);
    return edges;
}

cv::Mat getEdgeChannel(cv::Mat grayscaled_image)
{
    cv::Mat quad1_edges{getQuadrantEdges(
        grayscaled_image, 0, static_cast<int>(grayscaled_image.rows / 2), 0,
        static_cast<int>(grayscaled_image.cols / 2))};
    cv::Mat quad2_edges{getQuadrantEdges(
        grayscaled_image, 0, static_cast<int>(grayscaled_image.rows / 2),
        static_cast<int>(grayscaled_image.cols / 2), grayscaled_image.cols)};
    cv::Mat quad3_edges{getQuadrantEdges(
        grayscaled_image, static_cast<int>(grayscaled_image.rows / 2),
        grayscaled_image.rows, static_cast<int>(grayscaled_image.cols / 2),
        grayscaled_image.cols)};
    cv::Mat quad4_edges{getQuadrantEdges(
        grayscaled_image, static_cast<int>(grayscaled_image.rows / 2),
        grayscaled_image.rows, 0, static_cast<int>(grayscaled_image.cols / 2))};

    // stitch edges together
    const int new_rows{std::max(quad1_edges.rows + quad3_edges.rows,
                                quad2_edges.rows + quad4_edges.rows)};
    const int new_cols{std::max(quad1_edges.cols + quad3_edges.cols,
                                quad2_edges.cols + quad4_edges.cols)};

    cv::Mat edges_mask = cv::Mat::zeros(new_rows, new_cols, CV_8UC1);

    quad1_edges.copyTo(
        edges_mask(cv::Rect(0, 0, quad1_edges.cols, quad1_edges.rows)));
    quad2_edges.copyTo(edges_mask(
        cv::Rect(quad1_edges.cols, 0, quad2_edges.cols, quad2_edges.rows)));
    quad4_edges.copyTo(edges_mask(
        cv::Rect(0, quad1_edges.rows, quad3_edges.cols, quad3_edges.rows)));
    quad3_edges.copyTo(
        edges_mask(cv::Rect(quad1_edges.cols, quad1_edges.rows,
                            quad4_edges.cols, quad4_edges.rows)));
    return edges_mask;
}

cv::Mat getInverseEdgeChannel(cv::Mat edge_channel)
{
    cv::Mat inverse_edge_channel;
    cv::bitwise_not(edge_channel, inverse_edge_channel);
    return inverse_edge_channel;
}

cv::Mat combineImages(cv::Mat normalized_img, cv::Mat normalized_edges,
                      cv::Mat normalized_edges_inv)
{

    cv::Mat R, G, B;
    cv::extractChannel(normalized_img, B, 0);
    cv::extractChannel(normalized_img, G, 1);
    cv::extractChannel(normalized_img, R, 2);

    cv::Mat input_stacked[NETWORK_INPUT_CHANNEL] = {B, G, R, normalized_edges,
                                                    normalized_edges_inv};

    cv::Mat input_merged;
    cv::merge(input_stacked, NETWORK_INPUT_CHANNEL, input_merged);
    return input_merged;
}

cv::Mat convertHWC2CHW(cv::Mat input_image)
{
    std::vector<cv::Mat> rgb_images;
    cv::split(input_image, rgb_images);

    cv::Mat matArray[NETWORK_INPUT_CHANNEL];
    for (int i = 0; i < NETWORK_INPUT_CHANNEL; ++i) {
        matArray[i] = rgb_images[i].reshape(1, 1);
    }

    cv::Mat flat_image;
    cv::hconcat(matArray, NETWORK_INPUT_CHANNEL, flat_image);
    return flat_image;
}

Ort::SessionOptions getOrtSessionOptions(int intra_op_num_threads,
                                         OrtCUDAProviderOptions cuda_options)
{
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(intra_op_num_threads);
    session_options.AppendExecutionProvider_CUDA(cuda_options);
    session_options.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    return session_options;
}

} // namespace

LaneDetectionInferencer::LaneDetectionInferencer(std::string instance_name,
                                                 std::string model_path)
    : instance_name_{std::move(instance_name)}, model_path_{std::move(
                                                    model_path)},
      env_{OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, instance_name_.c_str()},
      cuda_options_{OrtCUDAProviderOptions()},
      session_options_{getOrtSessionOptions(1, cuda_options_)},
      session_{env_, model_path_.c_str(), session_options_},
      allocator_{Ort::AllocatorWithDefaultOptions()},
      input_names_{session_.GetInputName(0, allocator_)},
      output_names_{session_.GetOutputName(0, allocator_)},
      input_dims_{
          session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape()},
      output_dims_{
          session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape()},
      input_tensor_size_(vectorProduct(input_dims_)),
      output_tensor_size_(vectorProduct(output_dims_)),
      output_tensor_values_{std::vector<float>(output_tensor_size_)},
      memory_info_{Ort::MemoryInfo::CreateCpu(
          OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault)}
{
}

cv::Mat LaneDetectionInferencer::getLaneDetectionMask(cv::Mat input_image)
{
    return postprocessImage(performInference(preprocessImage(input_image)));
}

cv::Mat LaneDetectionInferencer::preprocessImage(cv::Mat input_img)
{
    cv::Mat resized_img;
    cv::resize(input_img, resized_img,
               cv::Size(NETWORK_INPUT_WIDTH, NETWORK_INPUT_HEIGHT),
               cv::INTER_LINEAR);

    cv::Mat input_gray_img;
    cv::cvtColor(resized_img, input_gray_img, cv::COLOR_BGR2GRAY);
    cv::Mat edges = getEdgeChannel(input_gray_img);
    cv::Mat edges_inv = getInverseEdgeChannel(edges);

    cv::Mat normalized_edges;
    cv::Mat normalized_edges_inv;
    edges.convertTo(normalized_edges, CV_32F, 1.0f / 255.0f, 0.0f);
    edges_inv.convertTo(normalized_edges_inv, CV_32F, 1.0f / 255.0f, 0.0f);

    cv::Mat normalized_img;
    resized_img.convertTo(normalized_img, CV_32F, 1.0f / 255.0f, 0.0f);

    return convertHWC2CHW(
        combineImages(normalized_img, normalized_edges, normalized_edges_inv));
}

cv::Mat LaneDetectionInferencer::performInference(cv::Mat preprocessed_img)
{
    std::vector<float> input_tensor_values(input_tensor_size_);
    input_tensor_values.assign(preprocessed_img.begin<float>(),
                               preprocessed_img.end<float>());

    std::vector<Ort::Value> input_tensors;
    std::vector<Ort::Value> output_tensors;

    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        memory_info_, input_tensor_values.data(), input_tensor_size_,
        input_dims_.data(), input_dims_.size()));

    output_tensors.push_back(Ort::Value::CreateTensor<float>(
        memory_info_, output_tensor_values_.data(), output_tensor_size_,
        output_dims_.data(), output_dims_.size()));

    session_.Run(Ort::RunOptions{nullptr}, input_names_.data(),
                 input_tensors.data(), 1, output_names_.data(),
                 output_tensors.data(), 1);

    // TODO: do not use C style cast, look for safer way to do this
    return cv::Mat(NETWORK_OUTPUT_HEIGHT, NETWORK_OUTPUT_WIDTH, CV_32FC1,
                   (float *)output_tensor_values_.data());
}

cv::Mat LaneDetectionInferencer::postprocessImage(cv::Mat raw_inference_output)
{
    cv::Mat scaled_output;
    raw_inference_output.convertTo(scaled_output, CV_8UC1, 1, 0);
    return scaled_output * 255;
}
