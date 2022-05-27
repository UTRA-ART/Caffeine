#ifndef LANE_DETECTION__LANE_DETECTION_INFERENCER_H
#define LANE_DETECTION__LANE_DETECTION_INFERENCER_H

#include "onnxruntime_cxx_api.h"
#include "opencv2/opencv.hpp"

class LaneDetectionInferencer
{

  public:
    LaneDetectionInferencer(std::string instance_name, std::string model_path);
    cv::Mat getLaneDetectionMask(cv::Mat input_img);

  private:
    cv::Mat preprocessImage(cv::Mat input_img);
    cv::Mat performInference(cv::Mat preprocessed_img);
    cv::Mat postprocessImage(cv::Mat raw_inference_output);

    const std::string instance_name_;
    const std::string model_path_;
   
    // TODO: do we need all these variables?
    Ort::Env env_;
    OrtCUDAProviderOptions cuda_options_;
    Ort::SessionOptions session_options_;
    Ort::Session session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<int64_t> input_dims_;
    std::vector<int64_t> output_dims_;
    size_t input_tensor_size_;
    size_t output_tensor_size_;
    std::vector<float> output_tensor_values_;
    Ort::MemoryInfo memory_info_;
};

#endif
