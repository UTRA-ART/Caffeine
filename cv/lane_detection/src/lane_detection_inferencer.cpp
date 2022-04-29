#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "opencv2/opencv.hpp"

// DELETE UNDER:

#include "ros/ros.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <onnxruntime_cxx_api.h>

#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace
{

constexpr int NETWORK_INPUT_WIDTH{180};
constexpr int NETWORK_INPUT_HEIGHT{330};
constexpr int NETWORK_INPUT_CHANNEL{5};
constexpr int NETWORK_OUTPUT_WIDTH{180};
constexpr int NETWORK_OUTPUT_HEIGHT{330};

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

} // namespace

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "cv_model");
    ros::NodeHandle n;
    // ros::Publisher model_pub = n.advertise<???>("/cv_model/output", 1);
    ros::Rate loop_rate(30);

    // Load in .onnx model
    std::string instance_name{"unet_inference"};
    std::string model_path{"/home/kajanan/Downloads/unet_with_sigmoid.onnx"};

    Ort::Env env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING,
                 instance_name.c_str());
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);

    // Set GPU settings. To switch to TensorRT accelerator at some point.
    OrtCUDAProviderOptions cuda_options{};
    sessionOptions.AppendExecutionProvider_CUDA(cuda_options);
    // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tensorrt(sessionOptions,
    // 0));
    // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(sessionOptions,
    // 0));

    sessionOptions.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    Ort::Session session(env, model_path.c_str(), sessionOptions);
    Ort::AllocatorWithDefaultOptions allocator;

    // Get input and output infromation for .onnx pipeline
    size_t numInputNodes = session.GetInputCount();
    size_t numOutputNodes = session.GetOutputCount();
    // std::cout << "Number of Input Nodes: " << numInputNodes << std::endl;
    // std::cout << "Number of Output Nodes: " << numOutputNodes << std::endl;

    const char *inputName = session.GetInputName(0, allocator);
    // std::cout << "Input Name: " << inputName << std::endl;
    Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(0);
    auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
    ONNXTensorElementDataType inputType = inputTensorInfo.GetElementType();
    // std::cout << "Input Type: " << inputType << std::endl;
    std::vector<int64_t> inputDims = inputTensorInfo.GetShape();
    // std::cout << "Input Dimensions: " << inputDims << std::endl;

    const char *outputName = session.GetOutputName(0, allocator);
    // std::cout << "Output Name: " << outputName << std::endl;
    Ort::TypeInfo outputTypeInfo = session.GetOutputTypeInfo(0);
    auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
    ONNXTensorElementDataType outputType = outputTensorInfo.GetElementType();
    // std::cout << "Output Type: " << outputType << std::endl;
    std::vector<int64_t> outputDims = outputTensorInfo.GetShape();
    // std::cout << "Output Dimensions: " << outputDims << std::endl;

    size_t inputTensorSize = vectorProduct(inputDims);
    size_t outputTensorSize = vectorProduct(outputDims);
    std::vector<float> outputTensorValues(outputTensorSize);

    std::vector<const char *> inputNames{inputName};
    std::vector<const char *> outputNames{outputName};

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    std::cout << "ros node starting" << std::endl;

    while (n.ok()) {
        ros::spinOnce();

        // <--- TODO: subscribe to ZED stereo cam image in plave of cv::imread
        // --->

        cv::Mat img =
            cv::imread("/home/kajanan/Downloads/road.jpg", cv::IMREAD_COLOR);
        cv::Mat cropped_raw_image;
        // cv::resize(img, cropped_raw_image, cv::Size(1280, 720),
        // cv::INTER_AREA)
        cv::resize(img, cropped_raw_image, cv::Size(330, 180),
                   cv::INTER_LINEAR);

        // cv::Mat cropped_raw_image = cv::Mat::zeros(cv::Size(330,180),
        // CV_8UC3);

        cv::Mat gray_image;
        cv::cvtColor(cropped_raw_image, gray_image, cv::COLOR_BGR2GRAY);
        cv::Mat edges = getEdgeChannel(gray_image);
        cv::Mat inv_edges = getInverseEdgeChannel(edges);

        cv::Mat edges_reg;
        cv::Mat edges_inv;

        //
        // cv::imwrite("/home/utra-art/Desktop/spencer_workspace/caffeine-ws/src/cv/src/lane_detection/edge.png",
        // std::get<0>(edges));
        //
        // cv::imwrite("/home/utra-art/Desktop/spencer_workspace/caffeine-ws/src/cv/src/lane_detection/edge_inv.png",
        // std::get<1>(edges));

        edges.convertTo(edges_reg, CV_32F, 1.0 / 255, 0);
        inv_edges.convertTo(edges_inv, CV_32F, 1.0 / 255, 0);

        // cv::resize(mg, cropped_raw_image, cv::Size(330, 180),
        // cv::INTER_LINEAR); cv::resize(img, cropped_raw_image, cv::Size(330,
        // 180), cv::INTER_LINEAR);

        cv::Mat normalized_image;
        cropped_raw_image.convertTo(normalized_image, CV_32F, 1.0 / 255, 0);

        cv::Mat R;
        cv::Mat G;
        cv::Mat B;
        cv::extractChannel(normalized_image, B, 0);
        cv::extractChannel(normalized_image, G, 1);
        cv::extractChannel(normalized_image, R, 2);

        cv::Mat input_stack[5] = {B, G, R, edges_reg, edges_inv};
        cv::Mat input_merged;
        cv::Mat input_permuted;

        cv::imwrite("/tmp/edge.png", edges);
        cv::imwrite("/tmp/inv_edge.png", inv_edges);
        cv::merge(input_stack, 5, input_merged);

        input_permuted = convertHWC2CHW(input_merged);

        std::vector<float> inputTensorValues(inputTensorSize);
        inputTensorValues.assign(input_permuted.begin<float>(),
                                 input_permuted.end<float>());

        std::vector<Ort::Value> inputTensors;
        std::vector<Ort::Value> outputTensors;

        inputTensors.push_back(Ort::Value::CreateTensor<float>(
            memoryInfo, inputTensorValues.data(), inputTensorSize,
            inputDims.data(), inputDims.size()));
        outputTensors.push_back(Ort::Value::CreateTensor<float>(
            memoryInfo, outputTensorValues.data(), outputTensorSize,
            outputDims.data(), outputDims.size()));

        session.Run(Ort::RunOptions{nullptr}, inputNames.data(),
                    inputTensors.data(), 1, outputNames.data(),
                    outputTensors.data(), 1);

        // outputTensors.data() = outputTensors.data() * 255;
        cv::Mat raw_output =
            cv::Mat(180, 330, CV_32FC1, (float *)outputTensorValues.data());
        // raw_output can likely be used directly for next steps, either
        // vectorization or projection into cost maps. Only need to scale with
        // CV_8UC1 for visualization purposes.

        // <--- TODO: publish model output to model output topic for cost maps
        // --->

        cv::Mat scaled_output;
        raw_output.convertTo(scaled_output, CV_8UC1, 1, 0);
        scaled_output = scaled_output * 255;

        // Visualizes data
        cv::imshow("Model input", cropped_raw_image);
        cv::imshow("Model output", scaled_output);
        int k = cv::waitKey(1);
        if (k == 'q') {
            break;
        }

        // model_pub.publish(data);
        loop_rate.sleep();
    }
    return 0;
}
