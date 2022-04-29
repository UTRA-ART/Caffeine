/*
TODO:
    - setup zed stereo cam package 
    - read single image from stereo cam 
    - publish result to ros friendly format 
*/
#include "ros/ros.h"


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
// #include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
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
// https://github.com/microsoft/onnxruntime/issues/3124 for onnxruntime setup with catkin cmake system 


using namespace std;
// using namespace cv;


template <typename T>
T vectorProduct(const std::vector<T>& v)
{
    return accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
}

/**
 * @brief Operator overloading for printing vectors
 * @tparam T
 * @param os
 * @param v
 * @return std::ostream&
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    os << "[";
    for (int i = 0; i < v.size(); ++i)
    {
        os << v[i];
        if (i != v.size() - 1)
        {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

/**
 * @brief Print ONNX tensor data type
 * https://github.com/microsoft/onnxruntime/blob/rel-1.6.0/include/onnxruntime/core/session/onnxruntime_c_api.h#L93
 * @param os
 * @param type
 * @return std::ostream&
 */
std::ostream& operator<<(std::ostream& os,
                         const ONNXTensorElementDataType& type)
{
    switch (type)
    {
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED:
            os << "undefined";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            os << "float";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            os << "uint8_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            os << "int8_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
            os << "uint16_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
            os << "int16_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            os << "int32_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
            os << "int64_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
            os << "std::string";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
            os << "bool";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            os << "float16";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
            os << "double";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
            os << "uint32_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
            os << "uint64_t";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX64:
            os << "float real + float imaginary";
            break;
        case ONNXTensorElementDataType::
            ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX128:
            os << "double real + float imaginary";
            break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
            os << "bfloat16";
            break;
        default:
            break;
    }

    return os;
}

std::vector<std::string> readLabels(std::string& labelFilepath)
{
    std::vector<std::string> labels;
    std::string line;
    std::ifstream fp(labelFilepath);
    while (std::getline(fp, line))
    {
        labels.push_back(line);
    }
    return labels;
}



unsigned char getMedian(std::vector<unsigned char> input2vec) {
  std::nth_element(input2vec.begin(), input2vec.begin() + input2vec.size() / 2, input2vec.end());
  return input2vec[input2vec.size() / 2];
}

std::vector<unsigned char> convertToQuadrant(cv::Mat input, int r_min, int r_max,
                                            int c_min, int c_max) {
  std::vector<unsigned char> output;

  for (int r = r_min; r < r_max; r++) {
    for (int c = c_min; c < c_max; c++) {
      output.emplace_back(input.at<unsigned char>(r, c));
    }
  }

  return output;
}

std::pair<cv::Mat, cv::Mat> find_edge_channel(cv::Mat img) {
    /*
        find_edge_channel finds the canny edge detections and its inverse using a 4 quadrant algorithm. 
        Doing this allows the model to be more robust against changing light conditions throughout the image. 
    */

    int width = img.cols;
    int height = img.rows;
    int depth = img.channels();

    cv::Mat edges_mask = cv::Mat::zeros(cv::Size(180,330), CV_8UC1);
    cv::Mat edges_mask_inv = cv::Mat::zeros(cv::Size(180,330), CV_8UC1);
    cv::Mat gray_im = cv::Mat::zeros(cv::Size(180,330), CV_8UC1);

    // convert to grayscale 
    cv::cvtColor(img,gray_im,cv::COLOR_BGR2GRAY);

    // gray_im = cv2.GaussianBlur(gray_im,(3,3),0)
    // Separate into quadrants
    std::vector<unsigned char> quad1 = convertToQuadrant(gray_im,0,static_cast<int>(height/2),
                                                            0,static_cast<int>(width/2));

    std::vector<unsigned char> quad2 = convertToQuadrant(gray_im,0,static_cast<int>(height/2),
                                                            static_cast<int>(width/2),width);

    std::vector<unsigned char> quad3 = convertToQuadrant(gray_im,static_cast<int>(height/2),height,
                                                            static_cast<int>(width/2),width);

    std::vector<unsigned char> quad4 = convertToQuadrant(gray_im,static_cast<int>(height/2),height,
                                                            0,static_cast<int>(width/2));
    double med1 = static_cast<double>(getMedian(quad1));
    double med2 = static_cast<double>(getMedian(quad2));
    double med3 = static_cast<double>(getMedian(quad3));
    double med4 = static_cast<double>(getMedian(quad4));

    // Find edges for each of the quadrants
    double l1 = std::max(0.,std::floor((1-0.205)*med1));
    double u1 = std::min(255.,std::floor((1+0.205)*med1));
    std::vector<unsigned char> quad1_int(quad1.begin(),quad1.end());
    cv::Mat quad1_mat_flat(quad1_int,CV_8UC1);
    cv::Mat quad1_mat = quad1_mat_flat.reshape(1,static_cast<int>(height/2));
    cv::Mat e1;
    cv::Canny(quad1_mat,e1,l1,u1);

    double l2 = std::max(0.,std::floor((1-0.205)*med2));
    double u2 = std::min(255.,std::floor((1+0.205)*med2));
    std::vector<unsigned char> quad2_int(quad2.begin(),quad2.end());
    cv::Mat quad2_mat_flat(quad2_int,CV_8UC1);
    cv::Mat quad2_mat = quad2_mat_flat.reshape(1,static_cast<int>(height/2));
    cv::Mat e2;
    cv::Canny(quad2_mat,e2,l2,u2);

    double l3 = std::max(0.,std::floor((1-0.205)*med3));
    double u3 = std::min(255.,std::floor((1+0.205)*med3));
    std::vector<unsigned char> quad3_int(quad3.begin(),quad3.end());
    cv::Mat quad3_mat_flat(quad3_int,CV_8UC1);
    cv::Mat quad3_mat = quad3_mat_flat.reshape(1,static_cast<int>(height/2));
    cv::Mat e3;
    cv::Canny(quad3_mat,e3,l3,u3);

    double l4 = std::max(0.,std::floor((1-0.205)*med4));
    double u4 = std::min(255.,std::floor((1+0.205)*med4));
    std::vector<unsigned char> quad4_int(quad4.begin(),quad4.end());
    cv::Mat quad4_mat_flat(quad4_int,CV_8UC1);
    cv::Mat quad4_mat = quad4_mat_flat.reshape(1,static_cast<int>(height/2));
    cv::Mat e4;
    cv::Canny(quad4_mat,e4,l4,u4);

    // Stitch the edges together
    int new_rows = std::max(e1.rows+e3.rows,e2.rows+e4.rows);
    int new_cols = std::max(e1.cols+e2.cols,e3.cols+e4.cols);
    edges_mask = cv::Mat::zeros(new_rows, new_cols, CV_8UC1);

    e1.copyTo(edges_mask(cv::Rect(0,0,e1.cols,e1.rows)));
    e2.copyTo(edges_mask(cv::Rect(e1.cols,0,e2.cols,e2.rows)));
    e4.copyTo(edges_mask(cv::Rect(0,e1.rows,e3.cols,e3.rows)));
    e3.copyTo(edges_mask(cv::Rect(e1.cols,e1.rows,e4.cols,e4.rows)));

    cv::bitwise_not(edges_mask,edges_mask_inv);

    return std::make_pair(edges_mask, edges_mask_inv);
}


cv::Mat hwc2chw(const cv::Mat &image){
    // permutes image data from HWC to CHW 
    std::vector<cv::Mat> rgb_images;
    cv::split(image, rgb_images);

    cv::Mat m_flat_r = rgb_images[0].reshape(1, 1); 
    cv::Mat m_flat_g = rgb_images[1].reshape(1, 1); 
    cv::Mat m_flat_b = rgb_images[2].reshape(1, 1); 
    cv::Mat m_flat_e1 = rgb_images[3].reshape(1, 1); 
    cv::Mat m_flat_e2 = rgb_images[4].reshape(1, 1); 

    cv::Mat matArray[] = { m_flat_r, m_flat_g, m_flat_b, m_flat_e1, m_flat_e2};

    cv::Mat flat_image; 

    cv::hconcat(matArray, 5, flat_image );
    return flat_image;
}


int main(int argc, char **argv)
{
    // Initialize ROS node 
    ros::init(argc, argv, "cv_model");
    ros::NodeHandle n;
    // ros::Publisher model_pub = n.advertise<???>("/cv_model/output", 1);
    ros::Rate loop_rate(30);

    // Load in .onnx model 
    string instance_name{"unet_inference"};
    string model_path{"/home/kajanan/Downloads/unet_with_sigmoid.onnx"};

    Ort::Env env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, instance_name.c_str());
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);

    // Set GPU settings. To switch to TensorRT accelerator at some point. 
    OrtCUDAProviderOptions cuda_options{};
    sessionOptions.AppendExecutionProvider_CUDA(cuda_options);
    // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tensorrt(sessionOptions, 0));
    // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(sessionOptions, 0));

    sessionOptions.SetGraphOptimizationLevel(
    GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    Ort::Session session(env, model_path.c_str(), sessionOptions);
    Ort::AllocatorWithDefaultOptions allocator;

    // Get input and output infromation for .onnx pipeline
    size_t numInputNodes = session.GetInputCount();
    size_t numOutputNodes = session.GetOutputCount();
    // std::cout << "Number of Input Nodes: " << numInputNodes << std::endl;
    // std::cout << "Number of Output Nodes: " << numOutputNodes << std::endl;

    const char* inputName = session.GetInputName(0, allocator);
    // std::cout << "Input Name: " << inputName << std::endl;
    Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(0);
    auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
    ONNXTensorElementDataType inputType = inputTensorInfo.GetElementType();
    // std::cout << "Input Type: " << inputType << std::endl;
    std::vector<int64_t> inputDims = inputTensorInfo.GetShape();
    // std::cout << "Input Dimensions: " << inputDims << std::endl;

    const char* outputName = session.GetOutputName(0, allocator);
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

    std::vector<const char*> inputNames{inputName};
    std::vector<const char*> outputNames{outputName};

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);


    std::cout << "ros node starting" << std::endl;

    while(n.ok())
    {
        ros::spinOnce();


        // <--- TODO: subscribe to ZED stereo cam image in plave of cv::imread ---> 

        cv::Mat img = cv::imread("/home/kajanan/Downloads/road.jpg", cv::IMREAD_COLOR);
        cv::Mat cropped_raw_image;
        // cv::resize(img, cropped_raw_image, cv::Size(1280, 720), cv::INTER_AREA)
        cv::resize(img, cropped_raw_image, cv::Size(330, 180), cv::INTER_LINEAR);

        // cv::Mat cropped_raw_image = cv::Mat::zeros(cv::Size(330,180), CV_8UC3);
        std::pair<cv::Mat, cv::Mat> edges = find_edge_channel(cropped_raw_image);

        cv::Mat edges_reg;
        cv::Mat edges_inv;

        // cv::imwrite("/home/utra-art/Desktop/spencer_workspace/caffeine-ws/src/cv/src/lane_detection/edge.png", std::get<0>(edges));
        // cv::imwrite("/home/utra-art/Desktop/spencer_workspace/caffeine-ws/src/cv/src/lane_detection/edge_inv.png", std::get<1>(edges));

        std::get<0>(edges).convertTo(edges_reg, CV_32F, 1.0 / 255, 0);
        std::get<1>(edges).convertTo(edges_inv, CV_32F, 1.0 / 255, 0);

        // cv::resize(img, cropped_raw_image, cv::Size(330, 180), cv::INTER_LINEAR);
        // cv::resize(img, cropped_raw_image, cv::Size(330, 180), cv::INTER_LINEAR);

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
        cv::merge(input_stack, 5, input_merged);
        input_permuted = hwc2chw(input_merged);
        
        std::vector<float> inputTensorValues(inputTensorSize);
        inputTensorValues.assign(input_permuted.begin<float>(),
                             input_permuted.end<float>());

        std::vector<Ort::Value> inputTensors;
        std::vector<Ort::Value> outputTensors;

        inputTensors.push_back(Ort::Value::CreateTensor<float>(
            memoryInfo, inputTensorValues.data(), inputTensorSize, inputDims.data(),
            inputDims.size()));
        outputTensors.push_back(Ort::Value::CreateTensor<float>(
            memoryInfo, outputTensorValues.data(), outputTensorSize,
            outputDims.data(), outputDims.size()));


        session.Run(Ort::RunOptions{nullptr}, inputNames.data(),
                    inputTensors.data(), 1, outputNames.data(),
                    outputTensors.data(), 1);

        // outputTensors.data() = outputTensors.data() * 255;
        cv::Mat raw_output = cv::Mat(180,330,CV_32FC1,(float*)outputTensorValues.data());
        // raw_output can likely be used directly for next steps, either vectorization or projection 
        // into cost maps. Only need to scale with CV_8UC1 for visualization purposes. 

        // <--- TODO: publish model output to model output topic for cost maps ---> 

        cv::Mat scaled_output;
        raw_output.convertTo(scaled_output, CV_8UC1, 1, 0);
        scaled_output = scaled_output * 255;

        // Visualizes data 
        cv::imshow("Model input", cropped_raw_image);
        cv::imshow("Model output", scaled_output);
        int k = cv::waitKey(1);
        if (k == 'q'){
            break;
        }

        // model_pub.publish(data);
        loop_rate.sleep();
    }


    return 0;
}

