
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>


dai::Pipeline createPipeline(bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh){
    dai::Pipeline pipeline;

    auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
    auto monoRight   = pipeline.create<dai::node::MonoCamera>();
    auto rgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRGB = pipeline.create<dai::node::XLinkOut>();
    auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
    //auto xoutRectLeft = pipeline.create<dai::node::XLinkOut>();
    //auto xoutRectRight = pipeline.create<dai::node::XLinkOut>();
    auto stereo      = pipeline.create<dai::node::StereoDepth>();
    auto xoutConfidenceMap = pipeline.create<dai::node::XLinkOut>();
    auto xoutDisparity = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth   = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutRGB->setStreamName("rgb");
    //xoutRectLeft->setStreamName("rectified_left");
    //xoutRectRight->setStreamName("rectified_right");
    xoutConfidenceMap->setStreamName("confidence_map");
    xoutDepth->setStreamName("depth");
    xoutDisparity->setStreamName("disparity");


    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // RGBCamera
    rgb->setPreviewSize(300, 300);
    rgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    rgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    rgb->setInterleaved(false);
    rgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

    // int maxDisp = 96;
    // if (extended) maxDisp *= 2;
    // if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    rgb->video.link(xoutRGB->input);

    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);

    stereo->depth.link(xoutDepth->input);
    stereo->disparity.link(xoutDisparity->input);

    //stereo->rectifiedLeft.link(xoutRectLeft->input);
    //stereo->rectifiedRight.link(xoutRectRight->input);

    stereo->confidenceMap.link(xoutConfidenceMap->input);

    return pipeline;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string cameraParamUri;
    int badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence = 200;
    int LRchecktresh = 5;

    badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    badParams += !pnh.getParam("camera_name",  deviceName);
    badParams += !pnh.getParam("lrcheck",      lrcheck);
    badParams += !pnh.getParam("extended",     extended);
    badParams += !pnh.getParam("subpixel",     subpixel);
    badParams += !pnh.getParam("confidence",   confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);

    if (badParams > 0)
    {   
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }

    dai::Pipeline pipeline = createPipeline(lrcheck, extended, subpixel, confidence, LRchecktresh);
    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", 30, false);
    auto rightQueue = device.getOutputQueue("right", 30, false);
    auto rgbQueue = device.getOutputQueue("rgb", 30, false);
    //auto leftRectQueue = device.getOutputQueue("rectified_left", 30, false);
    //auto rightRectQueue = device.getOutputQueue("rectified_right", 30, false);
    auto confidenceMapQueue = device.getOutputQueue("confidence_map", 30, false);

    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto disparityQueue = device.getOutputQueue("disparity", 30, false);

    auto calibrationHandler = device.readCalibration();
   
    dai::rosBridge::ImageConverter leftconverter(deviceName + "_left_camera_optical_frame", true);
    auto leftCameraInfo = leftconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 640, 480,
    dai::Point2f(), dai::Point2f(), dai::CameraBoardSocket::RIGHT); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                    pnh, 
                                                                                    std::string("left/image_mono"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &leftconverter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    leftCameraInfo,
                                                                                    "left");

    leftPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame", true);
    auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, 640, 480,
    dai::Point2f(), dai::Point2f(), dai::CameraBoardSocket::RIGHT); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                     pnh, 
                                                                                     std::string("right/image_mono"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "right");

    rightPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter rgbconverter(deviceName + "_rgb_camera_optical_frame", true);
    auto rgbCameraInfo = rgbconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1920, 1080,
    dai::Point2f(), dai::Point2f(), dai::CameraBoardSocket::RIGHT);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(rgbQueue,
                                                                                    pnh, 
                                                                                    std::string("rgb/image_raw"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &rgbconverter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    rgbCameraInfo,
                                                                                    "rgb");

    rgbPublish.addPublisherCallback();

    // dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftRectPublish(leftRectQueue,
    //                                                                                     pnh, 
    //                                                                                     std::string("left/image_rect"),
    //                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
    //                                                                                     &leftconverter, 
    //                                                                                     std::placeholders::_1, 
    //                                                                                     std::placeholders::_2) , 
    //                                                                                     30,
    //                                                                                     leftCameraInfo,
    //                                                                                     "left");

    // leftRectPublish.addPublisherCallback();

    // dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightRectPublish(rightRectQueue,
    //                                                                                     pnh, 
    //                                                                                     std::string("right/image_rect"),
    //                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
    //                                                                                     &rightconverter, 
    //                                                                                     std::placeholders::_1, 
    //                                                                                     std::placeholders::_2) , 
    //                                                                                     30,
    //                                                                                     rightCameraInfo,
    //                                                                                     "right");

    // rightRectPublish.addPublisherCallback();

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(depthQueue,
                                                                                     pnh, 
                                                                                     std::string("depth/image_rect"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "depth");
    depthPublish.addPublisherCallback();

    dai::rosBridge::DisparityConverter dispConverter(deviceName + "_right_camera_optical_frame", 880, 7.5, 20, 2000);
    dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame> dispPublish(disparityQueue,
                                                                                    pnh, 
                                                                                    std::string("disparity"),
                                                                                    std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, 
                                                                                    &dispConverter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    rightCameraInfo,
                                                                                    "disparity");
    dispPublish.addPublisherCallback();

    ros::spin();
    return 0;
}
