// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <iostream>
#include <thread>
#include "cv-helpers.hpp"

using namespace std;
using namespace cv;

int thr() try {
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    // Start streaming with default recommended configuration
    pipe.start(cfg);

//    const auto window_name = "Display Image";
//    namedWindow(window_name, WINDOW_AUTOSIZE);
    bool contFlag = true;
    string name = "1";
    VideoWriter vDepth("/home/peng/depth" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 30,
                       Size(640, 480),
                       true);
    VideoWriter vIr("/home/peng/ir" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(640, 480),
                    false);
    VideoWriter vColor("/home/peng/color" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(640, 480),
                       true);

    VideoCapture capture0(0);
    VideoWriter cap0("/home/peng/cap0" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(640, 480),
                     true);
//    VideoCapture capture1(1);
//    VideoWriter cap1("/home/leon/Downloads/cap1" + name + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(640, 480),
//                     true);
    while (contFlag) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::depth_frame depthFrame = data.get_depth_frame();
        rs2::frame depth = color_map(depthFrame);
        rs2::frame color = data.get_color_frame();
        rs2::frame ir = data.get_infrared_frame();
        Mat imaged = frame_to_mat(depth);
        Mat imagec = frame_to_mat(color);
        Mat imagei = frame_to_mat(ir);
        vDepth << imaged;
        vIr << imagei;
        vColor << imagec;
        if (capture0.isOpened()) {
            Mat mat;
            //imshow("fg",mat);
            capture0 >> mat;
            cap0 << mat;
        }
//        if (capture1.isOpened()) {
//            Mat mat;
//            capture1 >> mat;
//            cap1 << mat;
//        }
        // Update the window with new data
//        imshow(window_name, imaged);
        imshow("color", imagec);
//        imshow("imagei", imagei);
        //Mat fuck(Size(w, h), CV_16SC1, (void*)depth.get_data(), Mat::AUTO_STEP);
        //imshow("fuck",fuck);
        contFlag = waitKey(1) < 0;
    }
}
catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

int main(int argc, char *argv[]) {
    thread th1(thr);
    th1.detach();
    getchar();
    return EXIT_SUCCESS;
}
