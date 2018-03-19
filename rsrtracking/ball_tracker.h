//
// Created by peng on 18-3-1.
//

#ifndef DEPTHTRACK_BALL_TRACKER_H
#define DEPTHTRACK_BALL_TRACKER_H

#include<thread>
#include <iostream>
#include <zconf.h>
#include <future>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

class Tracker {
public:
    Tracker();

    //define operator()
    int operator()(std::future<int> &fut);

    void test();

private:
    int frameI;
    //x,y,r,score
    std::vector<cv::Vec4f> ballCoordinates;
    //x,y,z
    std::vector<cv::Vec3f> realCoordinates;
    //frame,points size,depth,
    std::vector<cv::Vec3f> ballInfo;
    //x,y,r,depth
    cv::Vec4f ring;
    //x,y,z
    cv::Vec3f ringCoordinate;
    float rRingR;
    int width;
    int height;
    cv::Ptr<cv::BackgroundSubtractorMOG2> pBackgroundKnn = cv::createBackgroundSubtractorMOG2();


    template<typename T>
    double distance(T x1, T x2, T y1, T y2);

    double realDistance(cv::Vec3f point1, cv::Vec3f point2);

    //参与点乘的两个Mat矩阵的数据类型（type）只能是 CV_32F、 CV_64FC1、 CV_32FC2、 CV_64FC2 这4种类型中的一种
    cv::Mat leastSquares(cv::Mat inMat, cv::Mat outMat);

    cv::Vec3f x2curveFitting(std::vector<float> x, std::vector<float> y);

    std::vector<float> curveFitting(std::vector<float> x, std::vector<float> y, int dimension);

    int passCF();

    int isPassed(cv::Mat &frame, rs2::depth_frame depthFrame);

    std::vector<std::vector<cv::Point>> findAllContours(cv::Mat &input);

    std::vector<std::vector<cv::Point>> findForegroundContours(
            cv::Mat scene, double scale);

    cv::Vec4f getEdgeCircle(std::vector<cv::Point> contour);

    cv::Vec3f getCircleCoordinate(cv::Vec4f circle, cv::Vec3f info, int wWidth = 512, int wHeight = 424);

    std::vector<cv::RotatedRect> getRotatedRect(std::vector<std::vector<cv::Point>>);

    cv::Vec4f getBall(std::vector<std::vector<cv::Point>> contours, cv::Mat &result, rs2::depth_frame depthFrame);

    int getRing(std::vector<std::vector<cv::Point>> contours, cv::Mat &result);

    cv::RotatedRect getRingPole(std::vector<cv::Point> contours);


    template<typename T>
    float selectROIDepth(std::string windowName, cv::Mat &depthMat);

    float getCircleDepth(cv::Vec4f circle, rs2::depth_frame depthFrame);

    template<typename T>
    float getRectDepth(cv::Rect circle, cv::Mat &depthMat);
};

#endif //DEPTHTRACK_BALL_TRACKER_H
