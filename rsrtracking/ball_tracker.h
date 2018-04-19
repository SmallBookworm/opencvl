//
// Created by peng on 18-3-1.
//

#ifndef DEPTHTRACK_BALL_TRACKER_H
#define DEPTHTRACK_BALL_TRACKER_H

#define SENSEANGLE (0.0 / 180 * M_PI)
#define HANGLE (64.0 / 180 * M_PI)
#define VANGLE (41.0 / 180 * M_PI)

#include<thread>
#include <iostream>
#include <zconf.h>
#include <future>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "ring_watcher.h"
#include "deviationPosition.h"

class Tracker {
public:
    Tracker();

    //define operator()
    int operator()(DeviationPosition &position);

    int test();

private:
    int frameI;
    //x,y,r,score
    std::vector<cv::Vec4f> ballCoordinates;
    //x,y,z(attention:z!=depth,coordinate system is changed)
    std::vector<cv::Vec3f> realCoordinates;
    //frame,points size,depth,
    std::vector<cv::Vec3f> ballInfo;
    //x,y,z,r
    std::vector<cv::Vec4f> reBall;
    //frame,points size,depth,
    std::vector<cv::Vec3f> reBallInfo;
    RingWatcher ringWatcher;
    cv::Point2f dValue;
    bool reboundTest;

    cv::Ptr<cv::BackgroundSubtractorMOG2> pBackgroundKnn = cv::createBackgroundSubtractorMOG2();


    template<typename T>
    double distance(T x1, T x2, T y1, T y2);

    double realDistance(cv::Vec3f point1, cv::Vec3f point2);

    //参与点乘的两个Mat矩阵的数据类型（type）只能是 CV_32F、 CV_64FC1、 CV_32FC2、 CV_64FC2 这4种类型中的一种
    cv::Mat leastSquares(cv::Mat inMat, cv::Mat outMat);

    cv::Vec3f x2curveFitting(std::vector<float> x, std::vector<float> y);

    std::vector<float> curveFitting(std::vector<float> x, std::vector<float> y, int dimension);

    //-1 insufficient information(restart),1 pass,2 not pass,3 not sure
    int passCF();

    //-1 no ball,0 ball run,1 pass,2 not pass,3 not sure.don't clear info when it return 0,1,2,3.
    int isPassed(cv::Mat &frame, rs2::depth_frame depthFrame);

    //-1 no ball,0 ball run,1 pass,2 not pass.auto clear.
    int surePassed(cv::Mat &frame, rs2::depth_frame depthFrame);

    std::vector<std::vector<cv::Point>> findAllContours(cv::Mat &input, bool isDepth);

    std::vector<std::vector<cv::Point>> findForegroundContours(
            cv::Mat &scene, double scale);

    int pSum(cv::Mat gray);

    cv::Vec4f getEdgeCircle(std::vector<cv::Point> contour);

    cv::Vec3f getCircleCoordinate(cv::Vec4f circle, cv::Vec3f info, int wWidth, int wHeight);

    std::vector<cv::RotatedRect> getRotatedRect(std::vector<std::vector<cv::Point>>);

    cv::Vec4f getBall(std::vector<std::vector<cv::Point>> contours, cv::Mat &result, rs2::depth_frame depthFrame);

    cv::Vec4f getReBall(std::vector<std::vector<cv::Point>> contours, cv::Mat &result, rs2::depth_frame depthFrame);

    void clearInfo();

    cv::Rect selectROIDepth(std::string windowName, cv::Mat &depthMat);

    float getCircleDepth(cv::Vec4f circle, rs2::depth_frame depthFrame);

    template<typename T>
    float getRectDepth(cv::Rect circle, cv::Mat &depthMat);
};

#endif //DEPTHTRACK_BALL_TRACKER_H
