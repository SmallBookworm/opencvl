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
#include "protonect.h"


class Tracker {
public:
    Tracker();

    static Protonect protonect;

    //define operator()
    void operator()(std::future<int> &fut);

    void test();

private:
    int frameI;
    //x,y,r,score
    std::vector<cv::Vec4f> ballCoordinates;
    //frame,points size,depth,
    std::vector<cv::Vec3f> ballInfo;
    //x,y,r,depth
    cv::Vec4f ring;

    cv::Ptr<cv::BackgroundSubtractorMOG2> pBackgroundKnn = cv::createBackgroundSubtractorMOG2();

    int isPassed(cv::Mat &frame);

    std::vector<std::vector<cv::Point>> findAllContours(cv::Mat &input);

    std::vector<std::vector<cv::Point>> findForegroundContours(
            cv::Mat scene, double scale);

    cv::Vec4f getEdgeCircle(std::vector<cv::Point> contour);

    std::vector<cv::RotatedRect> getRotatedRect(std::vector<std::vector<cv::Point>>);

    cv::Vec4f getBall(std::vector<std::vector<cv::Point>> contours, cv::Mat &result);

    int getRing(std::vector<std::vector<cv::Point>> contours, cv::Mat &result);

    cv::RotatedRect getRingPole(std::vector<cv::Point> contours);

    template<typename T>
    float selectROIDepth(std::string windowName, cv::Mat &depthMat);

    template<typename T>
    float getCircleDepth(cv::Vec4f circle, cv::Mat &depthMat);

    template<typename T>
    float getRectDepth(cv::Rect circle, cv::Mat &depthMat);
};

#endif //DEPTHTRACK_BALL_TRACKER_H
