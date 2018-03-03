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


// move-constructible function object (i.e., an object whose class defines operator(), including closures and function objects).

class Tracker {
public:
    Tracker();

    //define operator()
    void operator()(std::future<int> &fut);

    void test();

private:
    std::vector<cv::Point2f> ballCoordinates;
    cv::Vec4f ring;
    cv::Ptr<cv::BackgroundSubtractorMOG2> pBackgroundKnn = cv::createBackgroundSubtractorMOG2();
    int isPassed(cv::Mat &frame);

    std::vector<std::vector<cv::Point>> findAllContours(cv::Mat &input);

    std::vector<std::vector<cv::Point>> findForegroundContours(
            cv::Mat scene, double scale);

    cv::Vec4f getEdgeCircle(std::vector<cv::Point> contour);

    std::vector<cv::RotatedRect> getRotatedRect(std::vector<std::vector<cv::Point>>);

    cv::Vec4f getBall(std::vector<std::vector<cv::Point>> contours, cv::Mat &result);
};

#endif //DEPTHTRACK_BALL_TRACKER_H
