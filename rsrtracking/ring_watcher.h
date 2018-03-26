//
// Created by peng on 18-3-19.
//

#ifndef RSRTRACKING_RING_WATCHER_H
#define RSRTRACKING_RING_WATCHER_H

#include <opencv2/opencv.hpp>


class RingWatcher {
public:
    //x,y,r,depth
    cv::Vec4f ring;
    //real x,y,z
    cv::Vec3f coordinate;
    float r;

    int getRing(std::vector<std::vector<cv::Point>> contours, cv::Mat &result);
    //set ring;-1 fail
    int getThresholdRing(cv::Mat &result);

private:
    std::vector<cv::RotatedRect> getRotatedRect(std::vector<std::vector<cv::Point>>);

    std::vector<cv::Rect> getRect(std::vector<std::vector<cv::Point>>);

    cv::RotatedRect getRingPole(std::vector<cv::Point> contours);

    cv::Vec3f getPoleRange(cv::Mat grayFrame);
};


#endif //RSRTRACKING_RING_WATCHER_H
