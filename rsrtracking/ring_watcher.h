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
    //x,y,z
    cv::Vec3f coordinate;
    float r;
private:
    std::vector<cv::RotatedRect> getRotatedRect(std::vector<std::vector<cv::Point>>);

    int getRing(std::vector<std::vector<cv::Point>> contours, cv::Mat &result);

    cv::RotatedRect getRingPole(std::vector<cv::Point> contours);
};


#endif //RSRTRACKING_RING_WATCHER_H
