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

    int getRing(cv::Mat &result);
    cv::Vec3b getColor(cv::Mat &res);

private:

};


#endif //RSRTRACKING_RING_WATCHER_H
