//
// Created by peng on 18-4-6.
//

#ifndef RSRTRACKING_DEVIATIONPOSITION_H
#define RSRTRACKING_DEVIATIONPOSITION_H

#include <opencv2/opencv.hpp>
#include <mutex>

class DeviationPosition {
private:
    //1 pass,2 fail
    int state;
    cv::Point2f point;
    bool used;
    std::mutex coor_mutex;
private:
    bool stop;
    std::mutex stop_mutex;
private:
    //real x,y,z
    cv::Vec3f ring;
    std::mutex ring_mutex;
public:
    explicit DeviationPosition( cv::Vec3f in) {
        state = -1;
        used = true;
        stop = false;
        ring=in;
    }

    void init();

    //-1 no info
    int getPoint(cv::Point2f &out);

    void setPoint(cv::Point2f point, int res);

    void setStop(bool f);

    bool getStop();

    cv::Vec3f getRing();
};


#endif //RSRTRACKING_DEVIATIONPOSITION_H
