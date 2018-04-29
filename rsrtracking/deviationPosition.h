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
    bool standby;
    std::mutex stby_mutex;
private:
    //real x,y,z
    cv::Vec3f ring;
    std::mutex ring_mutex;
public:
    explicit DeviationPosition() {
        state = -1;
        used = true;
        stop = false;
        standby = true;
    }

    void init(cv::Vec3f in);

    void await();

    //-1 no info
    int getPoint(cv::Point2f &out);

    void setPoint(cv::Point2f point, int res);

    void setStop(bool f);

    bool getStop();

    void setStby(bool s);

    bool getStby();

    cv::Vec3f getRing();

    void setRing(cv::Vec3f in);
};


#endif //RSRTRACKING_DEVIATIONPOSITION_H
