//
// Created by peng on 18-3-1.
//

#ifndef DEPTHTRACK_BALL_TRACKER_H
#define DEPTHTRACK_BALL_TRACKER_H

#include<thread>
#include <iostream>
#include <zconf.h>
#include <future>

// move-constructible function object (i.e., an object whose class defines operator(), including closures and function objects).
class Tracker {
public:
    //define operator()
    void  operator()(std::future<int>& fut);
private:
    void outp(int i);
};

#endif //DEPTHTRACK_BALL_TRACKER_H
