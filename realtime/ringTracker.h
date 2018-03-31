//
// Created by peng on 18-3-28.
//

#ifndef REALTIME_RINGTRACKER_H
#define REALTIME_RINGTRACKER_H

#include <opencv2/opencv.hpp>
#include <zconf.h>
#include "coordinate.h"

class RingTracker {
public:
    void operator()(Coordinate &coordinate);
};


#endif //REALTIME_RINGTRACKER_H
