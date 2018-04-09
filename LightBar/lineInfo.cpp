//
// Created by peng on 18-4-8.
//

#include "lineInfo.h"

using namespace std;

int LineInfo::get(float *res) {
    lock_guard<mutex> l(line_mutex);
    if (used) {
        return -1;
    } else {
        res[0] = dModule;
        res[1] = argument;
        res[2] = relativeAngle;
        used = true;
        return 1;
    }
}

void LineInfo::set(float *value) {
    lock_guard<mutex> l(line_mutex);
    dModule = value[0];
    argument = value[1];
    relativeAngle = value[2];
    used = false;
}