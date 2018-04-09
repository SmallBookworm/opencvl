//
// Created by peng on 18-4-8.
//

#ifndef LIGHTBAR_LINEINFO_H
#define LIGHTBAR_LINEINFO_H

#include <future>
#include <mutex>

class LineInfo {
private:
    float dModule;
    float argument;
    float relativeAngle;
    bool used;
    std::mutex line_mutex;
public:
    //write only before read
    std::future<int> *fut;

    LineInfo() {
        used = true;
    }

public:
    void set(float value[3]);

//-1 used,1 success
    int get(float *res);
};


#endif //LIGHTBAR_LINEINFO_H
