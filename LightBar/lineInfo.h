//
// Created by peng on 18-4-8.
//

#ifndef LIGHTBAR_LINEINFO_H
#define LIGHTBAR_LINEINFO_H

#include <mutex>

class LineInfo {
private:
    float dModule;
    float argument;
    float relativeAngle;
    bool used;
    std::mutex line_mutex;
private:
    bool stop;
    std::mutex stop_mutex;
public:
    LineInfo() {
        used= true;
        stop = false;
    }

public:
    void init();

    void set(float value[3]);

//-1 used,1 success
    int get(float *res);

    void setStop(bool f);

    bool getStop();
};


#endif //LIGHTBAR_LINEINFO_H
