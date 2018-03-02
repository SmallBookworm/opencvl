//
// Created by peng on 18-3-1.
//


#include "ball_tracker.h"

using namespace std;

void Tracker::outp(int i) {
    cout << i << endl;
}
// move-constructible function object (i.e., an object whose class defines operator(), including closures and function objects).
void Tracker::operator()(std::future<int>& fut) {
    future_status status;
    int i = 0;
    do {
        this->outp(++i);
        sleep(1);
        status = fut.wait_for(chrono::milliseconds(10));
    } while (status != future_status::ready);
}