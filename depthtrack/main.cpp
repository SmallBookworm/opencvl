#include <iostream>
#include <thread>
#include <future>
#include "ball_tracker.h"

using namespace std;
using namespace cv;

int main() {
    promise<int> prom;
    future<int> fut = prom.get_future();
    Tracker tracker;
    thread thread1(tracker, ref(fut));
    thread1.detach();
    //tracker.test();
    getchar();
    prom.set_value(10);
    return 0;
}