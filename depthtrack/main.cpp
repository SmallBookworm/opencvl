#include <iostream>
#include <thread>
#include <future>
#include "ball_tracker.h"

using namespace std;


int main() {
    getchar();
    promise<int> prom;
    future<int> fut = prom.get_future();
    Tracker tracker;
    thread thread1(tracker, ref(fut));
    thread1.detach();
    getchar();
    prom.set_value(10);
    getchar();
    return 0;
}