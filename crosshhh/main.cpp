#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "LineTracker.h"

//#define standard_x X
//#define standard_y Y
//#define standard_angle ANGLE

using namespace cv;
using namespace std;



//............main................
int main()
{
    Mat computerImage = imread("/home/peng/下载/38_31_test/02.bmp");
    LineTracker lineTracker;
    Point2f point;
    lineTracker.watch(computerImage,&point);
    waitKey(0);
    return 0;
}
