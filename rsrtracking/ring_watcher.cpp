#include "ring_watcher.h"

using namespace cv;
using namespace std;

Vec3b RingWatcher::getColor(cv::Mat &res) {
    imshow("getColor", res);
    Rect rect = selectROI("getColor", res);
    Vec3b color = res.at<Vec3b>(rect.tl());
    return color;
}

int RingWatcher::getRing(cv::Mat &result) {
    Mat_<Vec3b>::iterator it = result.begin<Vec3b>();
    Mat_<Vec3b>::iterator itend = result.end<Vec3b>();
    for (; it != itend; ++it) {
        if ((*it)[0]<180&&(*it)[0]>0&&(*it)[1]>0&&(*it)[0] > (*it)[1] && (*it)[1] > (*it)[2])
            (*it)=Vec3b(0,255,255);
    }
}