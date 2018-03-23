//
// Created by peng on 18-3-19.
//

#include "ring_watcher.h"

using namespace std;
using namespace cv;

std::vector<RotatedRect> RingWatcher::getRotatedRect(std::vector<std::vector<Point>> region_contours) {
    std::vector<RotatedRect> objects;
    for (size_t i = 0; i != region_contours.size(); ++i) {
        RotatedRect rect = minAreaRect(region_contours[i]);
        objects.push_back(rect);
    }
    return objects;
}

std::vector<cv::Rect> RingWatcher::getRect(std::vector<std::vector<cv::Point>>) {

}

cv::RotatedRect RingWatcher::getRingPole(std::vector<cv::Point> contours) {
    vector<cv::Point> pole;
    RotatedRect rect = minAreaRect(contours);
    float minY = 1000, maxY = 0;
    Point2f points[4];
    rect.points(points);
    for (auto &point:points) {
        if (point.y < minY)
            minY = point.y;
        else if (point.y > maxY)
            maxY = point.y;
    }
    float min = minY + (maxY - minY) / 3, max = maxY - (maxY - minY) / 3;
    for (auto &pt:contours) {
        if (pt.y > min && pt.y < max)
            pole.push_back(pt);
    }
    return minAreaRect(pole);
}

int RingWatcher::getRing(std::vector<std::vector<cv::Point>> contours, cv::Mat &result) {
    vector<RotatedRect> rects = this->getRotatedRect(contours);
    for (int i = 0; i < rects.size(); ++i) {

        if (rects[i].size.height < 150)
            continue;
        if (rects[i].size.width > rects[i].size.height / 2)
            continue;
        if (rects[i].center.x < 200 || rects[i].center.x > 300)
            continue;
        if (rects[i].center.y < result.rows / 2)
            continue;

        this->ring[0] = 1;
        RotatedRect ringPole = this->getRingPole(contours[i]);
        //test
        Point2f points[4];
        ringPole.points(points);
        for (int j = 0; j < 4; ++j) {
            line(result, points[j], points[(j + 1) % 4], Scalar(0, 255, rects[i].center.y), 1, CV_AA);
        }
        break;
    }
    return 1;
}

cv::Vec3f RingWatcher::getPoleRange(cv::Mat &grayFrame) {
    float max = 0.75;
    float min = 0.4;
    float colsS[grayFrame.cols]{0};
    vector<int[2]> valueRange[grayFrame.cols];

    for (int i = 0; i < grayFrame.cols; ++i) {
        for (int j = 0; j < grayFrame.rows; ++j) {
            if (grayFrame.at<uchar>(j, i) > 0)
                colsS[i] += 1.0;
        }
        colsS[i] /= grayFrame.rows;
    }
    int minC = 1;
    int maxC = grayFrame.cols / 40;
    int count = 0;
    for (int k = 0; k < grayFrame.cols; ++k) {
        if (colsS[k] > min && colsS[k] < max)
            count++;
        else {

        }
    }
}