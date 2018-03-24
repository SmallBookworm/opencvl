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

cv::Vec3f RingWatcher::getPoleRange(Mat grayFrame) {
    float max = 0.75;
    float min = 0.4;
    float colsS[grayFrame.cols]{0};
    vector<vector<Vec2i>> valueRange;

    for (int i = 0; i < grayFrame.cols; ++i) {
        vector<Vec2i> range;
        int start = -1;
        int end = -1;
        for (int j = 0; j < grayFrame.rows; ++j) {
            if (grayFrame.at<uchar>(j, i) > 0) {
                colsS[i] += 1.0;
                if (start < 0)
                    start = j;
                end = j;
            } else if (start > 0) {
                range.emplace_back(start, end);
                start = end = -1;
            }
        }
        colsS[i] /= grayFrame.rows;
        valueRange.push_back(range);
    }
    int minC = 1;
    int maxC = grayFrame.cols / 40;
    int count = 0;
    int k;
    for (k = 0; k < grayFrame.cols; ++k) {
        if (colsS[k] > min && colsS[k] < max)
            count++;
        else if (count > minC && count < maxC) {
            break;
        } else
            count = 0;
    }
    Vec3f res;
    if (count == 0) {
        res[0] = -1;
        return res;
    }
    int endY = 0;
    int endX = 0;
    int noiseDis = grayFrame.rows / 10;
    for (int l = 1; l <= count; ++l) {
        endX += k - l;
        vector<Vec2i> col = valueRange[k - l];
        int minR = -1;
        for (auto i = col.rbegin(); i != col.rend(); i++) {
            if (((*i)[1] - (*i)[0]) < noiseDis)
                continue;
            if ((minR - (*i)[0]) > noiseDis)
                break;
            minR = (*i)[0];
        }
        if (minR > 0) {
            cout << minR << endl;
            endY += minR;
        }

    }
    res[0] = endX / count;
    res[1] = endY / count;
    return res;
}

int RingWatcher::getThresholdRing(cv::Mat &result) {
    int x0 = result.cols / 4;
    int y0 = 0;
    Rect roi(x0, y0, result.cols / 2, result.rows);
    Vec3f res = this->getPoleRange(Mat(result, roi));
    if (res[0] < 0)
        return -1;
    circle(result, Point(x0 + res[0], y0 + res[1]), 10, Scalar(255));
}