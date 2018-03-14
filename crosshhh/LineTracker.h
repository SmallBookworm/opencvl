//
// Created by peng on 18-3-14.
//

#ifndef LINETRACKER_H
#define LINETRACKER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "LineFinder.h"

#define MaxtransLinesgap 150
#define x_move 600
#define y_move 80

class LineTracker {
private:
    LineFinder finder;
public:
    explicit LineTracker() {
        // Set probabilistic Hough parameters
        this->finder.setLineLengthAndGap(100, 20);
        this->finder.setMinVote(10);
    }

    static bool x_cmp(cv::Vec6f a, cv::Vec6f b) {
        return -1 * (a[2] / a[0]) < -1 * (b[2] / b[0]);
    }

    static bool y_cmp(cv::Vec6f a, cv::Vec6f b) {
        return -1 * (a[2] / a[1]) < -1 * (b[2] / b[1]);
    }

    static bool theta_cmp(cv::Vec6f a, cv::Vec6f b) {
        return a[3] < b[3];
    }

    static bool size_cmp(std::vector<cv::Vec6f> a, std::vector<cv::Vec6f> b) {
        return a.size() < b.size();
    }

    static bool px_cmp(cv::Vec2f a, cv::Vec2f b) {
        return a[0] < b[0];
    }

    float distance(cv::Vec2f pa, cv::Vec2f pb) {
        return sqrt((pa[0] - pb[0]) * (pa[0] - pb[0]) + (pa[1] - pb[1]) * (pa[1] - pb[1]));
    }

    cv::Vec6f averLines(std::vector<cv::Vec6f> oneLine);

    float CalculateAngle(float a, float b);

    void drawLine(cv::Vec6f oLine, cv::Mat &singleLine);

    std::vector<std::vector<cv::Vec6f>> divideAngleLines(std::vector<cv::Vec6f> linesCount, float DValue, float cValue);

    std::vector<std::vector<cv::Vec6f>> divideLines(std::vector<cv::Vec6f> linesCount, int dataNmber, float DValue);

    int watch(cv::Mat &computerImage, cv::Point2f *point);
};


#endif //CROSS_LINETRACKER_H
