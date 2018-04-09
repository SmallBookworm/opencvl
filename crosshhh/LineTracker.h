//
// Created by peng on 18-3-14.
//

#ifndef LINETRACKER_H
#define LINETRACKER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "LineFinder.h"

#define standard_real_x -1210   //第一次交接位置 -620,8450
#define standard_real_y 6355    //第一次交接位置
#define mindiffer 15
#define Maxdist 130
#define x_move 1500
#define y_move 1050
#define paper_height 100
#define paper_weight 150
#define standard_picture_x 718
#define standard_picture_y 409
#define standard_angle 0.2324

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

    float distance(cv::Point2f pa, cv::Point2f pb);

    cv::Vec6f averLines(std::vector<cv::Vec6f> oneLine);

    float CalculateAngle(float a, float b);

    void drawLine(cv::Vec6f oLine, cv::Mat &singleLine);

    std::vector<std::vector<cv::Vec6f>> divideAngleLines(std::vector<cv::Vec6f> linesCount, float DValue, float cValue);

    std::vector<std::vector<cv::Vec6f>> divideLines(std::vector<cv::Vec6f> linesCount, int dataNmber, float DValue);
//-1 fail 1 success
    int watch(cv::Mat &computerImage, cv::Point2f *point);
};


#endif //CROSS_LINETRACKER_H
