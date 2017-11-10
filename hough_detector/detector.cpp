//
// Created by peng on 17-10-22.
//

#include "detector.h"

using namespace cv;

std::vector<Vec3f> Detector::getCircles(Mat &srcImage) {
    //中值滤波
    medianBlur(srcImage, srcImage, 3);
    Mat hsvImage;
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);
    //imshow("hsvImage",hsvImage);
    //s饱和度取0-60包含了其它淡色
    Mat whiteTempMat;
    inRange(hsvImage, Scalar(0, 0, 200), Scalar(180, 60, 255), whiteTempMat);
    //imshow("white",whiteTempMat);

    //高斯平滑
    GaussianBlur(whiteTempMat, whiteTempMat, Size(9, 9), 2, 2);

    std::vector<Vec3f> circles;
    //change arguments
    HoughCircles(whiteTempMat, circles, CV_HOUGH_GRADIENT, 1, whiteTempMat.rows / 6, 100, 40, whiteTempMat.rows / 40,
                 whiteTempMat.rows / 4);
    return circles;
}

const Mat &Detector::getSrcImage() const {
    return srcImage;
}

void Detector::setSrcImage(const Mat &srcImage) {
    Detector::srcImage = srcImage;
}

std::vector<int> Detector::getScore(std::vector<Vec3f> circles) {
    std::vector<int> scores;
    for (auto &circle:circles) {
        Mat decImage(srcImage.rows, srcImage.cols, CV_8UC3);
        auto width = static_cast<int>(circle[2] * _para.rWidth);
        auto height = static_cast<int>(circle[2] * _para.rHeight);
        srcImage(Rect(round(circle[0] - width / 2), round(circle[1] - height / 2), width, height)).copyTo(decImage);
        imshow("12", decImage);

    }
    return scores;
}