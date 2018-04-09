//
// Created by peng on 18-4-8.
//

#ifndef LIGHTBAR_LINESOPTION_H
#define LIGHTBAR_LINESOPTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

#define WIDTH 1920
#define HEIGHT 1080
#define sD 697
#define spix_light_height 238//灯条外接矩形的高度（像素）
#define spix_light_width 272//灯条外接矩形的宽度（像素）
#define sinit_angle -0.902912
#define sinit_ld 5.784
#define sinit_rd 2.904
#define sreal_height 158    //mm
#define sreal_width 182    //mm
#define AVG 6


class LinesOption {
public:
    cv::Point2f upPoint(int i, std::vector<cv::Vec4i> &a);

    cv::Point2f downPoint(int i, std::vector<cv::Vec4i> &a);

    cv::Point2f centerPoint(int i, std::vector<cv::Vec4i> &a);

    int pixheight(int i, std::vector<cv::Vec4i> &a);

    float realdist(int i, std::vector<cv::Vec4i> &a);

    float surposepixWidth(int i, std::vector<cv::Vec4i> &a);

};


#endif //LIGHTBAR_LINESOPTION_H
