//
// Created by peng on 18-4-8.
//

#ifndef LIGHTBAR_LINESOPTION_H
#define LIGHTBAR_LINESOPTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

const int WIDTH = 1280;
const int HEIGHT = 720;
const int SD = 577;
const int SPIX_LIGHT_HEIGHT = 272;//灯条外接矩形的高度（像素）
const int SREAL_HEIGHT = 150;	//mm
const int SREAL_WIDTH = 190;	//mm
const float SPIX_LIGHT_WIDTH = SPIX_LIGHT_HEIGHT*((float)SREAL_WIDTH / (float)SREAL_HEIGHT);//灯条外接矩形的宽度（像素）
const float SINIT_ANGLE = 0.00;
const float SLEFTTOCENTER = -156.7;//$$$$$$$$$$$$$$$$$$
const float SRIGHTTOCENTER = 189;//$$$$$$$$$$$$$$$$$$
const int AVG = 2;
const int contain = 3;


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
