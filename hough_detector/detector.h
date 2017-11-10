//
// Created by peng on 17-10-22.
//

#ifndef INC_1_DETECTOR_H
#define INC_1_DETECTOR_H

#endif //INC_1_DETECTOR_H

#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


struct ArmorParam {
    double min_radius;              //最小圆半径和图像高度比
    double max_radius;          //
    double min_dist;            //不同圆之间的距离...比

    double rHeight;               //装甲板和圆的比例
    double rWidth;
    uchar enemy_color;                 // 0 for red, otherwise blue

    ArmorParam() {
        min_radius = 1 / 40;
        max_radius = 1 / 4;
        min_dist = 1 / 6;
        rHeight = 4;
        rWidth = 4;
        enemy_color = 0;
    }
};

class Detector {
public:
    explicit Detector(cv::Mat srcImage,const ArmorParam &para = ArmorParam()) {
        srcImage=srcImage;
        _para = para;

    }

    std::vector<cv::Vec3f> getCircles(cv::Mat &srcImage);

    std::vector<int> getScore(std::vector<cv::Vec3f> circles);

private:
    ArmorParam _para;
    cv::Mat srcImage;
public:
    const cv::Mat &getSrcImage() const;

    void setSrcImage(const cv::Mat &srcImage);
};