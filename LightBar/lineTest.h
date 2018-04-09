//
// Created by peng on 18-4-8.
//

#ifndef LIGHTBAR_LINETEST_H
#define LIGHTBAR_LINETEST_H

#include "linesOption.h"
#include "lineInfo.h"

class LineTest {
private:
    bool is_nan(double dVal);

    void GetDiffImage(cv::Mat src1, cv::Mat src2, cv::Mat dst, int nThre);

    static bool comp(const cv::Vec4i &a, const cv::Vec4i &b);

    float RadianCalculate(float rleft, float rright);

    float rad(float ang);

    float AngleCalculate(float rleft, float rright);

    std::vector<cv::Vec4i> findCorner(cv::Mat dst, cv::Mat src);

    void analyse(LinesOption all_line, LinesOption left_line, LinesOption right_line, LinesOption left_line2,
                 LinesOption right_line2, std::vector<cv::Vec4i> lines);

    void drawDetectLines(cv::Mat &image, const std::vector<cv::Vec4i> &lines, cv::Scalar &color);

public:
    LinesOption all_line;
    LinesOption left_line;
    LinesOption right_line;
    LinesOption left2_line;
    LinesOption right2_line;
    float info_value[3]{0};
public:
    int watch(cv::Mat res);

    //define operator()
    int operator()(LineInfo &info);
};


#endif //LIGHTBAR_LINETEST_H
