//
// Created by peng on 18-3-14.
//

#include "LineFinder.h"

using namespace std;
using namespace cv;

vector<cv::Vec4i> LineFinder::findLines(cv::Mat &binary, cv::Mat &dstImage) {
    lines.clear();
    HoughLinesP(binary, lines, deltaRho, deltaTheta, 5, minLength, maxGap);
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
    }
//        imshow("manyLines", dstImage);
//        waitKey(50);
    return lines;
}