﻿#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/ml/ml.hpp"
#include <opencv2/videoio.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#define PI 3.14159265358979323846
#define POINT_X 0//!!!这四个常量是标准的位置的xy坐标和两条直线在标准位置的倾斜
#define POINT_Y 0
#define THETA_A 0
#define THETA_B 90
#define mindiffer 15

using namespace cv;
using namespace std;

int g_ir2 = 1;

bool x_cmp(Vec6f a, Vec6f b) {
    return -1 * (a[2] / a[0]) < -1 * (b[2] / b[0]);
}

bool y_cmp(Vec6f a, Vec6f b) {
    return -1 * (a[2] / a[1]) < -1 * (b[2] / b[1]);
}

bool theta_cmp(Vec6f a, Vec6f b) {
    return a[3] < b[3];
}

bool size_cmp(vector<Vec6f> a, vector<Vec6f> b) {
    return a.size() < b.size();
}

void k_0_drawAverlines(vector<Vec6f> oneLine, Mat &singleLine, vector<Vec6f> &linesAver) {
    float a = 0;
    float b = 0;
    float c = 0;
    float angle = 0;
    float x = 0;//其实x轴截距不存在
    float y = 0;

    for (int i = 0; i < oneLine.size(); i++) {
        a += oneLine[i][0];
        b += oneLine[i][1];
        c += oneLine[i][2];
    }
    a = a / oneLine.size();
    b = b / oneLine.size();
    c = c / oneLine.size();
    y = -1 * (c / b);

    Vec6f temp(a, b, c, angle, x, y);
    linesAver.push_back(temp);

    Point p1;
    Point p2;
    p1.x = 0;
    p1.y = y;
    p2.x = singleLine.cols;
    p2.y = y;
    line(singleLine, p1, p2, Scalar(255), 1, LINE_AA);
}

Vec6f averLines(vector<Vec6f> oneLine) {
    float a = 0;
    float b = 0;
    float c = 0;
    float angle = 0;
    float x = 0;
    float y = 0;

    for (int i = 0; i < oneLine.size(); i++) {
        a += oneLine[i][0];
        b += oneLine[i][1];
        c += oneLine[i][2];
    }
    a = a / oneLine.size();
    b = b / oneLine.size();
    c = c / oneLine.size();
    if (a == 0) {
        y = -1 * (c / b);

        return Vec6f(a, b, c, angle, x, y);
    } else if (b == 0) {
        angle = 90;
        x = -1 * (c / a);
        y = 0;//not exist

        return Vec6f(a, b, c, angle, x, y);
    } else {
        angle = atan(-a / b) * 180 / PI;
        x = -1 * (c / a);
        y = -1 * (c / b);

        return Vec6f(a, b, c, angle, x, y);
    }
}

void drawLine(Vec6f oLine, Mat &singleLine) {
    float a = oLine[0];
    float b = oLine[1];
    float c = oLine[2];
    float angle = oLine[3];
    float x = oLine[4];
    float y = oLine[5];
    Point p1;
    Point p2;

    if (a == 0) {
        p1.x = 0;
        p1.y = y;
        p2.x = singleLine.cols;
        p2.y = y;
        line(singleLine, p1, p2, Scalar(255), 1, LINE_AA);
    } else {
        p1.y = 0;
        p1.x = x;

        p2.y = singleLine.rows;
        p2.x = -(b * p2.y + c) / a;
        line(singleLine, p1, p2, Scalar(255), 1, LINE_AA);
    }

}

Vec6f meaningLineAngle(vector<Vec6f> linesCount, float dAngle) {
    Vec6f allM = averLines(linesCount);
    vector<Vec6f> meaningCount;
    for (int i = 0; i < linesCount.size(); ++i) {
        if (abs(linesCount[i][3] - allM[3]) < dAngle)
            meaningCount.push_back(linesCount[i]);
    }
    return averLines(meaningCount);
}

vector<vector<Vec6f>> divideAngleLines(vector<Vec6f> linesCount, float DValue, float cValue) {
    sort(linesCount.begin(), linesCount.end(), theta_cmp);
    vector<vector<Vec6f>> allLines;
    vector<Vec6f> oneLine;//本来是同一条线的集合
    size_t j = 0;
    for (size_t i = 0; i < linesCount.size(); i++) {
        //cout << DValue << ":" << linesCount[i][3] << endl;
        if (abs(linesCount[i][3] - linesCount[j][3]) > DValue) {
            allLines.push_back(oneLine);
            //k_0_drawAverlines(k_0_oneLine, singleLine,linesAver);
            oneLine.clear();
            j = i;
            oneLine.push_back(linesCount[i]);
        } else if (i == linesCount.size() - 1) {
            oneLine.push_back(linesCount[i]);
            allLines.push_back(oneLine);
        } else {
            oneLine.push_back(linesCount[i]);
        }
    }
    //任意相邻的两块应该合成一块
    vector<vector<Vec6f>> res;
    vector<vector<int>> closeZone;
    res.push_back(allLines[0]);
    for (int i = 0; i < allLines.size() - 1; i++) {
        oneLine.clear();
        int i1 = i + 1;
        float d = abs(allLines[i].back()[3] - allLines[i1].front()[3]);
        if (d > 90)
            d = 180 - d;
        if (d < cValue) {
            auto oneV = res.end() - 1;
            (*oneV).insert((*oneV).end(), allLines[i1].begin(), allLines[i1].end());
            sort((*oneV).begin(), (*oneV).end(), theta_cmp);
        } else
            res.push_back(allLines[i1]);
    }
    float d = abs(res.back().back()[3] - res.front().front()[3]);
    if (d > 90)
        d = 180 - d;
    if (d < cValue) {
        auto oneV = res.end() - 1;
        (*oneV).insert((*oneV).end(), res[0].begin(), res[0].end());
        sort((*oneV).begin(), (*oneV).end(), theta_cmp);
        res.erase(res.begin());
    }
    return res;
}

vector<vector<Vec6f>> divideLines(vector<Vec6f> linesCount, int dataNmber, float DValue) {
    vector<vector<Vec6f>> allLines;
    vector<Vec6f> oneLine;//本来是同一条线的集合
    size_t j = 0;
    for (size_t i = 0; i < linesCount.size(); i++) {
        if (abs(linesCount[i][dataNmber] - linesCount[j][dataNmber]) > DValue) {
            allLines.push_back(oneLine);
            //k_0_drawAverlines(k_0_oneLine, singleLine,linesAver);
            oneLine.clear();
            j = i;
            oneLine.push_back(linesCount[i]);
        } else if (i == linesCount.size() - 1) {
            oneLine.push_back(linesCount[i]);
            allLines.push_back(oneLine);
        } else {
            oneLine.push_back(linesCount[i]);
        }
    }
    return allLines;
}

float CalculateAngle(float a, float b) {
    if (b == 0) {
        return 90;
    } else {
        return atan(-a / b) * 180 / PI;
    }
}

class LineFinder {
private:
    Mat img; // original image
    vector<Vec4i> lines;
    double deltaRho;
    double deltaTheta;
    int minVote;
    double minLength; // min length for a line
    double maxGap; // max allowed gap along the line

public:
    // Default accumulator resolution is 1 pixel by 1 degree
    // no gap, no mimimum length
    LineFinder() : deltaRho(1),
                   deltaTheta(PI / 180),
                   minVote(10),
                   minLength(30),
                   maxGap(20) {}

    // Set the resolution of the accumulator
    void setAccResolution(double dRho, double dTheta) {
        deltaRho = dRho;
        deltaTheta = dTheta;
    }

    // Set the minimum number of votes
    void setMinVote(int minv) {
        minVote = minv;
    }

    // Set line length and gap
    void setLineLengthAndGap(double length, double gap) {
        minLength = length;
        maxGap = gap;
    }

    // Apply probabilistic Hough Transform
    vector<Vec4i> findLines(Mat &binary, Mat &dstImage) {
        lines.clear();
        HoughLinesP(binary, lines, deltaRho, deltaTheta, 5, minLength, maxGap);
        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];
            line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
        }
        imshow("manyLines", dstImage);
        return lines;
    }
};

//............main................
int main() {
    Mat computerImage = imread("/home/peng/下载/38_31_test/03.bmp");
    Mat shrink = computerImage.clone();
    //    resize(computerImage, shrink, Size(computerImage.cols, computerImage.rows), 0, 0, INTER_LINEAR);

    //梯形矫正
    Mat perspectiveImage;
    perspectiveImage = Scalar::all(0);

    //    //人工选角点
    //    bool showCrosshair = false;
    //    bool fromCenter = false;
    //    Rect2d r = selectROI("select", shrink, fromCenter, showCrosshair);
    //    imshow("shrink", shrink);
    //    cout << "左上角横坐标：" << r.x <<"\t" << "左上角纵坐标：" << r.y << endl;
    //    waitKey(0);

    Point2f srcTri[4] = {Point2f(), Point2f(), Point2f(), Point2f()};
    Point2f dstTri[4];
    //左上
    srcTri[0].x = 325;
    srcTri[0].y = 43;
    //右上
    srcTri[1].x = 611;
    srcTri[1].y = 43;
    //左下
    srcTri[2].x = 315;
    srcTri[2].y = 269;
    //右下
    srcTri[3].x = 658;
    srcTri[3].y = 268;

    dstTri[0] = Point2f(0, 0);
    dstTri[1] = Point2f(shrink.cols / 3, 0);
    dstTri[2] = Point2f(0, shrink.cols * 12.6 / 27.6);
    dstTri[3] = Point2f(shrink.cols / 3, shrink.cols * 12.6 / 27.6);

    circle(shrink, srcTri[0], g_ir2, Scalar(0, 0, 255), -1, 8, 0);
    circle(shrink, srcTri[1], g_ir2, Scalar(0, 255, 0), -1, 8, 0);
    circle(shrink, srcTri[2], g_ir2, Scalar(255, 0, 0), -1, 8, 0);
    circle(shrink, srcTri[3], g_ir2, Scalar(255, 255, 0), -1, 8, 0);
    //Mat perspectiveImage = shrink.clone();
    Mat transform = getPerspectiveTransform(dstTri, srcTri);
    //perspective.
    warpPerspective(shrink, perspectiveImage, transform, Size(shrink.cols * 1.5, shrink.rows * 3),
                    INTER_LINEAR | WARP_INVERSE_MAP);
    Mat perspSmall = perspectiveImage.clone();
    resize(perspectiveImage, perspSmall, Size(perspectiveImage.cols / 2, perspectiveImage.rows / 2), 0, 0,
           INTER_LINEAR);
    //    imshow("perspSmall", perspSmall);

    Mat binaryImage;
    Mat Gray;
    vector<Mat> hsvSplit;
    split(perspSmall, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, perspSmall);

    cvtColor(perspSmall, Gray, CV_BGR2GRAY);
    GaussianBlur(Gray, Gray, Size(3, 3), 2, 2);
    threshold(Gray, binaryImage, 140, 255, CV_THRESH_BINARY);
    //    imshow("binaryImage ", binaryImage);

    const int elesize = 20;
    Mat element = getStructuringElement(MORPH_RECT, Size(elesize, elesize));
    morphologyEx(binaryImage, binaryImage, MORPH_CLOSE, element);
    //    imshow("binaryImage", binaryImage);

    //求轮廓
    vector<vector<Point>> contour;
    vector<Vec4i> hierarchy;
    //    double minarea = 100.0;
    Mat binaryImage_Copy = binaryImage.clone();
    findContours(binaryImage_Copy, contour, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    Mat imageContours = Mat::zeros(binaryImage_Copy.size(), CV_8UC1);
    for (size_t i = 0; i < contour.size(); i++) {
        drawContours(imageContours, contour, i, Scalar(255), 1, 8, hierarchy);
    }
    imshow("Contours Image", imageContours); //轮廓

    Mat imageLines = Mat::zeros(imageContours.size(), CV_8UC1);
    LineFinder finder;
    vector<Vec4i> lines = finder.findLines(imageContours, imageLines);
    cout << "sum:" << lines.size() << endl;
    //    imshow("imageLines", imageLines);
    //    vector<Vec4i> lines = finder.findLines(imageContours);
    //    for(size_t i = 0; i < lines.size(); i++)
    //    {
    //        Vec4i l = lines[i];
    //        line(imageContours, Point(l[0], l[1]), Point(l[2],l[3]),Scalar(0,255,0), 1, LINE_AA);
    //    }
    //    imshow("[imageContours]",imageContours);
    vector<Vec6f> k_0_linesCount;
    vector<Vec6f> k_small_linesCount_no_0;//ax+by+c=0   第一位a，第二位是b，第三位是c，第四位是角度
    vector<Vec6f> linesCount;//ax+by+c=0   第一位a，第二位是b，第三位是c，第四位是角度
    vector<Vec6f> linesAver;

    for (size_t i = 0; i < lines.size(); i++) {
        float a, b, c, x, y;
        float angle;
        a = lines[i][1] - lines[i][3];//lines起点和终点的x,y坐标x1,y1,x2,y2->0,1,2,3
        b = lines[i][2] - lines[i][0];
        c = -1 * (a * lines[i][0] + b * lines[i][1]);
        angle = CalculateAngle(a, b);
        x = -1 * (c / a);
        y = -1 * (c / b);
//        if( b == 0 )//斜率不存在
//        {
//            Vec6f temp(a, b, c, angle, x, y);
//            k_n_linesCount.push_back(temp);
//        }
        if (a == 0)//斜率为0
        {
            x = 0;//not exist
            Vec6f temp(a, b, c, angle, x, y);
            linesCount.push_back(temp);
        } else if (angle > 45 || angle < -45) {
            if (angle == 90) {
                y = 0;//not exist
                Vec6f temp(a, b, c, angle, x, y);
                linesCount.push_back(temp);
            } else {
                Vec6f temp(a, b, c, angle, x, y);
                linesCount.push_back(temp);
            }
        } else {
            Vec6f temp(a, b, c, angle, x, y);
            linesCount.push_back(temp);
        }
    }
    Mat singleLine = perspSmall.clone();
    //45 < k <= 90 || -90 <= k < -45
    vector<vector<Vec6f>> allLines = divideAngleLines(linesCount, 6, 2);
    sort(allLines.begin(), allLines.end(), size_cmp);
    int maxL = 4;
    int maxLA = 2;
    maxLA = static_cast<int>(maxLA > allLines.size() ? allLines.size() : maxLA);
    for (auto il = allLines.rbegin(); il < allLines.rbegin() + maxLA; il++) {
        vector<Vec6f> largeLine = *il;
        cout << "l:" << largeLine.size() << endl;
        //divide again by x or y
        vector<vector<Vec6f>> oLines;
        cout << "fuck" << largeLine.back()[3] << endl;
        if (largeLine.back()[3] < 45 && largeLine.front()[3] > -45) {
            sort(largeLine.begin(), largeLine.end(), y_cmp);
            oLines = divideLines(largeLine, 5, 20);
        } else {
            sort(largeLine.begin(), largeLine.end(), x_cmp);
            oLines = divideLines(largeLine, 4, 20);
        }
        sort(oLines.begin(), oLines.end(), size_cmp);
        reverse(oLines.begin(), oLines.end());
        for (int j = 0; (j < maxL) && (j < oLines.size()); j++) {
            linesAver.push_back(averLines(oLines[j]));
        }
    }

    //画平均线
    for (size_t i = 0; i < linesAver.size(); i++) {
        drawLine(linesAver[i], singleLine);
        for (size_t j = i + 1; j < linesAver.size(); j++) {
            float x = 0;
            float y = 0;
            x = (linesAver[i][1] * linesAver[j][2] - linesAver[j][1] * linesAver[i][2]) /
                ((linesAver[i][0] * linesAver[j][1] - linesAver[j][0] * linesAver[i][1]));
            y = (linesAver[j][0] * linesAver[i][2] - linesAver[i][0] * linesAver[j][2]) /
                ((linesAver[i][0] * linesAver[j][1] - linesAver[j][0] * linesAver[i][1]));
            Point p;
            p.x = x;
            p.y = y;
            if (isnan(y) || isnan(x) || (x > singleLine.cols) || (y > singleLine.rows))//去掉超出图像的数据
            {
                continue;
            }
            circle(singleLine, p, 3, Scalar(0, 0, 255));
            cout << "x is:" << x << "y is:" << y << endl;
        }
    }
    imshow("singleLine", singleLine);
    waitKey(0);
    return 0;
}