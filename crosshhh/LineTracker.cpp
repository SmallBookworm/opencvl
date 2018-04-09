//
// Created by peng on 18-3-14.
//

#include "LineTracker.h"

using namespace std;
using namespace cv;

float LineTracker::distance(cv::Point2f pa, cv::Point2f pb) {
    return sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
}

Vec6f LineTracker::averLines(vector<Vec6f> oneLine) {
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
        angle = atan(-a / b) * 180 / M_PI;
        x = -1 * (c / a);
        y = -1 * (c / b);

        return Vec6f(a, b, c, angle, x, y);
    }
}

void LineTracker::drawLine(Vec6f oLine, Mat &singleLine) {
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
        line(singleLine, p1, p2, Scalar(255), 2, LINE_AA);
    } else {
        p1.y = 0;
        p1.x = x;

        p2.y = singleLine.rows;
        p2.x = -(b * p2.y + c) / a;
        line(singleLine, p1, p2, Scalar(255), 2, LINE_AA);
    }
}

vector<vector<Vec6f>> LineTracker::divideAngleLines(vector<Vec6f> linesCount, float DValue, float cValue) {
    sort(linesCount.begin(), linesCount.end(), theta_cmp);
    vector<vector<Vec6f>> allLines;
    vector<Vec6f> oneLine;//本来是同一条线的集合
    size_t j = 0;
    for (size_t i = 0; i < linesCount.size(); i++) {
        if (abs(linesCount[i][3] - linesCount[j][3]) > DValue) {
            allLines.push_back(oneLine);
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

vector<vector<Vec6f>> LineTracker::divideLines(vector<Vec6f> linesCount, int dataNmber, float DValue) {
    vector<vector<Vec6f>> allLines;
    vector<Vec6f> oneLine;//本来是同一条线的集合
    size_t j = 0;
    for (size_t i = 0; i < linesCount.size(); i++) {
        if (abs(linesCount[i][dataNmber] - linesCount[j][dataNmber]) > DValue) {
            allLines.push_back(oneLine);
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

float LineTracker::CalculateAngle(float a, float b) {
    if (b == 0) {
        return 90;
    } else {
        return atan(-a / b) * 180 / M_PI;
    }
}

int LineTracker::watch(cv::Mat &computerImage, cv::Point2f *point) {
    //梯形矫正
    Mat perspectiveImage;
    perspectiveImage = Scalar::all(0);

    Point2f srcTri[4] = {Point2f(), Point2f(), Point2f(), Point2f()};
    Point2f dstTri[4];
    //左上
    srcTri[0].x = 456;
    srcTri[0].y = 283;
    //右上
    srcTri[1].x = 796;
    srcTri[1].y = 286;
    //左下
    srcTri[2].x = 437;
    srcTri[2].y = 507;
    //右下
    srcTri[3].x = 804;
    srcTri[3].y = 511;

    dstTri[0] = Point2f(x_move, y_move);
    dstTri[1] = Point2f(x_move + computerImage.cols, y_move);
    dstTri[2] = Point2f(x_move, y_move + computerImage.cols * paper_height / paper_weight);
    dstTri[3] = Point2f(x_move + computerImage.cols, y_move + computerImage.cols * paper_height / paper_weight);
    Mat transform = getPerspectiveTransform(dstTri, srcTri);
    //perspective.
    warpPerspective(computerImage, perspectiveImage, transform,
                    Size(computerImage.cols * 3.2 + 200, computerImage.rows * 3.2 + 170),
                    INTER_LINEAR | WARP_INVERSE_MAP);
    Mat perspSmall = perspectiveImage.clone();
    resize(perspectiveImage, perspSmall, Size(perspectiveImage.cols / 3.2, perspectiveImage.rows / 3.2), 0, 0,
           INTER_LINEAR);
    const float unit = (float) paper_weight / (float) 336;//换算单位
    Mat imageROI = perspSmall(Rect(0, 0, perspSmall.cols, perspSmall.rows - 60));
    Mat copy_imageROI = imageROI.clone();
    Mat binaryImage;
    Mat Gray;
    cvtColor(imageROI, Gray, CV_BGR2GRAY);
    GaussianBlur(Gray, Gray, Size(3, 3), 2, 2);
    threshold(Gray, binaryImage, 78, 255, CV_THRESH_BINARY);
    const int elesize = 3;
    Mat element = getStructuringElement(MORPH_RECT, Size(elesize, elesize));
    morphologyEx(binaryImage, binaryImage, MORPH_CLOSE, element);
    imshow("binaryImage", binaryImage);
    //求轮廓
    vector<vector<Point>> contour;
    vector<Vec4i> hierarchy;
    Mat binaryImage_Copy = binaryImage.clone();
    findContours(binaryImage_Copy, contour, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    Mat imageContours = Mat::zeros(binaryImage_Copy.size(), CV_8UC1);
    for (size_t i = 0; i < contour.size(); i++) {
        if (contour[i].capacity() < 300) {
            continue;
        }
        drawContours(imageContours, contour, i, Scalar(255), 1, 8, hierarchy);
    }
    imshow("imageContours", imageContours);
    Mat imageLines = Mat::zeros(imageContours.size(), CV_8UC1);
    LineFinder finder;
    vector<Vec4i> lines = finder.findLines(imageContours, imageLines);
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
    vector<vector<Vec6f>> allLines = divideAngleLines(linesCount, 3, 7);
    sort(allLines.begin(), allLines.end(), size_cmp);
    int maxL = 2;//每个角度区域线的最大数量
    int maxLA = 2;//角度区域的最大数量
    maxLA = static_cast<int>(maxLA > allLines.size() ? allLines.size() : maxLA);
    for (auto il = allLines.rbegin(); il < allLines.rbegin() + maxLA; il++) {

        vector<Vec6f> largeLine = *il;
        //divide again by x or y
        vector<vector<Vec6f>> oLines;
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
    int i = 0;
    if (linesAver.size() < 2 || linesAver.size() == 2) {
        return -1;
    }
    if (linesAver.size() == 3) {
        if (abs((linesAver[i][3] + linesAver[i + 1][3]) / 2) < abs(linesAver[i + 2][3])) {
            cout << " the small angle is: " << (linesAver[i][3] + linesAver[i + 1][3]) / 2 << endl;
            cout << "delta_angle is: " << (linesAver[i][3] + linesAver[i + 1][3]) / 2 - standard_angle << endl;
        } else {
            cout << " the small angle is: " << linesAver[i + 2][3] << endl;
            cout << "delta_angle is: " << linesAver[i + 2][3] - standard_angle << endl;
        }
    }
    if (abs((linesAver[i][3] + linesAver[i + 1][3]) / 2) < abs((linesAver[i + 2][3] + linesAver[i + 3][3]) / 2)) {
        cout << " the small angle is: " << (linesAver[i][3] + linesAver[i + 1][3]) / 2 << endl;
        cout << "delta_angle is: " << (linesAver[i][3] + linesAver[i + 1][3]) / 2 - standard_angle << endl;

    } else {
        cout << " the small angle is: " << (linesAver[i + 2][3] + linesAver[i + 3][3]) / 2 << endl;
        cout << "delta_angle is: " << (linesAver[i + 2][3] + linesAver[i + 3][3]) / 2 - standard_angle << endl;
    }
    //画平均线
    vector<Point2f> PointGroup;
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
            if (isnan(y) || isnan(x) || (x > singleLine.cols) || (y > singleLine.rows) || (x <= 0.1) ||
                (y <= 0.1))//去掉超出图像的数据
            {
                continue;
            }
            circle(singleLine, p, 3, Scalar(255, 255, 255));
            Vec2f temp(x, y);
            PointGroup.push_back(temp);
        }
    }
    sort(PointGroup.begin(), PointGroup.end(), px_cmp);

    vector<vector<Point2f>> allPoints;
    vector<Point2f> onePoint;
    for (size_t k = 0; k < PointGroup.size(); k++) {
        onePoint.clear();
        Vec2f fourp_k(PointGroup[k].x, PointGroup[k].y);
        onePoint.push_back(fourp_k);
        for (size_t g = 0; g < PointGroup.size(); g++) {
            if (g == k) {
                continue;
            }
            if (this->distance(PointGroup[k], PointGroup[g]) < Maxdist) {
                onePoint.push_back(PointGroup[g]);
            }
        }
        allPoints.push_back(onePoint);
    }
    int max = 0;
    int index_max = 0;
    for (int i = 0; i < allPoints.size(); i++) {
        if (allPoints[i].size() > max) {
            max = allPoints[i].size();
            index_max = i;
        }
    }
    int index = -1;
    int index_copy;
    bool flag = true;
    for (int i_maxC = 0; i_maxC < allPoints.size(); i_maxC++) //为了max - 1 继续像max那样循环使用设置的循环
    {
        for (int i_aP = 0; i_aP < allPoints.size(); i_aP++) {
            if (allPoints[i_aP].size() == max) {
                for (int i = 0; i < max; i++) {
                    for (int j = i + 1; j < max; j++) {
                        if (distance(allPoints[index_max][i], allPoints[index_max][j]) > Maxdist) {
                            j = max;
                            i = max;    //跳出两层循环
                            flag = false;
                            index_copy = index;
                        }
                    }
                }
                if (flag) {
                    index = i_aP;
                    i_aP = allPoints.size();
                    i_maxC = allPoints.size();
                }
                flag = true;
            }
        }
        if (index == index_copy)//如果index等于上一个index，即前面没有一个把i_aP赋给index的，所有集合里的点都不符合两两距离小于阈值
        {
            max--;
        } else {
            break;
        }
    }
    for (int i_fits = 0; i_fits < allPoints[index].size(); i_fits++) {
        circle(singleLine, allPoints[index][i_fits], 3, Scalar(150, 150, 155));
    }
    vector<Vec3f> threePts;
    if (index == -1) {
        return -1;
    } else if (max == 1) {
        return -1;
    } else if (max == 2) {
        return -1;
    } else if (max == 3)//如果只有三个点，预测中点，即斜边中点
    {
        float d = 0;
        for (int one = 0; one < 3; one++) {
            for (int two = one + 1; two < 3; two++) {
                d = distance(allPoints[index][one], allPoints[index][two]);
                Vec3f threep(d, one, two);
                threePts.push_back(threep);
            }
        }
        float max_d;
        if (threePts[0][0] > threePts[1][0]) {
            max_d = threePts[0][0];
        } else {
            max_d = threePts[1][0];
        }
        if (threePts[2][0] > max_d) {
            max_d = threePts[2][0];
        }
        for (int b = 0; b < 3; b++) {
            if (max_d == threePts[b][0]) {
                int e = threePts[b][1];
                int f = threePts[b][2];
                Point center;
                center.x = (allPoints[index][e].x + allPoints[index][f].x) / 2;
                center.y = (allPoints[index][e].y + allPoints[index][f].y) / 2;
                circle(singleLine, center, 3, Scalar(0, 0, 255));
                cout << "the real centerX is: " << standard_real_x + (standard_picture_y - center.y) * unit << "\t"
                     << "Y is: " << standard_real_y + (center.x - standard_picture_x) * unit << endl;
                cout << "the real centerX is: " << standard_real_x + (standard_picture_x - center.x) * unit << "\t"
                     << "Y is: " << standard_real_y + (standard_picture_y - center.y) * unit << endl;
                point->x = center.x;
                point->y = center.y;
                return 1;
            }
        }
    } else {
        float x = 0;
        float y = 0;
        for (int c = 0; c < max; c++) {
            x += allPoints[index][c].x;
            y += allPoints[index][c].y;
        }
        Point center;
        center.x = x / max;
        center.y = y / max;
        circle(singleLine, center, 3, Scalar(0, 0, 255));
        cout << "the real centerX is: " << standard_real_x + (standard_picture_y - center.y) * unit << "\t" << "Y is: "
             << standard_real_y + (center.x - standard_picture_x) * unit << endl;
        cout << "the real centerX is: " << standard_real_x + (standard_picture_x - center.x) * unit << "\t" << "Y is: "
             << standard_real_y + (standard_picture_y - center.y) * unit << endl;
        point->x = center.x;
        point->y = center.y;
        return 1;
    }
    //test
    imshow("singleLine", singleLine);
    waitKey(0);
    return 0;
}