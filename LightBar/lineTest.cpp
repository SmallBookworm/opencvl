//
// Created by peng on 18-4-8.
//

#include "lineTest.h"

using namespace std;
using namespace cv;

/*
寻找光线色差，分离通道
*/
bool LineTest::is_nan(double dVal) {
    if (dVal == dVal) {
        return false;
    }
    return true;
}

void LineTest::GetDiffImage(Mat src1, Mat src2, Mat dst, int nThre) {

    for (int i = 0; i < HEIGHT; i++) {
        uchar *pData1 = src1.ptr<uchar>(i);
        uchar *pData2 = src2.ptr<uchar>(i);
        uchar *pData3 = dst.ptr<uchar>(i);
        for (int j = 0; j < WIDTH; j++) {
            if (pData1[j] - pData2[j] > nThre) {
                pData3[j] = 254;
            } else {
                pData3[j] = 0;
            }
        }
    }
}

bool LineTest::comp(const Vec4i &a, const Vec4i &b) {
    return a[0] < b[0];

}

float LineTest::RadianCalculate(float rleft, float rright) {
    float radian = asinf((float) (rleft - rright) / (float) sreal_width);
    return radian;
}

float LineTest::rad(float ang) {
    return (float) ang * M_PI / (float) 180;
}

float LineTest::AngleCalculate(float rleft, float rright) {
    float radian = asinf((float) (rleft - rright) / (float) sreal_width);
    float angle = (float) radian * 180 / (float) M_PI;
    return angle;
}

vector<Vec4i> LineTest::findCorner(Mat dst, Mat src) {
    vector<vector<Point> > contours;
    vector<Vec4i> lines;
    cv::findContours(dst, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++) {
        if (contours[i].capacity() > 300) {
            Vec4i leftLine;
            Vec4i rightLine;
            int averageRightDown = 0;
            int averageRightDownNum = 0;
            int averageLeftUp = 0;
            int averageLeftUpNum = 0;
            int averageLeftDown = 0;
            int averageLeftDownNum = 0;
            int averageRightUp = 0;
            int averageRightUpNum = 0;

            int rightDown = 0;
            int rightDownD = 0;
            int leftUp = WIDTH * HEIGHT;
            int leftUpD = 0;
            int leftDown = 0;
            int leftDownD = 0;
            int rightUp = 0;
            int rightUpD = 0;

            for (int j = 0; j < contours[i].size(); j++) {
                if (contours[i][j].x * contours[i][j].y > rightDown) {
                    rightDownD = j;
                    rightDown = contours[i][j].x * contours[i][j].y;
                }
                if (contours[i][j].x * contours[i][j].y < leftUp) {
                    leftUpD = j;
                    leftUp = contours[i][j].x * contours[i][j].y;
                }
                if (contours[i][j].x * (HEIGHT - contours[i][j].y) > rightUp) {
                    rightUpD = j;
                    rightUp = contours[i][j].x * (HEIGHT - contours[i][j].y);
                }
                if ((WIDTH - contours[i][j].x) * contours[i][j].y > leftDown) {
                    leftDownD = j;
                    leftDown = (WIDTH - contours[i][j].x) * contours[i][j].y;
                }

            }
            if (contours[i][leftDownD].y - contours[i][leftUpD].y < 150 ||
                contours[i][rightDownD].y - contours[i][rightUpD].y < 150) {
                return lines;
            }
            for (int j = 0; j < contours[i].size(); j++) {
                if (contours[i][j].x > (contours[i][rightDownD].x - AVG) &&
                    contours[i][j].x < (contours[i][rightDownD].x + AVG)) {
                    averageRightDown += contours[i][j].x;
                    averageRightDownNum++;
                }
                if (contours[i][j].x > (contours[i][leftUpD].x - AVG) &&
                    contours[i][j].x < (contours[i][leftUpD].x + AVG)) {
                    averageLeftUp += contours[i][j].x;
                    averageLeftUpNum++;
                }
                if (contours[i][j].x > (contours[i][leftDownD].x - AVG) &&
                    contours[i][j].x < (contours[i][leftDownD].x + AVG)) {
                    averageLeftDown += contours[i][j].x;
                    averageLeftDownNum++;
                }
                if (contours[i][j].x > (contours[i][rightUpD].x - AVG) &&
                    contours[i][j].x < (contours[i][rightUpD].x + AVG)) {
                    averageRightUp += contours[i][j].x;
                    averageRightUpNum++;
                }


            }
            averageRightDown = averageRightDown / averageRightDownNum;
            averageLeftUp = averageLeftUp / averageLeftUpNum;
            averageLeftDown = averageLeftDown / averageLeftDownNum;
            averageRightUp = averageRightUp / averageRightUpNum;

            Point rd(averageRightDown, contours[i][rightDownD].y);
            Point lu(averageLeftUp, contours[i][leftUpD].y);
            Point ld(averageLeftDown, contours[i][leftDownD].y);
            Point ru(averageRightUp, contours[i][rightUpD].y);

            /*cv::circle(src, rd, 2, cv::Scalar(255, 0, 0));
            cv::circle(src, lu, 2, cv::Scalar(255, 0, 0));
            cv::circle(src, ld, 2, cv::Scalar(255, 0, 0));
            cv::circle(src, ru, 2, cv::Scalar(255, 0, 0));*/

            leftLine[0] = ld.x;
            leftLine[1] = ld.y;
            leftLine[2] = lu.x;
            leftLine[3] = lu.y;
            rightLine[0] = rd.x;
            rightLine[1] = rd.y;
            rightLine[2] = ru.x;
            rightLine[3] = ru.y;
            lines.push_back(leftLine);
            lines.push_back(rightLine);
        }
    }
    sort(lines.begin(), lines.end(), comp);
    return lines;
}

void LineTest::analyse(LinesOption all_line, LinesOption left_line, LinesOption right_line, LinesOption left_line2,
                       LinesOption right_line2, vector<Vec4i> lines) {

    //for (int i = 0; i < lines.size(); i++) {
    //	cout << i + 1 << " ";
    //	for (int j = 0; j < 4; j++) {
    //		cout << lines[i][j] << " ";
    //	}
    //	cout << endl;
    //}
    //下面的点：[0]x1 [1]y1 上面的点：[2]x2 [3]y2
    float pix_left_height = left_line.pixheight(0, lines) - 2;//-2经验值，每次都多检测大概2像素
    float pix_right_height = right_line.pixheight(3, lines) - 2;
    float real_left_D = left_line.realdist(0, lines);
    float real_right_D = right_line.realdist(3, lines);
    float pic_angle = AngleCalculate(real_left_D, real_right_D);
    float angle = pic_angle - sinit_angle;
    //cout << "the angle is : " << angle << endl;
    float radian = rad(angle);
    float leftToCenter = left_line.centerPoint(0, lines).x - WIDTH / 2;
    float rightToCenter = right_line.centerPoint(3, lines).x - WIDTH / 2;
    const float sleftToCenter = -126;//$$$$$$$$$$$$$$$$$$
    const float srightToCenter = 146;//$$$$$$$$$$$$$$$$$$
    const float unit = (float) sreal_height / (float) spix_light_height;
    int r = 2;

    float vectRadian;
    float vectLength;
    if (pix_left_height > pix_right_height) {
        float pix_delta_x = (float) sreal_width * ((float) leftToCenter / (float) left_line.surposepixWidth(0, lines) -
                                                   (float) sleftToCenter / (float) spix_light_width);
        float real_delta_x = pix_delta_x * unit;
        float real_delta_d = real_left_D - sD;
        float a[2] = {real_delta_x, real_delta_d};
        Mat av = Mat(2, 1, CV_32FC1, a);
        float b[2] = {sleftToCenter * (float) sreal_width / (float) spix_light_width, sD};
        Mat bv = Mat(2, 1, CV_32FC1, b);
        float rotate[4] = {cosf(-radian), -sinf(-radian), sinf(-radian), cosf(-radian)};
        Mat rotatev = Mat(2, 2, CV_32FC1, rotate);
        Mat cv = rotatev * bv;
        Mat dv = bv + av - cv;
        float *data1 = dv.ptr<float>(0);
        float x = data1[0];
        float *data2 = dv.ptr<float>(0);
        float y = data1[1];
        vectRadian = atan2f(y, x);
        vectLength = sqrtf(powf(x, 2) + powf(y, 2));
        //cout << "vectRadian: " << vectRadian << endl;//
        //cout << "vectLength: " << vectLength << endl;
    }
    if (pix_right_height > pix_left_height) {
        float pix_delta_x = (float) sreal_width *
                            ((float) rightToCenter / (float) right_line.surposepixWidth(3, lines) -
                             (float) srightToCenter / (float) spix_light_width);
        float real_delta_x = pix_delta_x * unit;
        float real_delta_d = real_right_D - sD;
        float a[2] = {real_delta_x, real_delta_d};
        Mat av = Mat(2, 1, CV_32FC1, a);
        float b[2] = {srightToCenter * (float) sreal_width / (float) spix_light_width, sD};
        Mat bv = Mat(2, 1, CV_32FC1, b);
        float rotate[4] = {cosf(-radian), -sinf(-radian), sinf(-radian), cosf(-radian)};
        Mat rotatev = Mat(2, 2, CV_32FC1, rotate);
        Mat cv = rotatev * bv;
        Mat dv = bv + av - cv;
        float *data1 = dv.ptr<float>(0);
        float x = data1[0];
        float *data2 = dv.ptr<float>(0);
        float y = data1[1];
        vectRadian = atan2f(y, x);
        vectLength = sqrtf(powf(x, 2) + powf(y, 2));
        //cout << "vectRadian: " << vectRadian << endl;
        //cout << "vectLength: " << vectLength << endl;
    }
    if (pix_left_height == pix_right_height) {
        float pix_delta_x = (float) sreal_width * ((float) srightToCenter / (float) spix_light_width -
                                                   (float) rightToCenter / (float) (right_line.pixheight(3, lines) /
                                                                                    (float) (spix_light_height /
                                                                                             (float) spix_light_width)));
        float real_delta_x = pix_delta_x * unit;
        float real_delta_d = real_right_D - sD;
        float a[2] = {real_delta_x, real_delta_d};
        Mat av = Mat(2, 1, CV_32FC1, a);
        float b[2] = {srightToCenter * (float) sreal_width / (float) spix_light_width, sD};
        Mat bv = Mat(2, 1, CV_32FC1, b);
        float rotate[4] = {cosf(-radian), -sinf(-radian), sinf(-radian), cosf(-radian)};
        Mat rotatev = Mat(2, 2, CV_32FC1, rotate);
        Mat cv = rotatev * bv;
        Mat dv = bv + av - cv;
        float *data1 = dv.ptr<float>(0);
        float x = data1[0];
        float *data2 = dv.ptr<float>(0);
        float y = data1[1];
        vectRadian = atan2f(y, x);
        vectLength = sqrtf(powf(x, 2) + powf(y, 2));
        //cout << "vectRadian: " << vectRadian << endl;
        //cout << "vectLength: " << vectLength << endl;
    }
    this->info_value[0] = vectLength;
    this->info_value[1] = vectRadian;
    this->info_value[2] = angle;

//    cout << "the angle is : " << angle << endl;
//    cout << "vectRadian: " << vectRadian << endl;
//    cout << "vectLength: " << vectLength << endl;
}

void LineTest::drawDetectLines(Mat &image, const vector<Vec4i> &lines, Scalar &color) {
    // 将检测到的直线在图上画出来
    vector<Vec4i>::const_iterator it = lines.begin();
    while (it != lines.end()) {
        Point pt1((*it)[0], (*it)[1]);
        Point pt2((*it)[2], (*it)[3]);
        line(image, pt1, pt2, color, 2); //  线条宽度设置为2
        ++it;
    }
}

int LineTest::watch(cv::Mat src) {
    Mat pBinary, record, dst;
    dst = Mat::zeros(Size(WIDTH, HEIGHT), CV_8UC1);
    vector<Mat> mv;
    vector<Vec4i> lines;
    Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
    Mat elementE = getStructuringElement(MORPH_RECT, Size(9, 9));
    //亮度调整
    src.convertTo(record, -1, 0.1, 0);
    //通道分离
    split(record, mv);
    //mv[2] 红 mv[1] 绿 mv[0] 蓝
    //得到差异图像，转为黑白
    GetDiffImage(mv[0], mv[1], dst, 6);
    //先膨胀，后腐蚀（联通区域）
    cv::dilate(dst, pBinary, element);
    cv::erode(pBinary, dst, elementE);
    //cv::imshow("middle", dst);
    //GaussianBlur(dst, dst, Size(5, 5), 0, 0);
    //得到角点
    lines = findCorner(dst, src);
    if (lines.size() == 4) {
        //drawDetectLines(src, lines, Scalar(0, 255, 0));
        analyse(all_line, left_line, right_line, left2_line, right2_line, lines);
    } else if (lines.size() < 4) {
        //cout << "invalid " << lines.size() << endl;
    } else if (lines.size() > 4) {
        //cout << "invalid " << lines.size() << endl;
    }
    return static_cast<int>(lines.size());
}

int LineTest::operator()(LineInfo &info) {
    VideoCapture capture;
    capture.open("/home/peng/下载/realse/1.avi");

    Mat srcImage;
    if (!capture.isOpened()) {
        std::cout << "fail to open video!" << std::endl;
        return -1;
    }

    bool status=info.getStop();
    while (!status) {
        capture >> srcImage;
        if (!capture.isOpened() || srcImage.empty())
            break;
        int size = watch(srcImage);
        //test
        //cout << size << endl;
        if (size == 4) {
            info.set(info_value);
        }
        //test
        imshow("show", srcImage);
        if (waitKey(1) == 27) {
            break;
        }
        status = info.getStop();
    };
    return 0;
}
