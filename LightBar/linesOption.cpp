//
// Created by peng on 18-4-8.
//

#include "linesOption.h"

using namespace std;
using namespace cv;

Point2f LinesOption::upPoint(int i, vector<Vec4i> &a) {
    Point2f p;
    p.x = a[i][2];
    p.y = a[i][3];
    return p;
}

Point2f LinesOption::downPoint(int i, vector<Vec4i> &a) {
    Point2f p;
    p.x = a[i][0];
    p.y = a[i][1];
    return p;
}

Point2f LinesOption::centerPoint(int i, vector<Vec4i> &a) {
    Point2f p;
    p.x = (a[i][0] + a[i][2]) / 2;
    p.y = (a[i][1] + a[i][3]) / 2;
    return p;
}

int LinesOption::pixheight(int i, vector<Vec4i> &a) {
    return a[i][1] - a[i][3];
}

float LinesOption::realdist(int i, vector<Vec4i> &a) {
    return (float) sD * spix_light_height / (float) (a[i][1] - a[i][3]);//a[i][1] - a[i][3]:pixheight
}

float LinesOption::surposepixWidth(int i, vector<Vec4i> &a) {
    return (a[i][1] - a[i][3]) * ((float) spix_light_width / (float) spix_light_height); //a[i][1] - a[i][3]:pixheight
}