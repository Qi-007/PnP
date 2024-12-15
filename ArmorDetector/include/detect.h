#ifndef DETECT_H
#define DETECT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>  // 包含 sort 函数

using namespace std;
using namespace cv;

//计算两点之间的欧几里得距离
inline float calculateDistance(const Point2f& p1, const Point2f& p2){
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// 创建灯条类
class LightDescriptor{
public:
    float width, length, angle, area;    // 宽度，长度，角度，面积
    Point2f center, point[4];    //灯条的质心，4个顶点
public:
    LightDescriptor() {};    //简单声明灯条
    LightDescriptor(const RotatedRect& light){
        width = light.size.width;
        length = light.size.height;
        angle = light.angle;
        area = light.size.area();
        center = light.center;
        light.points(point);
    }
};

//图像的预处理
class imageDispose{
public:
    Mat imageGaussion(const Mat& m_frame);   //使用高斯函数平滑图像，减少噪声
    Mat stressBlue(const Mat& m_frame);   //筛选出蓝色
    Mat stressRed(const Mat& m_frame);    //筛选出红色
    Mat imageThreshold(const Mat& m_frame, const double& thresh);    //对彩色/灰色图像进行二值化处理
    Mat imageDilate(const Mat& m_binaryImage);    //对二值化图像进行膨胀
    Mat imageClose(const Mat& binary);       // 对二值化图像进行闭运算处理
    Mat imageOpen(const Mat& binary);       // 对二值化图像进行开运算处理
};

//检测灯条
class findLightBar {
public:
    // //按宽高比筛选轮廓
    // vector<RotatedRect> ScreenAspect(const vector<vector<Point>>& m_contours, const float& minRatio, const float& maxRatio, const float& minArea, vector<RotatedRect>& m_rightAspectRect);

    vector<LightDescriptor> Lights(const vector<vector<Point>>& CONTOURS_MATCH_I2);
};

//匹配灯条
class matchingLightBar{
public:
    vector<pair<LightDescriptor, LightDescriptor>> matchLight(const vector<LightDescriptor>& lights);
};

// 识别装甲板
class findArmor{
public:
    vector<pair<LightDescriptor, LightDescriptor>> find_Armor(vector<pair<LightDescriptor, LightDescriptor>>& m_matchingLightBar);
};

// 匹配装甲板
class matchingArmor{
public:
    //绘制装甲板
    Mat matchingArmors(const vector<pair<LightDescriptor, LightDescriptor>>& m_lights, Mat& m_frame, vector<Point2f>& m_armor);
};

#endif

