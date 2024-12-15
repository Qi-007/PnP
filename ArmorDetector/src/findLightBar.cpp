#include "detect.h"

bool isVerticalLike(float angle) {
    return angle <= 15 || angle >= 165;
}

//按轮廓的宽高比.面积大小筛选轮廓
vector<LightDescriptor> findLightBar:: Lights(const vector<vector<Point>>& contours){
    float minArea = 100.0f;   //最小面积
    float maxArea = 1500.0f;   // 最大面积
    double minRatio = 1.2;   //最小宽高比  
    double maxRatio = 8.0;   //最大宽高比
    //存储筛选过的灯条
    vector<LightDescriptor> m_light;

    // 遍历轮廓，拟合椭圆并计算外接矩形
    // 因为灯条是椭圆型的，所以用椭圆去拟合轮廓，再直接获取旋转外接矩形
    for (size_t i = 0; i < contours.size(); ++i) {
        if (contours[i].size() < 5) continue; // 拟合椭圆需要至少5个点
        // 获取椭圆的外接矩形
        RotatedRect Light_Rec = fitEllipse(contours[i]);

        // // 通过倾斜角度筛选灯条
        if (!isVerticalLike(Light_Rec.angle)) {
            continue;
        }

        // 计算宽高比
        float aspectRatio = static_cast<float> (Light_Rec.size.height) / (Light_Rec.size.width); 
        if(aspectRatio > minRatio && aspectRatio < maxRatio && Light_Rec.size.area() > minArea &&
            Light_Rec.size.area() < maxArea){
                // cout << "height:" << Light_Rec.size.height << "   width :" << Light_Rec.size.width << endl;
                // cout << Light_Rec.angle << endl;
                // cout << "area:  " << Light_Rec.size.area() << endl;
                // cout << "ratio:" << aspectRatio << endl;
            m_light.push_back(Light_Rec);
        }
   }
    return m_light;
}