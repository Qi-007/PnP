#include "detect.h"

// //绘制最小外接矩形
// void drawRotatedRect1(Mat& image, const RotatedRect& rotatedRect, const Scalar& color, int thickness) {
//     // 获取 RotatedRect 的四个顶点
//     Point2f vertices[4];
//     rotatedRect.points(vertices);

//     // 将四个顶点连线绘制出旋转矩形
//     for (int i = 0; i < 4; ++i) {
//         line(image, vertices[i], vertices[(i + 1) % 4], color, thickness);
//     }
// }

// //计算两点之间的欧几里得距离
// float calculateDistance2(const Point2f& p1, const Point2f& p2){
//     return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
// }

// //计算最小外接矩形质心
// Point2f getCentroid(const Rect& rect){
//     return Point2f(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
// }

Mat matchingArmor:: matchingArmors(const vector<pair<LightDescriptor, LightDescriptor>>& m_lights, Mat& m_frame, vector<Point2f>& m_armor){
    // 如果没有灯条，则直接返回原始图像
    if (m_lights.empty()) {
        return m_frame;
    }

    double min_armorArea = 500.0;

    //绘制装甲板区域
    for(const auto& m_light : m_lights){

        const auto& leftLight = m_light.first;
        const auto& rightLight = m_light.second;

        // 提取两个旋转矩形的顶点
        // 顶点的顺序是按矩形的边排列，依次为左上、右上、右下、左下（以逆时针顺序为准）。
        Point2f vertices1[4], vertices2[4];
        for(int i = 0; i < 4; i++){
            vertices1[i] = leftLight.point[i];
        }
        for(int i = 0; i < 4; i++){
            vertices2[i] = rightLight.point[i];
        }
 
        // 合并所有顶点
        vector<Point2f> allPoints(vertices1, vertices1 + 4);
        allPoints.insert(allPoints.end(), vertices2, vertices2 + 4);

        // 转换为整型点
        vector<Point> intPoints;
        for (const auto& pt : allPoints) {
            intPoints.emplace_back(Point(cvRound(pt.x), cvRound(pt.y)));
        }

        // 使用cv::convexHull计算凸包
        vector<Point> hull;
        convexHull(intPoints, hull);

        // 计算凸包面积（面积太小的装甲板不要）
        double armorArea = contourArea(hull);

        if(armorArea < min_armorArea){
            continue;
        } 

        // cout << armorArea << endl;

        // 绘制凸包
        for (size_t i = 0; i < hull.size(); i++) {
            cv::line(m_frame, hull[i], hull[(i+1) % hull.size()], cv::Scalar(255, 255, 255), 2);
        }

        RotatedRect armor = minAreaRect(hull);
        vector<Point2f> m_armor_points;
        armor.points(m_armor_points);
        
        // 按左上. 左下. 右下. 右上的顺序存入m_armor
        // 按x从左到右排序
        sort(m_armor_points.begin(), m_armor_points.end(), [](const Point2f& a, const Point2f& b) {
            return a.x < b.x;
        });

        // 输入左上左下两个点
        if(m_armor_points[0].y < m_armor_points[1].y){
            m_armor.emplace_back(m_armor_points[0]);
            m_armor.emplace_back(m_armor_points[1]);
        }else{
            m_armor.emplace_back(m_armor_points[1]);
            m_armor.emplace_back(m_armor_points[0]);
        }   
    
        // 输入右下右上两个点
        if(m_armor_points[2].y < m_armor_points[3].y){
            m_armor.emplace_back(m_armor_points[3]);
            m_armor.emplace_back(m_armor_points[2]);
        }else{
            m_armor.emplace_back(m_armor_points[2]);
            m_armor.emplace_back(m_armor_points[3]);
        }

    }
    return m_frame;
}
