#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "detect.h"
#include "PnPslover.h"
#include "HikDriver/HikDriver.h"

using namespace std;
using namespace cv;

//绘制最小外接矩形(灯条类)
void drawLight(Mat& image, const LightDescriptor& light, const Scalar& color, int thickness) {
    // 将四个顶点连线绘制出旋转矩形
    for (int i = 0; i < 4; ++i) {
        line(image, light.point[i], light.point[(i + 1) % 4], color, thickness);
    }
    return;
}

// 通道相减图像的预处理
void minus_dispose(const Mat& m_frame, Mat& m_dst){
    imageDispose minus_dispose;

    // 使用高斯函数平滑图像，减少噪声
    Mat blurred, red_minus_blue, blue_minus_red, binary, open, close;
    blurred = minus_dispose.imageGaussion(m_frame);
    // 红蓝通道相减，强调红色区域
    red_minus_blue = minus_dispose.stressRed(blurred);

    // // 蓝红通道相减，强调蓝色区域
    // blue_minus_red = minus_dispose.stressBlue(blurred);

    // 对通道相减图像进行二值化处理
    binary = minus_dispose.imageThreshold(red_minus_blue, 100);

    // 对二值化后的图像进行开运算处理
    open = minus_dispose.imageOpen(binary);

    // 进行闭运算处理
    close = minus_dispose.imageClose(open);

    // 对闭运算后的图像进行膨胀处理
    m_dst = minus_dispose.imageDilate(close);
}

// 原图像的预处理
void image_dispose(const Mat& m_frame, Mat& m_binaryImage){
    imageDispose frame_dispose;

    // 使用高斯函数平滑图像，减少噪声
    Mat blurred;
    blurred = frame_dispose.imageGaussion(m_frame);

    // 对原图像进行二值化处理
    m_binaryImage = frame_dispose.imageThreshold(blurred, 140);
}


// #define FPS 150

// int main(){
//     HikDriver hik_driver(0);
//     if (hik_driver.isConnected()) {
//         hik_driver.setExposureTime(30000);
//         hik_driver.setGain(15);
//         hik_driver.showParamInfo();
//         hik_driver.startReadThread();
//     }

//     HikFrame Hik_image = hik_driver.getFrame();

//     Mat frame = Hik_image.getRgbFrame()->clone();

//     while (true) {
//         Hik_image = hik_driver.getFrame();
//         frame = Hik_image.getRgbFrame()->clone();

//         if (frame.empty()) {
//             std::cout << " empty" << std::endl;
//             continue;
//         }
//         cv::imshow("frame", frame);
//         cv::waitKey(1000/FPS);
//     }
// }

// 相机内参
array<double, 9> intrinsic_matrix = {1806.202836613486, 0, 706.479880371389,
                                     0, 1812.980528629544, 546.7058549527911,
                                     0, 0, 1};
// 畸变系数
vector<double> distortion_vector = {-0.1066716215207354, 1.026102429629, -0.0003129016621888697, -0.001878173941429469, -6.758932699953562};


int main(){
    HikDriver hik_driver(0);
    if(hik_driver.isConnected()){
        hik_driver.setExposureTime(30000);
        hik_driver.setGain(15);
        hik_driver.showParamInfo();
        hik_driver.startReadThread();
    }

    // //读取视频文件
    // VideoCapture cap("/home/zjq/Desktop/detect_video/前哨站/红方前哨站下方视角全速.mp4");
    // if(!cap.isOpened()){
    //     cout << "视频加载失败" << endl;
    // }

    // Mat frame;      // 原图像
    Mat binaryImage,    // 原图处理后的图像
        dst;     // 通道相减处理后的图像

    vector<vector<Point>> all_contours;     //未经筛选的轮廓
    vector<LightDescriptor> lights;       //筛选过宽高比的矩形
    vector<pair<LightDescriptor, LightDescriptor>> matching_lights;    //根据倾斜角等筛选出的配对灯条
    vector<pair<LightDescriptor, LightDescriptor>> foundArmor;      // 识别后的装甲板

    #define FPS 180

    while(true){
    // // 读取每一帧
    // cap >> frame;
    // if(frame.empty()){
    //     break;
    // }

    HikFrame Hik_image = hik_driver.getFrame();

    Mat frame = Hik_image.getRgbFrame()->clone();

    if(frame.empty()){
        cout << " empty " << endl;
        continue;
    }

    // 通道相减图像的二值化处理
    thread minusDispose(minus_dispose, ref(frame), ref(dst));

    // 原图像的二值化处理
    thread frameDispose(image_dispose, ref(frame), ref(binaryImage));

    // 等待线程执行完毕
    minusDispose.join();
    frameDispose.join();

    // imshow("dst", dst);
    // imshow("binaryImage", binaryImage);

    Mat image;
    bitwise_and(dst, binaryImage, image);
    // imshow("image", image);

    // 寻找轮廓
    findContours(image, all_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //识别灯条
    findLightBar all_lightBar;
    lights = all_lightBar.Lights(all_contours);

    //匹配灯条
    matchingLightBar right_lightBar;

    //两两匹配灯条
    matching_lights = right_lightBar.matchLight(lights);

    // 识别装甲板
    findArmor armors;
    foundArmor = armors.find_Armor(matching_lights);

    // 匹配装甲板
    vector<Point2f> armor;
    matchingArmor all_armors;
    frame = all_armors.matchingArmors(foundArmor, frame, armor);
    if(!armor.empty()){
        Mat tvec, rvec;
        rvec.create(3, 1, CV_64F);  // 创建 3x1 的双精度矩阵
        tvec.create(3, 1, CV_64F);  // 创建 3x1 的双精度矩阵
        armor_auto_aim:: ArmorPnpSlover slover(intrinsic_matrix, distortion_vector);
        slover.drawPoints(armor, frame);
        slover.pnpSlove(armor, rvec, tvec);
        // std::cout <<  "rvec = " << rvec.at<double>(0,0) << " ," << rvec.at<double>(1,0) << " ," << rvec.at<double>(2,0) << std::endl;
        // std::cout << "tvec = " << tvec.at<double>(0,0) << " ," << tvec.at<double>(1,0) << " ," << tvec.at<double>(2,0) << std::endl;

        // 计算相机距离被测物的实际距离
        float distance = sqrt(tvec.at<double>(0,0) * tvec.at<double>(0,0) + tvec.at<double>(1,0) * tvec.at<double>(1,0) + tvec.at<double>(2,0) * tvec.at<double>(2,0)); 
        if(distance > 0){
            std::cout << "distance = "<< distance << std::endl;
        }
    }

    imshow("前哨站", frame);

    all_contours.clear();       // 清空上一帧的轮廓
    lights.clear();    // 清空上一帧筛选的矩形
    matching_lights.clear();    // 清空上一帧的配对灯条
    foundArmor.clear();     // 清空上一帧的装甲板
    armor.clear();     

    waitKey(1000/FPS);

    // char c = waitKey(1000/cap.get(CAP_PROP_FPS));
    // char c = waitKey(0);
    // if(c == 'q' || c == 27){
    //     break;
    // }
    }
    // cap.release();
    destroyAllWindows();
    return 0;
}