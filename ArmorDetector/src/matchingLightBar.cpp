#include "detect.h"

// 角度差值并进行标准化
float angle_normalization(float& angle){
    if(angle >= 90){
        angle = fabs(angle - 180);
    }
    return angle;
}

//两两匹配灯条
vector<pair<LightDescriptor, LightDescriptor>> matchingLightBar:: matchLight(const vector<LightDescriptor>& lights){
    // float max_angle_diff = 5.0f;    //筛选平行角度的阈值
    // float max_len_diff_ratio = 0.4;     // 灯条长度差的最大比率

    // vector<pair<LightDescriptor, LightDescriptor>> matched;

    // // 双重循环检查每对矩形
    // for (size_t i = 0; i < lights.size(); ++i) {
    //     for (size_t j = i + 1; j < lights.size(); ++j) {
    //         LightDescriptor leftLight = lights[i];
    //         LightDescriptor rightLight = lights[j];

    //         //角度值归一化
    //         angle_normalization(rightLight.angle);
    //         angle_normalization(leftLight.angle);

    //         //cout << fabs(leftLight.angle - rightLight.angle) << endl;

    //         // 角差
    //         float angleDiff = fabs(leftLight.angle - rightLight.angle);

    //         // 长度差比率
    //         float lenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);

    //         // 跳过不符合条件的灯条
    //         if(
    //             angleDiff > max_angle_diff ||
    //             lenDiff_ratio > max_len_diff_ratio
    //         ){
    //             continue;
    //         }
    //             // cout << lenDiff_ratio << endl; 
    //             matched.emplace_back(leftLight, rightLight);
    //         }
    //     }
    // return matched;
// }

// 按距离进行排序的匹配灯条
    float max_angle_diff = 10.0f;        // 筛选平行角度的阈值
    float max_len_diff_ratio = 0.25f;   // 灯条长度差的最大比率
    float max_dist = 500.0f;           // 最大距离阈值

    vector<pair<LightDescriptor, LightDescriptor>> matched;

    // 对灯条的所有可能组合按照距离排序
    vector<tuple<float, size_t, size_t>> distances; // (距离, index1, index2)
    for (size_t i = 0; i < lights.size(); i++) {
        for (size_t j = i + 1; j < lights.size(); j++) {
            float dist = calculateDistance(lights[i].center, lights[j].center);
            distances.emplace_back(dist, i, j);
        }
    }

    // 按距离从小到大排序
    sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return get<0>(a) < get<0>(b);
    });

    // 检查每对灯条是否符合条件
    for (const auto& [dist, i, j] : distances) {
        if (dist > max_dist) break; // 跳过超过最大距离的灯条

        LightDescriptor leftLight = lights[i];
        LightDescriptor rightLight = lights[j];

        // 角度值归一化
        angle_normalization(rightLight.angle);
        angle_normalization(leftLight.angle);

        // 角差
        float angleDiff = fabs(leftLight.angle - rightLight.angle);

        // 长度差比率
        float lenDiff_ratio = fabs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);

        // 跳过不符合条件的灯条
        if (angleDiff > max_angle_diff || lenDiff_ratio > max_len_diff_ratio) {
            continue;
        }

        matched.emplace_back(leftLight, rightLight);
    }

    return matched;
}
