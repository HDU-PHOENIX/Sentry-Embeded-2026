#include "alg_gimbal_feed.h"

/**
 * @brief 重力补偿计算函数
 * @param position 当前云台轴位置（弧度）
 * @return float 输出的力矩补偿值
 * @note 采用三次多项式拟合曲线生成重力补偿力矩
 */
float G_feed_calculate(float position) {
    // 历史补偿系数记录（根据需求切换使用）:
    // Sine model:
    // float torque = -21.321498f * sinf(position - 1.625094f) - 21.348101f;
    //
    // Polynomial models [a, b, c, d]:
    // [354.533813, -99.411941, 7.604260, -0.441783]
    // [ 1.34165963, -1.72110943, 0.29976696, -0.17707916]
    // [2.319374, -3.015623, 0.755581, -0.311670]
    // [1.549729, -2.003787, 0.406681, -0.285731]
    // [0.386597, -0.374320, -0.419100, 0.075079]（装了枪管的）

    // 当前使用的补偿系数：
    float a = 1.016980f;
    float b = -1.500239f;
    float c = -0.062756f;
    float d = 0.195295f;

    // a * x^3 + b * x^2 + c * x + d
    float torque = (a * position * position * position) + 
                   (b * position * position) + 
                   (c * position) + d;
                   
    return torque;
}