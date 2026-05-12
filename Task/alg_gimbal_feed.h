#ifndef ALG_GIMBAL_FEED_H
#define ALG_GIMBAL_FEED_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 重力补偿计算函数
 * @param position 当前云台轴位置（弧度）
 * @return float 输出的力矩补偿值
 */
float G_feed_calculate(float position);

#ifdef __cplusplus
}
#endif

#endif /* ALG_GIMBAL_FEED_H */