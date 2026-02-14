#ifndef ALG_HEAT_CONTROL_H
#define ALG_HEAT_CONTROL_H




typedef struct
{
    float maxium_temperature;//最大热量
    float current_temperature;//当前热量
    float hit_rate;//命中率
    float heat_recovery_rate;//热量恢复速率
    float maxium_fire_rate;//最大射速
    float heat_control_mdoe;//热量控制模式
} heat_control_t;






















#endif // ALG_HEAT_CONTROL_H