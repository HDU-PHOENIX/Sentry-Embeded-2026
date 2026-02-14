/**
 * @file    alg_heat_control.c
 * @author  CGH
 * @brief   Heat control algorithm implementation
 * @version V1.0.0
 * @note 主要就两点，一个是根据拨弹盘最大输出来极限爆发，一个是根据热量冷却来控制持续射击
 */

#include "alg_heat_control.h"



//有待完善
uint8_t Heat_Caculate(heat_control_t *heat_control,uint16_t current_heat, float delta_time)
{
  uint16_t expect_damage=0;
  uint8_t result=0;
  return result;
}