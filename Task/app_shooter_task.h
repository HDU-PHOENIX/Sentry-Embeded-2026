#ifndef APP_SHOOTER_TASK_H
#define APP_SHOOTER_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "robot_config.h"

#include "app_command_task.h"
#include "app_gimbal_task.h"


#include "alg_fliter.h"

#include "bsp_log.h"
#include "bsp_can.h"


#include "dev_motor_dji.h"


#define MAX_TORQUE 30.0f //正常状态下允许的拨弹盘电机最大扭矩
#define FIRING_TORQUE 22.0f //不在开火状态下正常摩擦轮的最大扭矩
#define FIRE_ORIGIN 0.0f//拨弹盘原点位置
#define SHOOTER_RANGE 0.007//0.58f//一颗弹丸的位置
#define HEAT_MAX 260;
#define SHOOTER_WHEEL_SPEED -5500.0f//-2000.0f//-4000.0f;//射击轮目标速度
//状态枚举,
typedef enum{
  SHOOTER_TEST=9,
  SHOOTER_STOP=0,
  SHOOTER_STARTFIRE=1,
  SHOOTER_FIRING=2,
  SHOOTER_STUCK=3,
  SHOOTER_TRANS=4,
  SHOOTER_2HOT=5,
  SHOOTER_AUTO=6,
	SHOOTER_READY=7
} ShooterState_t;








#endif // APP_SHOOTER_TASK_H