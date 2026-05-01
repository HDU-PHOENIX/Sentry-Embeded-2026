#ifndef APP_COMMAND_TASK_H
#define APP_COMMAND_TASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "app_shooter_task.h"

#include "dev_dr16.h"
#include "dev_minipc.h"
#include "Com_System.h"
#include "dev_board_communicate.h"

#include "bsp_log.h"

/* ================================================================ */
/* 统一模式枚举（与下板 Clean_Down 保持一致）                          */
/* ================================================================ */
typedef enum {
    DISABLE_MODE        = 0,  // 失能
    PC_MODE             = 1,  // 上位机模式
    TRANS_MODE          = 2,  // 使能/失能过渡
    UP_FOLLOW_MODE      = 3,  // 上云台跟随
    UP_SHOOT_MODE       = 4,  // 上云台射击
    UP_SCORPE_MODE      = 5,  // 保留
    CHASSIS_GYRO_MODE   = 6,  // 底盘小陀螺
    CHASSIS_FOLLOW_MODE = 7,  // 底盘跟随云台
    CHASSIS_LOCK_MODE   = 8,  // 底盘锁定
    HANDLE_MODE         = 9,  // 手动模式
    //手动模式下的子类
    UP_FOLLOW_PC_MODE      = 10, // 上云台跟随 + PC 模式
    UP_SHOOT_PC_MODE       = 11, // 上云台射击 + PC 模式
    UP_SHOOT_AUTO_MODE     = 12, // 上云台射击 + 自动模式
    UP_SHOOT_SINGLE_MODE   = 13, // 上云台射击 + 单发模式
    //导航模式下的子类
    CHASSIS_FOLLOW_PC_MODE   = 14, // 底盘跟随云台 + PC 模式
    CHASSIS_AUTOAIM_PC_MODE  = 15, // 底盘跟随云台 + PC 模式 + 自动瞄准
    CHASSIS_COMPETATION_MODE = 16, // 竞赛模式
} SentryMode_t;

extern board_instance_t *board_instance;

#endif // APP_COMMAND_TASK_H
