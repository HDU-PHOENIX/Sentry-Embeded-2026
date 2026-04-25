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
    UP_LOCK_MODE        = 5,  // 上云台锁定
    CHASSIS_GYRO_MODE   = 6,  // 底盘小陀螺
    CHASSIS_FOLLOW_MODE = 7,  // 底盘跟随云台
    CHASSIS_LOCK_MODE   = 8,  // 底盘锁定
} SentryMode_t;

extern board_instance_t *board_instance;

#endif // APP_COMMAND_TASK_H
