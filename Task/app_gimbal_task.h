#ifndef APP_GIMBAL_TASK_H
#define APP_GIMBAL_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "main.h"


#include "robot_config.h"


//app层
#include "app_ins_task.h"
#include "app_command_task.h"
// alg层
#include "alg_fliter.h"
// BSP层
#include "bsp_log.h"
#include "bsp_dwt.h"
// module层

#include "dev_dr16.h"
#include "dev_motor_dm.h"
#include "dev_motor_dji.h"
#include "dev_board_communicate.h"


// 通信系统
#include "Com_System.h"


//命令
#include "app_command_task.h"


//宏定义
#define PC_MODE 1
#define RC_MODE 2
#define TRANS_MODE 3//用于失能，使能之间的过渡
#define UP_MODE 4//用于控制小云台
#define SHOOT_MODE 5//用于控制发射机构
#define SCROP_MODE 6 //用于小陀螺
#define DISABLE_MODE 0
#define TEST_MODE 9

enum {
    IMU_MODE=0,//使用IMU值进行控制
    ENCODER_MODE=1//使用编码器值进行控制
};



// 函数声明
void StartGimbalTask(void const * argument);

#endif // APP_GIMBAL_TASK_H


