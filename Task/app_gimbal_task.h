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


//命令（包含 SentryMode_t 枚举与 mode 变量）
#include "app_command_task.h"


// 云台闭环反馈源选择
enum {
    IMU_MODE           = 0,  // IMU 绝对角度控制（跟随目标）
    ENCODER_MODE       = 1,  // 编码器角度锁定（保持当前位置）
    ENCODER_SCAN_MODE  = 2   // 编码器扫描模式（yaw在±PI范围往复扫描，pitch归零）
};



// 函数声明
void StartGimbalTask(void const * argument);

#endif // APP_GIMBAL_TASK_H


