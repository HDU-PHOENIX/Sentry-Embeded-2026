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

#define PC_MODE 1
#define RC_MODE 2
#define TRANS_MODE 3//用于失能，使能之间的过渡
#define UP_MODE 4//用于控制小云台
#define SHOOT_MODE 5//用于控制发射机构
#define SCROP_MODE 6 //用于小陀螺
#define DISABLE_MODE 0
#define TEST_MODE 9//测试用模式，使其可以脱离遥控器/其他外部控制信号来进行调试

extern board_instance_t *board_instance;

#endif // APP_COMMAND_TASK_H
