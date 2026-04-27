/**
 * @file app_gimbal_task.c
 * @author CGH
 * @brief 云台任务 —— 简化版：IMU 跟随 / 编码器锁定
 * @version 2.0
 * @note 移除复杂抬头/IMU切换，模式由 command_task 通过 raw combined 独立解析
 */
#include "app_gimbal_task.h"
#define DEBUG
#define IMU
//#define G_FEED_TEST

#define YAW_ORIGIN 0.0f

//实例声明
DmMotorInstance_s *pitch;
DjiMotorInstance_s *Up_yaw;
extern board_instance_t *board_instance;

//变量声明
uint8_t find_bool=0;
#ifdef DEBUG    
uint32_t can_count=0;

#endif

#ifndef DEBUG
uint8_t ControlMode=DISABLE_MODE;
float target_position=0.0;//后续改为上位机提供
float target_up_position=0.0;
#else
float dt3 = 0.001f;  // 初始dt
extern uint8_t mode;
uint8_t ControlMode= DISABLE_MODE;
uint8_t gimbal_ready_flag;

float target_position=0.0,test_speed=0.0,test_position=0.0,target_speed=0.0,test_output=0.0,test_g_out=0.0;
float target_up_speed=0.0;//yaw目标速度（用于IMU下）
float target_up_position=0.0;
float target_pitch_position=0.0;
float temp_position=0.0;
float output=0;
extern quaternions_struct_t Quater;

#endif
uint8_t gimbal_mode=IMU_MODE;//云台控制模式

////////////////////////////电机配置/////////////////////////////////////////


static DmMotorInitConfig_s pitch_config = {
     //.control_mode = DM_VELOCITY,    
.control_mode = DM_POSITION, // 位置控制模式
		.topic_name = "pitch",
    .can_config = {
        .can_number = 2,
				.topic_name = "pitch",
            #ifndef G_FEED_TEST
          .tx_id = 0x006,
          #else
         .tx_id = 0x106,
         #endif
         .rx_id = 0x016,
         
        .can_module_callback = NULL,
    },
		.parameters = {
.pos_max = 6.2831853,    // [查手册] 电机最大位置范围(弧度)
        .vel_max = 30.0f,    // [查手册] 电机最大速度范围 (弧度/秒)
        .tor_max = 18.0f,    // [查手册] 电机最大扭矩范围 (N·m)
        .kp_max  = 500.0f,   // [查手册] Kp增益最大值
        .kd_max  = 5.0f,     // [查手册] Kd增益最大值
        .kp_int  = 0.0f,  // [调试设定] 要发送给电机的kp值( MIT模式)
        .kd_int  = 0.0f,     // [调试设定] 要发送给电机的kd值( MIT模式)
    },
		#ifdef IMU
		.angle_pid_config = {
			
        .kp = 20.0f,
        .ki = 0.0015f,
        .kd = 0.5f,
        .kf = 0.0f,
        .angle_max = 2.0f * PI,
        .i_max = 100.0,
        .out_max = 400.0,
    },
    .velocity_pid_config = {
        .kp = 0.75f,
        .ki = 0.00f,
        .kd = 0.06f,
        .kf = 0.0f,
        .angle_max = 0,
        .i_max = 5.0,
        .out_max = 10.0,//待商榷
    }
		#else
    .angle_pid_config = {
			
        .kp = 17,
        .ki = 0.0015,
        .kd = 0.06,
        .kf = 0.0,
        .angle_max = 2.0f * PI,
        .i_max = 100.0,
        .out_max = 400.0,
    },
    .velocity_pid_config = {
        .kp = 0.53,
        .ki = 0.009,
        .kd = 0.017,
        .kf = 0.0,
        .angle_max = 0,
        .i_max = 500.0,
        .out_max = 2000.0,//待商榷
    }
		#endif
};
static  DjiMotorInitConfig_s Up_config = {
    .id = 1 ,                      // 电机ID(1~4)
    .type = GM6020,               // 电机类型
   // .control_mode = DJI_POSITION,  // 电机控制模式
	.control_mode = DJI_VELOCITY,
		.topic_name = "up_yaw",
    .can_config = {
        .can_number = 2,
				.topic_name = "up_yaw",              // can句柄
        .tx_id = 0x1FF,                     // 发送id 
        .rx_id = 0x205,                     // 接收id
    },
    .reduction_ratio = 1,              // 减速比

    .angle_pid_config = {
        //.kp = 0.0f,
.kp = 22.0f,                        // 位置环比例系数
        .ki = 0.0f,  
        //.ki = 0.05f,                      // 位置环积分系数
        .kd = 0.0f,                        // 位置环微分系数
        .kf = 0.0f,                        // 前馈系数
        .angle_max = 2.0f * PI,//0.0f,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0,                   // 积分限幅
        .out_max = 500.0,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
.kp = 4500.0f,//100.0f,                       // 速度环比例系数
        .ki = 0.0f,                        // 速度环积分系数
        .kd = 0.0f,                        // 速度环微分系数
        .kf = 0.0f,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 6000.0,                  // 积分限幅
        .out_max = 25000,                // 输出限幅(电流输出)
    }
};

//限幅0.17-0
//速度：0.8，0.001，角度：10
/**
* @brief 重力补偿代码
* @param position 电机当前位置
*	@return float 输出力矩
*/
float G_feed(float position){
    //[354.533813, -99.411941, 7.604260, -0.441783]
    // float torque = -21.321498 * sin(position + -1.625094) + -21.348101;
	//[ 1.34165963 -1.72110943  0.29976696 -0.17707916]
	//[2.319374, -3.015623, 0.755581, -0.311670]
	//[1.549729, -2.003787, 0.406681, -0.285731]
//[0.386597, -0.374320, -0.419100, 0.075079]（装了枪管的）
   // float torque = 0.386597*position*position*position +  (-0.374320*position*position) + -0.419100*position + ( 0.075079);
   float torque = (1.016980 * position * position * position) + (-1.500239 * position * position) + (-0.062756 * position) + (0.195295); 
   return torque; 
}


void StartGimbalTask(void const * argument)
{
	#ifdef DEBUG
     uint32_t dwt_cnt_last3 = 0;
     
     static float lasttime3 = 0;
    
    // 初始化DWT计数器
    dwt_cnt_last3 = DWT->CYCCNT;
		#endif
	
    pitch=Motor_DM_Register(&pitch_config);//4310
    Up_yaw=Motor_Dji_Register(&Up_config);//6020
	
    
		 if(Up_yaw==NULL)
    {
        Log_Error("Up_yaw motor register failed\r\n");
        
    }

	   while (pitch->motor_state!=DM_ENABLE)
    {
        Motor_Dm_Cmd(pitch,DM_CMD_MOTOR_ENABLE);
        Motor_Dm_Transmit(pitch);
        osDelay(1);
    }
    Log_Information("pitch motor enable success\r\n");
    while(Quater.ins_ready==0){
        //视情况要不要启用编码器控制
        // Motor_Dm_Control(pitch,target_position);
        // output=pitch->output+G_feed(pitch->message.out_position);
				
        // Motor_Dm_Mit_Control(pitch,0,0,output);
        // Motor_Dm_Transmit(pitch);
			Up_yaw->control_mode=DJI_POSITION;
			Up_yaw->angle_pid->ki=0.05f;
        Up_yaw->velocity_pid->kp=70.0f;
			Up_yaw->velocity_pid->ki=0.0f;
			
        Motor_Dji_Control(Up_yaw,YAW_ORIGIN);
        Motor_Dji_Transmit(Up_yaw);
        if(fabsf(Up_yaw->message.out_position-YAW_ORIGIN)<0.05f){
            gimbal_ready_flag=1;
            break;
        }
        #ifdef G_FEED_TEST
        //gimbal_ready_flag=1;
        #endif
        
        osDelay(1);
    }
    //零点刚刚初始化的时候做个差，得出上下云台零点之间的偏移
		Up_yaw->angle_pid->ki=0.0f;
		osDelay(4);
		if(Quater.ins_ready!=1){
			Log("Erro!Offset may not correct.");
		}
		while(Quater.ins_ready!=1){
			osDelay(1);
		}
    
    Up_yaw->velocity_pid->kp=2300.0f;
		Up_yaw->control_mode=DJI_VELOCITY;
    Up_yaw->angle_pid->i_out=0.0f;
    Up_yaw->velocity_pid->i_out=0.0f;
    Up_yaw->angle_pid->p_out=0.0f;
    Up_yaw->velocity_pid->p_out=0.0f;
    gimbal_ready_flag=1;

    Log("Gimbal ready\r\n");

    static uint8_t last_gimbal_mode = IMU_MODE;
    static float encoder_lock_yaw_target = 0.0f;
    static uint8_t encoder_lock_initialized = 0;
  for(;;)
  {
        #ifdef DEBUG
        can_count=board_instance->can_instance->cnt;
        test_speed=Up_yaw->message.out_velocity;
        static uint8_t send_flag=0;
         dt3 = Dwt_GetDeltaT(&dwt_cnt_last3);
        if (dt3 > 0.01f || dt3 <= 0.0f) { dt3 = 0.001f; }
        #endif
        ControlMode=board_instance->received_control_mode;
        find_bool = board_instance->received_shoot_bool;

        // 把发送频率降低一点
        if(send_flag==0){
            send_flag=1;
            board_send_message(board_instance,Quater.yaw, Quater.pitch, find_bool, Quater.ins_ready);
        }else{
            send_flag=0;
        }
        ControlMode=mode;
        // ---- 根据 SentryMode_t 决定 gimbal_mode ---- 
        // UP_LOCK_MODE: 编码器锁定当前位置
        // PC_MODE / UP_FOLLOW_MODE / UP_SHOOT_MODE: IMU 跟随（目标由下板下发)
        if (ControlMode == UP_LOCK_MODE) {
            if (!encoder_lock_initialized) {
                encoder_lock_yaw_target = Up_yaw->message.out_position;
                encoder_lock_initialized = 1;
            }
            gimbal_mode = ENCODER_MODE;
            target_up_position = encoder_lock_yaw_target;
            target_position = Quater.pitch; // pitch 也锁定当前位置
        } else {
            encoder_lock_initialized = 0;
            gimbal_mode = IMU_MODE;
            target_up_position = board_instance->received_target_up_yaw;
            target_position = board_instance->received_target_up_pitch;
        }

        switch (ControlMode) {
        case PC_MODE:
        case UP_FOLLOW_MODE:
        case UP_SHOOT_MODE:
        case UP_LOCK_MODE:
            if(pitch->motor_state==DM_DISABLE||Up_yaw->velocity_pid->is_enabled==0){
                ControlMode=TRANS_MODE;
            }
            Motor_Dm_Cmd(pitch, DM_CMD_MOTOR_ENABLE);
            Motor_Dm_Transmit(pitch);
            Pid_Enable(Up_yaw->angle_pid);
            Pid_Enable(Up_yaw->velocity_pid);
            Pid_Enable(pitch->angle_pid);
            Pid_Enable(pitch->velocity_pid);
            // 模式切换时清除PID中间值
            if (gimbal_mode != last_gimbal_mode) {
                Pid_Clear(Up_yaw->angle_pid);
                Pid_Clear(Up_yaw->velocity_pid);
                last_gimbal_mode = gimbal_mode;
            }

            // Pitch轴限幅
            if(pitch->control_mode==DM_POSITION){
                target_position=target_position>0.3?0.3:target_position;
                target_position=target_position<-0.7?-0.7:target_position;
            }
            if(Quater.ins_ready==1){
                target_speed=Pid_Calculate(pitch->angle_pid,target_position,Quater.pitch);
                pitch->output = Pid_Calculate(pitch->velocity_pid,Quater.Gyro[1],target_speed);
            }else{
                pitch->angle_pid->i_out=0.0;
                pitch->velocity_pid->i_out=0.0;
                pitch->output=0;
            }
            output=pitch->output+G_feed(pitch->message.out_position);
            Motor_Dm_Mit_Control(pitch,0,0,output);
            #ifdef DEBUG
            test_output=pitch->message.torque;
            #endif
            Motor_Dm_Transmit(pitch);

            // Yaw轴
            {
                float yaw_angle_feedback;
                if(gimbal_mode==IMU_MODE){
                    yaw_angle_feedback = Quater.yaw;
                }else{
                    yaw_angle_feedback = Up_yaw->message.out_position;
                }
                target_up_speed = Pid_Calculate(Up_yaw->angle_pid, target_up_position, yaw_angle_feedback);
                Up_yaw->output = Pid_Calculate(Up_yaw->velocity_pid, target_up_speed, Quater.Gyro[2]);
            }
            Motor_Dji_Transmit(Up_yaw);
            break;
        case DISABLE_MODE:
        case CHASSIS_GYRO_MODE:
        case CHASSIS_FOLLOW_MODE:
        case CHASSIS_LOCK_MODE:
            Pid_Disable(Up_yaw->velocity_pid);
            Pid_Disable(Up_yaw->angle_pid);
            Pid_Disable(pitch->velocity_pid);
            Pid_Disable(pitch->angle_pid);
            Motor_Dm_Cmd(pitch,DM_CMD_MOTOR_DISABLE);
            Motor_Dm_Transmit(pitch);
            Motor_Dji_Control(Up_yaw,target_position);
            Motor_Dji_Transmit(Up_yaw);
            break;
        case TRANS_MODE:
            Motor_Dm_Cmd(pitch, DM_CMD_MOTOR_ENABLE);
            Motor_Dm_Transmit(pitch);
            Pid_Enable(Up_yaw->angle_pid);
            Pid_Enable(Up_yaw->velocity_pid);
            Pid_Enable(pitch->angle_pid);
            Pid_Enable(pitch->velocity_pid);
            break;
        default:
            Pid_Disable(Up_yaw->velocity_pid);
            Pid_Disable(Up_yaw->angle_pid);
            Pid_Disable(pitch->velocity_pid);
            Pid_Disable(pitch->angle_pid);
            Motor_Dm_Cmd(pitch,DM_CMD_MOTOR_DISABLE);
            Motor_Dm_Transmit(pitch);
            Motor_Dji_Control(Up_yaw,target_position);
            break;
        }
    osDelay(1);
  }
}
