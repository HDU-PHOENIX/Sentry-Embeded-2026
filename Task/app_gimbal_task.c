/**
 * @file app_gimbal_task.c
 * @author CGH
 * @brief 云台任务 —— 简化版：IMU 跟随 / 编码器锁定
 * @version 2.0
 * @note 移除复杂抬头/IMU切换，模式由 command_task 通过 raw combined 独立解析
 */
#include "app_gimbal_task.h"
#include "alg_ramp.h"
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"
#define DEBUG
#define IMU
//#define G_FEED_TEST

#define YAW_ORIGIN 0.0f//0.0f
#define COMPETITION_ENCODER_DEBOUNCE_TICKS 300U
#define COMPETITION_ENCODER_MIN_HOLD_MS 3000U

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

#ifdef GRAVITY_COMP_RECORD
#include "alg_ramp.h"
#define G_FEED_TEST
static GravityCompTargetGenerator_s g_gravity_gen;
#endif

#endif
uint8_t gimbal_mode=IMU_MODE;//云台控制模式

////////////////////////////编码器扫描模式相关///////////////////////////////////
#define SCAN_SPEED         1.5f    // 扫描角速度 (rad/s)
#define SCAN_RANGE_HALF    (PI/3.0f)  // 扫描范围半宽 (±60°)
#define PITCH_SCAN_SPEED       0.5f    // Pitch 扫描角速度 (rad/s)
#define PITCH_SCAN_LOWER_LIMIT -0.7f   // Pitch 限幅下界
#define PITCH_SCAN_UPPER_LIMIT 0.3f    // Pitch 限幅上界

static float   scan_target_yaw     = 0.0f;  // 当前扫描目标值
static int8_t  scan_direction      = 1;     // 扫描方向: 1=正向, -1=反向
static float   scan_target_pitch   = 0.0f;  // Pitch 当前扫描目标值
static int8_t  scan_pitch_dir      = 1;     // Pitch 扫描方向: 1=正向, -1=反向
static uint8_t scan_initialized    = 0;     // 扫描状态是否已初始化
////////////////////////////////////////////////////////////////////////////////

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
			
        .kp = 20.5f,
        .ki = 0.001f,
        .kd = 3.0f,
        .kf = 0.0f,
        .angle_max = 2.0f * PI,
        .i_max = 100.0,
        .out_max = 400.0,
    },
    .velocity_pid_config = {
        .kp = 0.8f,
        .ki = 0.008f,
        .kd = 0.0f,
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
.kp = 1.0f,                        // 位置环比例系数
        .ki = 0.001f,  
        //.ki = 0.05f,                      // 位置环积分系数
        .kd = 0.0f,                        // 位置环微分系数
        .kf = 0.0f,                        // 前馈系数
        .angle_max = 2.0f * PI,//0.0f,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0,                   // 积分限幅
        .out_max = 500.0,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
.kp = 3500.0f,//100.0f,                       // 速度环比例系数
        .ki = 20.0f,                        // 速度环积分系数
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
//[1.686418, -1.499381, -0.216508, 0.298176]（最新的）
   // float torque = 0.386597*position*position*position +  (-0.374320*position*position) + -0.419100*position + ( 0.075079);
   float torque = (1.686418 * position * position * position) + (-1.499381 * position * position) + (-0.216508 * position) + (0.298176);
   return torque; 
}

static uint8_t competition_encoder_mode_initialized = 0;
static uint8_t competition_encoder_mode_last_raw = IMU_MODE;
static uint8_t competition_encoder_mode_filtered = IMU_MODE;
static uint16_t competition_encoder_mode_same_cnt = 0;
static uint32_t competition_encoder_mode_last_switch_tick = 0;

static void ResetCompetitionEncoderModeFilter(void)
{
    competition_encoder_mode_initialized = 0;
    competition_encoder_mode_last_raw = IMU_MODE;
    competition_encoder_mode_filtered = IMU_MODE;
    competition_encoder_mode_same_cnt = 0;
    competition_encoder_mode_last_switch_tick = 0;
}

static uint8_t UpdateCompetitionEncoderMode(uint8_t shoot_bool)
{
    uint32_t now_tick = osKernelSysTick();
    uint8_t requested_mode = (shoot_bool == 0U) ? ENCODER_MODE : IMU_MODE;

    if (!competition_encoder_mode_initialized) {
        competition_encoder_mode_initialized = 1;
        competition_encoder_mode_last_raw = IMU_MODE;
        competition_encoder_mode_filtered = IMU_MODE;
        competition_encoder_mode_same_cnt = 0;
        competition_encoder_mode_last_switch_tick = now_tick;
    }

    if (requested_mode == IMU_MODE) {
        if (competition_encoder_mode_filtered != IMU_MODE) {
            competition_encoder_mode_filtered = IMU_MODE;
            competition_encoder_mode_last_switch_tick = now_tick;
        }
        competition_encoder_mode_last_raw = IMU_MODE;
        competition_encoder_mode_same_cnt = 0;
    } else {
        if (competition_encoder_mode_last_raw == ENCODER_MODE) {
            if (competition_encoder_mode_same_cnt < COMPETITION_ENCODER_DEBOUNCE_TICKS) {
                competition_encoder_mode_same_cnt++;
            }
        } else {
            competition_encoder_mode_same_cnt = 1;
        }

        if (competition_encoder_mode_filtered != ENCODER_MODE
            && competition_encoder_mode_same_cnt >= COMPETITION_ENCODER_DEBOUNCE_TICKS
            && (now_tick - competition_encoder_mode_last_switch_tick) >= COMPETITION_ENCODER_MIN_HOLD_MS) {
            competition_encoder_mode_filtered = ENCODER_MODE;
            competition_encoder_mode_last_switch_tick = now_tick;
        }

        competition_encoder_mode_last_raw = ENCODER_MODE;
    }

    return competition_encoder_mode_filtered;
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
    
    Up_yaw->velocity_pid->kp=Up_config.velocity_pid_config.kp;
		Up_yaw->velocity_pid->ki=Up_config.velocity_pid_config.ki;
		Up_yaw->control_mode=DJI_VELOCITY;
    Up_yaw->angle_pid->i_out=0.0f;
    Up_yaw->velocity_pid->i_out=0.0f;
    Up_yaw->angle_pid->p_out=0.0f;
    Up_yaw->velocity_pid->p_out=0.0f;
    gimbal_ready_flag=1;

#ifdef GRAVITY_COMP_RECORD
    /* 初始化重力补偿标定用斜坡生成器：
     *   范围 [-0.7, 0.3] rad（对应 pitch 限幅边界）
     *   单程 60 步 × 800ms = 48s 单向，端点暂停 3s
     *   完整一轮 ~102s
     */
    GravityCompTargetGenerator_Init(&g_gravity_gen,
                             0.0f, 1.0f,
                             80,      // 步数，范围1.0rad分80步，每步约不到1°
                             800,     // 800ms/步
                             3000);   // 端点暂停 3s
    
#endif

    Log("Gimbal ready\r\n");

        static uint8_t last_gimbal_mode = IMU_MODE;
    	static uint8_t send_flag = 0;
  for(;;)
  {
        #ifdef DEBUG
        can_count=board_instance->can_instance->cnt;
        test_speed=Up_yaw->message.out_velocity;
         dt3 = Dwt_GetDeltaT(&dwt_cnt_last3);
        if (dt3 > 0.01f || dt3 <= 0.0f) { dt3 = 0.001f; }
        #endif
        find_bool = board_instance->received_shoot_bool;

        // 把发送频率降低一点
        if(send_flag==0){
            send_flag=1;
            board_send_message(board_instance,Quater.yaw, Quater.pitch, find_bool, Quater.ins_ready);
        }else{
            send_flag=0;
        }
        
        ControlMode = mode;
        
        // 根据 shoot_bool 切换云台控制模式
        //   shoot_bool == 0 → IMU 模式（下板目标跟随）
        //   shoot_bool == 1 → 编码器扫描模式（yaw 在 ±PI 范围往复，pitch 归零）
        gimbal_mode = (find_bool == 0U) ? IMU_MODE : ENCODER_SCAN_MODE;

        if (gimbal_mode == ENCODER_SCAN_MODE) {
            /* ---- 编码器扫描模式（固定 ±60° 绝对值范围） ---- */
            // 首次进入或从其他模式切换来时，从当前位置出发，无跳变
            if (!scan_initialized || last_gimbal_mode != ENCODER_SCAN_MODE) {
                scan_target_yaw = Up_yaw->message.out_position;
                // 决定初始方向：若在范围外则指向最近边界，否则默认正向
                if (scan_target_yaw > SCAN_RANGE_HALF) {
                    scan_direction = -1;   // 高于上界，向下进入范围
                } else if (scan_target_yaw < -SCAN_RANGE_HALF) {
                    scan_direction = 1;    // 低于下界，向上进入范围
                } else {
                    scan_direction = 1;    // 在范围内，默认正向开始
                }
                
                scan_target_pitch = Quater.pitch;
                // Pitch 决定初始方向
                if (scan_target_pitch > PITCH_SCAN_UPPER_LIMIT) {
                    scan_pitch_dir = -1;
                } else if (scan_target_pitch < PITCH_SCAN_LOWER_LIMIT) {
                    scan_pitch_dir = 1;
                } else {
                    scan_pitch_dir = 1;
                }
                
                scan_initialized = 1;
            }

            // 三角波扫描：边界固定在 ±SCAN_RANGE_HALF（绝对值 ±60°）
            scan_target_yaw += scan_direction * SCAN_SPEED * dt3;
            if (scan_target_yaw >= SCAN_RANGE_HALF) {
                scan_target_yaw = SCAN_RANGE_HALF;
                scan_direction = -1;
            } else if (scan_target_yaw <= -SCAN_RANGE_HALF) {
                scan_target_yaw = -SCAN_RANGE_HALF;
                scan_direction = 1;
            }
            
            // Pitch 三角波扫描：边界在 -0.7 到 0.3
            scan_target_pitch += scan_pitch_dir * PITCH_SCAN_SPEED * dt3;
            if (scan_target_pitch >= PITCH_SCAN_UPPER_LIMIT) {
                scan_target_pitch = PITCH_SCAN_UPPER_LIMIT;
                scan_pitch_dir = -1;
            } else if (scan_target_pitch <= PITCH_SCAN_LOWER_LIMIT) {
                scan_target_pitch = PITCH_SCAN_LOWER_LIMIT;
                scan_pitch_dir = 1;
            }

            // 扫描模式下忽略下板目标角度，pitch 和 yaw 同样进入扫描
            target_up_position = scan_target_yaw;
#ifndef GRAVITY_COMP_RECORD
            target_position = scan_target_pitch;
#endif
        } else {
            /* ---- IMU 模式（下板目标跟随） ---- */
            target_up_position = board_instance->received_target_up_yaw;
#ifndef GRAVITY_COMP_RECORD
            target_position = board_instance->received_target_up_pitch;
#endif
        }

        switch (ControlMode) {
        /* ---- 云台使能模式：所有非失能模式均使能云台 ---- */
        case UP_FOLLOW_MODE:
        case UP_SHOOT_MODE:
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

#ifndef GRAVITY_COMP_RECORD
            // Pitch轴限幅
            if(pitch->control_mode==DM_POSITION){
                target_position=target_position>0.3?0.3:target_position;
                target_position=target_position<-0.7?-0.7:target_position;
            }
#endif
#ifdef GRAVITY_COMP_RECORD
            if(Quater.ins_ready==1){
                /* 重力补偿标定模式：用极慢的斜坡驱动 pitch */
                target_position = GravityCompTargetGenerator_Update(&g_gravity_gen);
                if (g_gravity_gen.done) {
                    Log("Gravity comp sweep DONE\r\n");
                }
                Motor_Dm_Pos_Vel_Control(pitch, target_position,10);
                // 注意：由于下面已经有统一的 Motor_Dm_Transmit(pitch); 这里不要再额外发送，
                // 更不要误写成 Motor_Dji_Transmit(pitch); 这会触发指针转型HardFault！
                // target_speed = Pid_Calculate(pitch->angle_pid, target_position, Quater.pitch);
                // ...
            }
#else
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
#endif
            #ifdef DEBUG
            test_output=pitch->message.torque;
            #endif
            Motor_Dm_Transmit(pitch);

            // Yaw轴
            //复合语句块，又一个冷门语法，用来局部定义变量，避免在其他模式下占用资源
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
        /* ---- 仅 DISABLE_MODE 失能云台 ---- */
        case DISABLE_MODE:
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
