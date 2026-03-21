/**
 * 
 * @file app_chassis_task.c
 * @author CGH
 * @brief 底盘控制任务
 * @version V1.0.0
 */
#include "app_chassis_task.h"
#include "alg_chassis_calc.h"
#include "alg_pid.h"
#include "alg_fliter.h"
#include "app_command_task.h"
#include "app_ins_task.h"
#include "dev_dr16.h"
#include "dev_motor_dji.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
//宏定义
#define DEBUG
//#define SHOOT_DEBUG

//实例声明
ChassisInstance_s *Chassis;
DmMotorInstance_s *Down_yaw;
extern BuzzerInstance_s *buzzer;
DjiMotorInstance_s *Trigger;
DjiMotorInstance_s *Up_yaw;
Subscriber *CH_Subs;
Dr16Instance_s* CH_Receive_s;
MiniPC_Instance *MiniPC;
MiniPC_Instance *MiniPC_SelfAim;
MiniPC_Instance *MiniPC_ExpAim;
board_instance_t *board_instance;
gimbal_follow_instance_s* GimbalFollow_Instance;
LowpassFilter_t *Trigger_In_LPF;
LowpassFilter_t *Trigger_Out_LPF;
//extern QEKF_INS_t QEKF_INS; 
uint8_t enemy_color =1;//暂时的逻辑
uint8_t shoot_bool=0;
uint8_t find_bool=0;
//变量声明
#ifndef DEBUG
uint8_t controlmode=DISABLE_MODE;
float target_position=0.0;//后续改为上位机提供
float target_up_position=0.0;
float target_up_pitch=0.0;
#else
extern volatile uint8_t mode;
extern volatile uint8_t combined_state_global;
float target_position=0.0f,test_speed=0.0,test_position=0.0,target_speed=0.0,test_output=0.0;
float target_tr=40.0;
float test_vel_tr=0.0,test_output_tr=0.0,test_pos_tr=0.0f;
uint16_t lasttime=0;
float speed1=0.0,speed2=0.0,speed3=0.0,speed4=0.0;
float target1=0.0,target2=0.0,target3=0.0,target4=0.0;
float target_up_position=0.0f;//暂时的逻辑，一定要记得改回来！！！！！(又记，可能是改回来了吧)
float target_up_pitch=0.0f;
#endif
uint8_t rune_flag=0;//打符开关
uint8_t minipc_mode=0;//0自瞄，1打符
uint8_t control_mode=RC_MODE;//默认遥控器模式
uint16_t max_torque=7000;

#define TRIGGER_STEP_ANGLE (0.7f * BULLET_ANGLE)
#define TRIGGER_FIRE_PERIOD_S 0.033f
#define TRIGGER_JAM_REBOUND_ANGLE (2.0f * BULLET_ANGLE)
#define TRIGGER_JAM_REBOUND_DURATION_MS 1000  // 退弹持续时间 (ms)
#define TRIGGER_JAM_TORQUE_THRESHOLD 9000     // 判定卡弹的转矩电流阈值
#define TRIGGER_JAM_REBOUND_ENABLE 1          // 启用卡弹退弹逻辑

typedef struct {
  uint32_t jam_start_tick;   // 卡弹起始时间
  uint32_t rebound_start_tick; // 开始退弹的时间
  bool is_rebounding;        // 是否正在退弹
} TriggerJamState_s;

static TriggerJamState_s trigger_jam = {0};

static float WrapAnglePi(float angle) {
  angle = fmodf(angle + PI, 2.0f * PI);
  if (angle <= 0.0f) {
    angle += 2.0f * PI;
  }
  return angle - PI;
}

static void Trigger_ResetState(void) {
  memset(&trigger_jam, 0, sizeof(trigger_jam));
}
//配置
static ChassisInitConfig_s Chassis_config={
		.type = Omni_Wheel,
		.gimbal_yaw_zero = 2.58681059,//-0.027132988,//2.00935459,//-1.08712959,//0.66278553, //0.0f,////0.641885281,//-2.62492895,//(-10663.0f / 262144.0f) * 2.0f * 3.141593f
		//.gimbal_yaw_half = 0.130077288,//(251481.0f / 262144.0f) * 2.0f * 3.141593f
		.omni_steering_message={
		.wheel_radius= 0.0765f,
	  .chassis_radius= 0.26176f,
		},
    .Gyroscope_Speed = 0.4f,  // 设置小陀螺旋转速度 (rad/s)
		.gimbal_follow_pid_config={
		  .kp = 4.5f,
      .ki = 0.0f,
      .kd = 0.0f,
      .angle_max = 2.0f * PI,
			.dead_zone = 0.15f,
      .i_max = 0.0f,
      .out_max = 2 * 3.141593f,
		},

    .power_control_config = {
      .enabled = false,
      .power_buffer_target = 30.0f,
      .steering_power_ratio = 0.0f,

      .wheel_group = {
          .method = CHASSIS_POWER_CONTROL_METHOD_CURRENT_ATTENUATION,
          .motor_count = 4,
          .model = {
              .k0 = 0.66419934f,
              .k1 = 0.00644428f,
              .k2 = 0.00014239f,
              .k3 = 0.01764443f,
              .k4 = 0.16501439f,
              .k5 = 0.00003097f,
          },
      },

      .steering_group = {
          .method = CHASSIS_POWER_CONTROL_METHOD_DISABLED,
          .motor_count = 0,
      },
    },
		.motor_config[0]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 1,
		.topic_name = "ch1",
    .can_config = {
            .can_number = 1,
			.topic_name = "ch1",
      .tx_id = 0x200,
      .rx_id = 0x201,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 100.0f,
      .ki = 15.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  },
		.motor_config[1]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 2,
		.topic_name = "ch2",
    .can_config = {
      .can_number = 1,
			.topic_name = "ch2",
      .tx_id = 0x200,
      .rx_id = 0x202,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  },.motor_config[2]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 3,
		.topic_name = "ch3",
    .can_config = {
      .can_number = 1,
			.topic_name = "ch3",
      .tx_id = 0x200,
      .rx_id = 0x203,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 100.0f,
      .ki = 15.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  },
	.motor_config[3]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 4,
		.topic_name = "ch4",
    .can_config = {
      .can_number = 1,
			.topic_name = "ch4",
      .tx_id = 0x200,
      .rx_id = 0x204,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  }
	};

//上云台yaw电机配置(用于读取编码器角度)
static DjiMotorInitConfig_s Up_config = {
    .id = 1,                      // 电机ID(1~4)
    .type = GM6020,               // 电机类型
     .control_mode = DJI_POSITION,  // 电机控制模式
    .topic_name = "up_yaw",
    .can_config = {
        .can_number = 2,
        .topic_name = "up_yaw",              // can句柄
        .tx_id = 0x1FF,                     // 发送id 
        .rx_id = 0x205,                     // 接收id
        .can_module_callback=NULL,
    },
    .reduction_ratio = 1.0f,              // 减速比

    .angle_pid_config = {
        .kp = 0.0f,                        // 位置环比例系数
        .ki = 0.0f,                        // 位置环积分系数
        .kd = 0.0f,                        // 位置环微分系数
        .kf = 0.0f,                        // 前馈系数
        .angle_max =2*PI,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0f,                   // 积分限幅
        .out_max = 400.0f,                 // 输出限幅(速度环输入)
    }, 
    .velocity_pid_config = {
        .kp = 0.0f,                       // 速度环比例系数
        .ki = 0.0f,                        // 速度环积分系数
        .kd = 0.0f,                        // 速度环微分系数
        .kf = 0.0f,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 1000.0f,                  // 积分限幅
        .out_max = 2000.0f,                // 输出限幅(电流输出)
    }
};
	
//底盘大yaw电机配置
	static DmMotorInitConfig_s Down_config = {
   //.control_mode = DM_VELOCITY,     
	 .control_mode = DM_POSITION,
		.topic_name = "down_yaw",
    .can_config = {
        .can_number = 1,
				.topic_name = "down_yaw",
        .tx_id = 0x009,
        .rx_id = 0x019,
        .can_module_callback = NULL,
    },
		.parameters = {
        .pos_max = 6.2831853f,    // [查手册] 电机最大位置范围 (弧度)
        .vel_max = 45.0f,    // [查手册] 电机最大速度范围 (弧度/秒)
        .tor_max = 18.0f,    // [查手册] 电机最大扭矩范围 (N·m)
        .kp_max  = 500.0f,   // [查手册] Kp增益最大值
        .kd_max  = 5.0f,     // [查手册] Kd增益最大值
        .kp_int  = 0.0f,  // [调试设定] 要发送给电机的Kp值 (仅MIT模式)
        .kd_int  = 0.0f,     // [调试设定] 要发送给电机的Kd值 (仅MIT模式)
    },
    .angle_pid_config = {
        .kp = 9.0f,//1.0f,//8.0f,
        .ki = 0.0f,
        .dead_zone = 0.003f,
        .kd = 0.0f,
        .kf = 0.0f,
        .angle_max = 2.0f * PI,
        .i_max = 100.0,
        .out_max = 400.0,
    }, 
//    .velocity_pid_config = {
//        .kp = 1.0f,
//        .ki = 0.0005f,
//        .kd = 0.0f,
//        .kf = 0.0f,
//        .angle_max = 0,
//        .i_max = 1000.0,
//        .out_max = 2000.0,
//    }
		 .velocity_pid_config = {
        .kp = 2.0f,
        .ki = 0.04f,
        .kd = 0.5f,
        .kf = 0.0f,
        .angle_max = 0,
        .i_max = 5.0,
        .out_max = 10.0,
		 }
};
//拨弹盘配置
static  DjiMotorInitConfig_s Trigger_Config = {
    .id = 2,                      // 电机ID(1~4)
    .type = M2006,               // 电机类型
  .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "Trigger",
    .can_config = {
        .can_number = 2,//记得改回来
				.topic_name = "Trigger",              // can句柄
        .tx_id = 0x200,                     // 发送id 
        .rx_id = 0x202,                     // 接收id
			  .can_module_callback=NULL,
    },
    .reduction_ratio = (36.0/19.0)*47.0,              // 减速比

    .angle_pid_config = {
        .kp = 35.0f,                        // 位置环比例系数
        .ki = 0.0f,                        // 位置环积分系数
        .kd = 0.0f,                        // 位置环微分系数
        .kf = 0.0f,                        // 前馈系数
        .angle_max =2*PI,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0f,                   // 积分限幅
        .out_max = 4000.0f,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
        .kp = 120.0f,                       // 速度环比例系数
        .ki = 0.00f,                        // 速度环积分系数
        .kd = 0.0f,                        // 速度环微分系数
        .kf = 0.0f,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 500.0f,                  // 积分限幅
        .out_max = 8000.0f,                // 输出限幅(电流输出)
    }
};



MiniPC_Config miniPC_config = {
    .callback = NULL,
    .message_type = USB_MSG_CHASSIS_RX, // 底盘数据
    .Send_message_type = USB_MSG_AIM_TX // 发送数据类型
};

MiniPC_Config SelfAim_config = {
    .callback = NULL,
    .message_type = USB_MSG_AIM_RX, // 底盘数据
    .Send_message_type = USB_MSG_FRIEND1_TX // 发送数据类型
};
MiniPC_Config ExpAim_config={
	  .callback = NULL,
	  .message_type = USB_MSG_EXP_AIM_RX,
		.Send_message_type = USB_MSG_EXP_AIM_TX

};

board_config_t board_config = {
    .board_id = 1,
    .can_config = {
        .can_number = 2,
        .topic_name = "Board_Comm"
        
        
    },
    .message_type = DOWN2UP_MESSAGE_TYPE, // down2up_message_t
};

gimbal_follow_config_s GimbalFollow_config = {
    .up_origin = 0.0f, // 云台偏航零点角度
    .up_angle_ptr = NULL, // 指向上云台yaw电机的角度反馈,因为不能赋值变量初始化，所以在任务开始时赋值
    .angle_range = 2.0f * PI, // 角度范围，单位弧度，360度为2*PI
    .gimbal_follow_pid_config = {
        .kp = -5.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .dead_zone = 0.15f,
        .i_max = 0.0f,
        .out_max = 2 * 3.141593f,
    }
};
static void Chassis_Disable(ChassisInstance_s *chassis){
	for(uint8_t i=0;i<4;i++){
	chassis->chassis_motor[i]->output=0.0f;
	Pid_Disable(chassis->chassis_motor[i]->velocity_pid);
	Pid_Disable(chassis->chassis_motor[i]->angle_pid);
	}
	
}
//这里要用引用传参否则会复制结构体，很浪费资源
static void Chassis_Enable(ChassisInstance_s *chassis){
	for(uint8_t i=0;i<4;i++){
	Pid_Enable(chassis->chassis_motor[i]->velocity_pid);
	Pid_Enable(chassis->chassis_motor[i]->angle_pid);
	}
	
}

static uint32_t trigger_pause_tick = 0;

static bool Trigger_Control(DjiMotorInstance_s *trigger, uint8_t shoot_bool) {
    if (trigger == NULL) {
        return false;
    }
    static float last_output = 0.0f;
    last_output = trigger->output;

    // 对速度反馈进行低通滤波，减少噪声干扰
    float filtered_velocity = 0.0f;
    LowpassFilter_Process(Trigger_In_LPF, trigger->message.out_velocity, &filtered_velocity);

    uint32_t current_tick = HAL_GetTick();

    // 1. 卡弹检测逻辑
    if (shoot_bool && !trigger_jam.is_rebounding) {
        // 如果目标速度很大但实际速度很小，且电流很大，判定为卡弹
        if (fabsf(trigger->target_velocity) > 10.0f && fabsf(filtered_velocity) < 5.0f && 
          (trigger->message.torque_current > TRIGGER_JAM_TORQUE_THRESHOLD ||
           trigger->message.torque_current < -TRIGGER_JAM_TORQUE_THRESHOLD)) {
            if (trigger_jam.jam_start_tick == 0) {
                trigger_jam.jam_start_tick = current_tick;
            } else if (current_tick - trigger_jam.jam_start_tick > 200) { // 持续200ms
                // 触发退弹
                trigger_jam.is_rebounding = true;
                trigger_jam.rebound_start_tick = current_tick;
                Log_Error("Trigger Jammed! Rebounding...");
            }
        } else {
            trigger_jam.jam_start_tick = 0;
        }
    }

    // 2. 状态机逻辑
    if (trigger_jam.is_rebounding) {
        if (current_tick - trigger_jam.rebound_start_tick < TRIGGER_JAM_REBOUND_DURATION_MS) {
            trigger->target_velocity = -target_tr; // 反转速度进行退弹
        } else {
            trigger_jam.is_rebounding = false;
            trigger_jam.jam_start_tick = 0;
            
        }
    } else if (shoot_bool == 1) {
        trigger->target_velocity = target_tr;
    } else {
        trigger->target_velocity = 0.0f;
    }

    // 使能/关闭 PID
    if (shoot_bool || trigger_jam.is_rebounding) {
        Pid_Enable(trigger->velocity_pid);
    } else {
        Pid_Disable(trigger->velocity_pid);
    }

    // 计算 PID 输出
    float raw_output = Pid_Calculate(trigger->velocity_pid, trigger->target_velocity, filtered_velocity);
   
    // 阶跃抑制：如果单次增量过大，则限制增量，防止电机电流突变
    const float max_step = 2000.0f; 
    if (raw_output - last_output > max_step) {
        raw_output = last_output + max_step;
    } else if (raw_output - last_output < -max_step) {
        raw_output = last_output - max_step;
    }

    // 对抑制后的 PID 输出进行低通滤波，进一步平滑
    LowpassFilter_Process(Trigger_Out_LPF, raw_output, &trigger->output);

    return true;
}


//其实把结构体定义到外面没有什么意义，不用指针的话还是会复制一份到栈内

//主任务
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
	CH_Subs=Create_Subscriber("dr16_topic",sizeof(Dr16Instance_s));
  CH_Receive_s = (Dr16Instance_s*)pvPortMalloc(sizeof(Dr16Instance_s)); // 为指针分配内存
  Chassis = Chassis_Register(&Chassis_config);
    if (Chassis == NULL) {
        Log_Error("Chassis Register Failed!");
    }
  MiniPC = Minipc_Register(&miniPC_config);
	MiniPC_SelfAim = Minipc_Register(&SelfAim_config);	
	MiniPC_ExpAim = Minipc_Register(&ExpAim_config);
    if (MiniPC == NULL||MiniPC_SelfAim==NULL||MiniPC_SelfAim==NULL) {
        Log_Error("MiniPC Register Failed!");
    }
		
	Down_yaw = Motor_DM_Register(&Down_config);
		if (Down_yaw == NULL){
				Log_Error("Chassis Register Failed!");
		}

	Trigger = Motor_Dji_Register(&Trigger_Config);
			Up_yaw = Motor_Dji_Register(&Up_config);
		if (Trigger == NULL){
				Log_Error("Trigger Register Failed!");
		}
  if (Trigger != NULL) {
    Trigger->target_velocity = 0.0f;
    Trigger_ResetState();
    Pid_Disable(Trigger->angle_pid);
  }
  board_instance= board_init(&board_config);
    if (board_instance == NULL) {
        Log_Error("Board Register Failed!");
    }
  GimbalFollow_config.up_angle_ptr = &Up_yaw->message.out_position;
  GimbalFollow_Instance = GimbalFollow_Register(&GimbalFollow_config);
      // 初始化拨弹盘滤波器
    FilterInitConfig_t trigger_filter_config = {
        .cutoff_freq = 30.0f,   // 截止频率 30Hz
        .sample_freq = 1000.0f  // 采样频率 1000Hz (任务周期 1ms)
    };
    Trigger_In_LPF = LowpassFilter_Register(&trigger_filter_config);
    
    trigger_filter_config.cutoff_freq = 30.0f; // 输出滤波可以稍微宽一点，减少延迟
    Trigger_Out_LPF = LowpassFilter_Register(&trigger_filter_config);
		 while (Quater.ins_ready!=1)
   {
       osDelay(10);
   }
		
		//循环使能
		#ifndef SHOOTER_DEBUG
		while(Down_yaw->motor_state!=DM_ENABLE){
			Motor_Dm_Cmd(Down_yaw,DM_CMD_MOTOR_ENABLE);
			Motor_Dm_Transmit(Down_yaw);
			osDelay(1);
		}
		#endif
		Minipc_ConfigAimTx(MiniPC,&board_instance->received_up_yaw_pos,&board_instance->received_up_pitch_pos,
                        &enemy_color,&minipc_mode,
                        &rune_flag,&Down_yaw->message.out_position);//这里可能引入悬空指针，但是似乎没影响程序运行，后面再管。
		
    
      
     //施工中，可能需要修改板间通信，我现在写的太烂了拓展性很差                   
    //Minipc_ConfigExpAimTx(MiniPC_SelfAim,)
    
   
    
    uint32_t dwt2_cnt_last = 0;
		float dt2 = 0.001f;  // 初始dt
		dwt2_cnt_last = DWT->CYCCNT;

    static uint8_t send_flag=0;
  
  while(Quater.ins_ready!=1)
   {
       osDelay(10);
   }
   //等待IMU初始化完成，确保姿态数据有效后再进入主循环
  for(;;)
  {
		
		#ifdef DEBUG
		test_speed=Down_yaw->message.out_velocity;
    test_pos_tr=Trigger->message.out_position;
    uint16_t last_wheel=CH_Receive_s->dr16_handle.wheel;
    speed1=Chassis->chassis_motor[0]->message.out_velocity;
//    speed2=Chassis->chassis_motor[1]->message.out_velocity;
    speed3=Chassis->chassis_motor[2]->message.out_velocity;
//    speed4=Chassis->chassis_motor[3]->message.out_velocity;
		target1=Chassis->chassis_motor[0]->target_velocity;
//		target2=Chassis->chassis_motor[1]->target_velocity;
		target3=Chassis->chassis_motor[2]->target_velocity;
//		target4=Chassis->chassis_motor[3]->target_velocity;
		test_position=Down_yaw->message.out_position;//Chassis->chassis_motor[0]->message.out_position;
		// target_speed=Down_yaw->target_velocity;//Chassis->chassis_motor[0]->target_velocity;
		dt2 = Dwt_GetDeltaT(&dwt2_cnt_last);
			test_vel_tr=Trigger->message.out_velocity;
			test_output_tr=Trigger->output;
		#endif

		Get_Message(CH_Subs,CH_Receive_s);
		
		control_mode=mode;
    // target_up_position=MiniPC_SelfAim->message.exp_aim_pack.yaw;
    // target_up_pitch=MiniPC_SelfAim->message.exp_aim_pack.pitch;

    if(CH_Receive_s->dr16_handle.wheel>400||MiniPC_SelfAim->message.norm_aim_pack.shoot_bool==0x31){
      shoot_bool=1;

    }else{
      shoot_bool=0;
    }

    Minipc_UpdateAllInstances();
    find_bool=MiniPC_SelfAim->message.norm_aim_pack.find_bool;
		if(control_mode==PC_MODE||control_mode==UP_MODE||control_mode==SHOOT_MODE){
      //两个周期跑一次。也就是500Hz
      if(send_flag==0){
        send_flag=1;
      }else{
      board_send_message(board_instance,target_up_position,Quater.yaw ,target_up_pitch, combined_state_global, find_bool);
      send_flag=0;
      }
  }
		
    //测试代码
    Follow_Calculate(GimbalFollow_Instance);
    #ifdef SHOOT_DEBUG
    control_mode=SHOOT_MODE;
    #endif 
    if(CH_Receive_s->dr16_handle.wheel<-400){
       buzzer_play_note(buzzer, 1, 1, 1, 300); // “滴”一声提示开始检测
             buzzer_play_note(buzzer, 2, 0, 1, 300); // “滴”一声提示开始检测
       buzzer_play_note(buzzer, 3, 1, 1, 300); // “滴”一声提示开始检测
      
      
    }
    //测试代码结束
    switch (control_mode)
    {
    case PC_MODE:
  				target_tr=40.0f;
        
        // Chassis->gimbal_yaw_angle
        //后面这里加个自动打弹逻辑
        target_up_position=MiniPC_SelfAim->message.norm_aim_pack.yaw;
        target_up_pitch=MiniPC_SelfAim->message.norm_aim_pack.pitch;

        Chassis_Change_Mode(Chassis, CHASSIS_NORMAL);
        Chassis->gimbal_yaw_angle=Down_yaw->message.out_position;
        Chassis->Chassis_speed.Vx=MiniPC->message.ch_pack.x_speed;
        Chassis->Chassis_speed.Vy=MiniPC->message.ch_pack.y_speed;
        // Down_yaw->target_position=MiniPC->message.ch_pack.yaw;
        Chassis_Control(Chassis);
        //临时逻辑
        target_position=MiniPC->message.ch_pack.yaw;
        target_position=target_position>PI?target_position-2*PI:target_position;
        target_position=target_position<-PI?target_position+2*PI:target_position;
        target_position=target_position>PI+0.1f?PI:target_position;
        target_position=target_position<-PI-0.1f?-PI:target_position;
        
        Motor_Dm_Cmd(Down_yaw,DM_CMD_MOTOR_DISABLE);
				Motor_Dm_Transmit(Down_yaw);
        Pid_Disable(Trigger->angle_pid);


        target_position=Quater.yaw;
        //防止疯车用的

        Trigger_Control(Trigger, shoot_bool);
          // Motor_Dji_Control(Trigger,Trigger->target_velocity); // Trigger_Control handles PID and output
          Motor_Dji_Transmit(Trigger);
        




//    target_speed=Pid_Calculate(Down_yaw->angle_pid,target_position,Quater.yaw);
//    test_output=Pid_Calculate(Down_yaw->velocity_pid,target_speed,QEKF_INS.Gyro[2]);
//    Motor_Dm_Mit_Control(Down_yaw,0.0,0.0,test_output);
//	  Motor_Dm_Transmit(Down_yaw);

        break;
		
    case RC_MODE:
        /* code */
				//ch2：x，ch3：y
        //Chassis_Change_Mode(Chassis, CHASSIS_NORMAL);
		    Chassis_Change_Mode(Chassis, CHASSIS_GYROSCOPE);
				Chassis->gimbal_yaw_angle=Down_yaw->message.out_position;
        //摇杆漂移死区
        if(abs(CH_Receive_s->dr16_handle.ch2)<15){
          CH_Receive_s->dr16_handle.ch2=0;
        }
				Chassis->Chassis_speed.Vx=CH_Receive_s->dr16_handle.ch3/132.0f;
				Chassis->Chassis_speed.Vy=-CH_Receive_s->dr16_handle.ch2/132.0f;
				Chassis_Control(Chassis);
        //小yaw位置跟随，避免出现问题
        target_up_position=board_instance->received_up_yaw_pos;
        //大Yaw控制逻辑
				target_position-=(CH_Receive_s->dr16_handle.ch0) * 3.1415 / 360000.0f;
        target_position=target_position>PI?target_position-2*PI:target_position;
        target_position=target_position<-PI?target_position+2*PI:target_position;
        target_position=target_position>PI+0.1f?PI:target_position;
        target_position=target_position<-PI-0.1f?-PI:target_position;
        target_speed=Pid_Calculate(Down_yaw->angle_pid,target_position,Quater.yaw);
        
				//Down_yaw->target_velocity=Pid_Calculate(Down_yaw->angle_pid,Down_yaw->target_position,QEKF_INS.Yaw);
				//Motor_Dm_Control(Down_yaw,Pid_Calculate(Down_yaw->angle_pid,Down_yaw->target_position,QEKF_INS.Yaw*3.1415/360));
				//Motor_Dm_Control(Down_yaw,target_position);
				//test_output=Down_yaw->output;
        test_output=Pid_Calculate(Down_yaw->velocity_pid,target_speed,Quater.Gyro[2]);
				//Motor_Dm_Mit_Control(Down_yaw,0.0,0.0,Down_yaw->output);
        Motor_Dm_Mit_Control(Down_yaw,0.0,0.0,test_output);
				Motor_Dm_Transmit(Down_yaw);

         target_tr=0.0f;
        Pid_Disable(Trigger->angle_pid);
        Trigger_ResetState();
        Trigger->output=0.0f;
        Motor_Dji_Transmit(Trigger);
				break;
					
        
    case TRANS_MODE:
        /* code */
				Chassis_Enable(Chassis);
				Motor_Dm_Cmd(Down_yaw,DM_CMD_MOTOR_ENABLE);
				Motor_Dm_Transmit(Down_yaw);
        Trigger_ResetState();
        // Pid_Enable(Trigger->angle_pid); // 速度控制不需要使能位置环
				
                // 修正2: 切换模式时，将目标设为当前IMU角度，实现平滑“锁头”
                target_position = Quater.yaw; 

        break;
		//case SCROP_MODE:
				
    case DISABLE_MODE:
				Motor_Dm_Cmd(Down_yaw,DM_CMD_MOTOR_DISABLE);

		Motor_Dm_Transmit(Down_yaw);
        Pid_Disable(Trigger->angle_pid);
				Chassis_Change_Mode(Chassis,CHASSIS_NORMAL);
        Chassis_Disable(Chassis);
				
        Chassis->Chassis_speed.Vx=0.0f;
        Chassis->Chassis_speed.Vy=0.0f;
				Chassis->Chassis_speed.Vw=0.0f;
        Trigger_ResetState();
        Trigger->output = 0.0f;
        Motor_Dji_Transmit(Trigger);
        
        // 修正: 禁用模式下持续重置目标位置为当前角度，防止切出时疯转
        target_position = Quater.yaw;

        break;
			case SHOOT_MODE:
        //target_tr=60.0f;
				Motor_Dm_Cmd(Down_yaw,DM_CMD_MOTOR_DISABLE);
				Motor_Dm_Transmit(Down_yaw);
				target_tr=40.0;
				Chassis_Change_Mode(Chassis,CHASSIS_NORMAL);
        
				
        Chassis->Chassis_speed.Vx=0.0f;
        Chassis->Chassis_speed.Vy=0.0f;
				Chassis->Chassis_speed.Vw=0.0f;
				// Pid_Enable(Trigger->angle_pid);                
                // 修正: 射击模式下大Yaw无力，需同步目标值防止切回RC时跳变
                target_position = Quater.yaw;
        if(!Trigger_Control(Trigger, shoot_bool)){
          // Motor_Dji_Control(Trigger,Trigger->target_velocity); // Trigger_Control handles PID and output
          Motor_Dji_Transmit(Trigger);
        }
//                if(Trigger->message.torque_current>max_torque||Trigger->message.torque_current<-max_torque){
//                    lasttime++;
//                    if(lasttime>100){
//                    Trigger->output=0.0;
//                    }
//                }else{
//                    lasttime=0;
//                }
        break;
      case UP_MODE:
      //小云台逻辑
      if(Up_yaw!=NULL){
        Follow_Calculate(GimbalFollow_Instance);
      }else{
        control_mode=RC_MODE;
        break;
      }
        target_up_position-=(CH_Receive_s->dr16_handle.ch0) * 3.1415 / 360000.0f;
        target_up_pitch-=(CH_Receive_s->dr16_handle.ch1)*3.1415/ 360000.0f;
        
				// if (target_up_position > PI) target_up_position -= 2 * PI;
				// if (target_up_position < -PI) target_up_position += 2 * PI;
				// if (target_up_position < 1.7f && target_up_position > -1.7f) {
				// 		if (target_up_position > 0) target_up_position = 1.7f;
				// 		else target_up_position = -1.7f;
				// }
        //重新改回原来基于IMU的限幅
        target_up_position=target_up_position<-1.7f? -1.7f:target_up_position;
        target_up_position=target_up_position>1.7f? 1.7f:target_up_position;

				target_up_pitch=target_up_pitch<-0.3?-0.3:target_up_pitch;
				target_up_pitch=target_up_pitch>0.7?0.7:target_up_pitch;
				//底盘逻辑
        Chassis_Change_Mode(Chassis, CHASSIS_NORMAL);
				Chassis->gimbal_yaw_angle=Down_yaw->message.out_position;
        //摇杆漂移死区
        if(abs(CH_Receive_s->dr16_handle.ch3)<5){
          CH_Receive_s->dr16_handle.ch3=0;
        }
				Chassis->Chassis_speed.Vx=CH_Receive_s->dr16_handle.ch3/132.0f;
				Chassis->Chassis_speed.Vy=-CH_Receive_s->dr16_handle.ch2/132.0f;
				Chassis_Control(Chassis);
        //大yaw
        //Motor_Dm_Control(Down_yaw,target_position);
				//test_output=Down_yaw->output;
				//Motor_Dm_Mit_Control(Down_yaw,0.0,0.0,Down_yaw->output);
				//target_speed=Pid_Calculate(Down_yaw->angle_pid,target_position,Quater.yaw);
        target_speed=GimbalFollow_Instance->output;
			  test_output=Pid_Calculate(Down_yaw->velocity_pid,target_speed,Quater.Gyro[2]);
				Motor_Dm_Mit_Control(Down_yaw,0.0,0.0,test_output);
				Motor_Dm_Transmit(Down_yaw);
				
        Pid_Disable(Trigger->angle_pid);
        Trigger_ResetState();
        target_tr=0.0f;
        Trigger->output=0.0f;
        Motor_Dji_Transmit(Trigger);
        break;
			case SCROP_MODE:
        // Yaw 锯齿波: 从 1.7 扫到 4.58 (即回绕后的 -1.7)，周期 2秒
       // target_up_position = (float)(HAL_GetTick() % 2000) / 2000.0f * 2.883f + 1.7f;
        //if (target_up_position > 3.14159f) target_up_position -= 6.28318f; 
       
        // Pitch 锯齿波: -0.3 到 0.7，周期 3秒
        

				Chassis_Change_Mode(Chassis, CHASSIS_GYROSCOPE);
				Chassis->gimbal_yaw_angle=Down_yaw->message.out_position;
				Chassis->Chassis_speed.Vx=CH_Receive_s->dr16_handle.ch3/132.0f;
				Chassis->Chassis_speed.Vy=-CH_Receive_s->dr16_handle.ch2/132.0f;
				Chassis_Control(Chassis);
        //大Yaw控制逻辑
				target_position-=(CH_Receive_s->dr16_handle.ch0) * 3.1415 / 360000.0f;
        target_position=target_position>PI?target_position-2*PI:target_position;
        target_position=target_position<-PI?target_position+2*PI:target_position;
        if(target_position>PI+0.1f) target_position=PI;
        if(target_position<-PI-0.1f) target_position=-PI;
        target_speed=Pid_Calculate(Down_yaw->angle_pid,target_position,Quater.yaw);
        
        test_output=Pid_Calculate(Down_yaw->velocity_pid,target_speed,Quater.Gyro[2]);
        Motor_Dm_Mit_Control(Down_yaw,0.0,0.0,test_output);
				Motor_Dm_Transmit(Down_yaw);


        target_tr=0.0f;
        Trigger->output=0.0f;
				Pid_Disable(Trigger->angle_pid);
        Trigger_ResetState();
        Motor_Dji_Transmit(Trigger);
				break;
    default:
			mode=DISABLE_MODE;
        break;
    }
    osDelay(1);
  }  /* USER CODE END StartChassisTask */
}