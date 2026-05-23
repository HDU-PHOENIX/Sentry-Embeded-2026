/**
 * @file app_shooter_task.c
 * @author CGH
 * @brief 射击任务
 * 
 */

#include "app_shooter_task.h"
#include "robot_config.h"

#define HALF_RANGE PI

#define right_wheel_vel_lpf_cutoff  25.0f;    // 截止频率（Hz），按需调整
#define right_wheel_vel_sample_freq  1000.0f; // 采样频率（Hz），默认1kHz
//实例声明
DjiMotorInstance_s *Left_Wheel;
DjiMotorInstance_s *Right_Wheel;
extern uint8_t gimbal_ready_flag;
extern board_instance_t *board_instance;
#define DEBUG
#ifdef DEBUG
float target_wheel_speed=0.0;
float test_speed_left=0.0f;
float test_speed_right=0.0f;
uint8_t test_flag=1;
#endif

//变量声明
extern ShooterState_t Shooter_State;
extern ShooterState_t Shooter_State_last;

LowpassFilter_t *right_wheel_vel_filter = NULL;
FilterInitConfig_t right_wheel_filter_config;

float filtered_right_vel;
float last_time=0.0f;
float heat=0.0f;
float cool=30;
uint8_t CtrlMode=0;



////////////////////////////电机配置/////////////////////////////////////////
static  DjiMotorInitConfig_s Left_Config = {
    .id = 2,                      // 电机ID(1~4)
    .type = M3508,               // 电机类型
    .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "LW",
    .can_config = {
        .can_number = 1,
				.topic_name = "LW",              // can句柄
        .tx_id = 0x200,                     // 发送id 
        .rx_id = 0x202,                     // 接收id
    },
    .reduction_ratio = 1,              // 减速比

    .angle_pid_config = {
        .kp = 0.0,                        // 位置环比例系数
        .ki = 0.0,                        // 位置环积分系数
        .kd = 0.0,                        // 位置环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 2.0f * PI,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0,                   // 积分限幅
        .out_max = 400.0,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
        .kp = 20.0,                       // 速度环比例系数
        .ki = 0.0,                        // 速度环积分系数
        .kd = 0.0,                        // 速度环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 4000.0,                  // 积分限幅
        .out_max = 16000.0,                // 输出限幅(电流输出)
    }
};
static  DjiMotorInitConfig_s Right_Config = {
    .id = 1,                      // 电机ID(1~4)
    .type = M3508,               // 电机类型
    .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "RW",
    .can_config = {
        .can_number = 1,
				.topic_name = "RW",              // can句柄
        .tx_id = 0x200,                     // 发送id 
        .rx_id = 0x201,                     // 接收id
    },
    .reduction_ratio = 1,              // 减速比

    .angle_pid_config = {
        .kp = 0.0,                        // 位置环比例系数
        .ki = 0.0,                        // 位置环积分系数
        .kd = 0.0,                        // 位置环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 2.0f * PI,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0,                   // 积分限幅
        .out_max = 400.0,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
        .kp = 20.0,                       // 速度环比例系数
        .ki = 0.0,                        // 速度环积分系数
        .kd = 0.0,                        // 速度环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 4000.0,                  // 积分限幅
        .out_max = 16000.0,                // 输出限幅(电流输出)
    }
};









  



//整体逻辑是这样：先进摩擦轮start使其转速稳定，稳定态下会进行一发拨弹。
//如果不稳定，则视为开火状态，不拨弹，除非其稳定
void StartShooterTask(void const * argument)
{

  /* USER CODE BEGIN StartShooterTask */
	
	Left_Wheel=Motor_Dji_Register(&Left_Config);
  Right_Wheel=Motor_Dji_Register(&Right_Config);
  Shooter_State=SHOOTER_STOP;
  #ifdef DEBUG
  Shooter_State = SHOOTER_TEST;
  #endif
  right_wheel_filter_config.cutoff_freq = right_wheel_vel_lpf_cutoff;
  right_wheel_filter_config.sample_freq = right_wheel_vel_sample_freq;
  right_wheel_filter_config.filter_size = 0; // 对低通滤波器未使用
  right_wheel_vel_filter = LowpassFilter_Register(&right_wheel_filter_config);



   


 
  // 初始化时按固定方向旋转到原点
//while (fabsf(Trigger->message.out_position - FIRE_ORIGIN) > 0.03f) {
//    pos_target_tr -= 0.001f;
//    pos_target_tr=pos_target_tr>HALF_RANGE?pos_target_tr-2*HALF_RANGE:pos_target_tr;
//    pos_target_tr=pos_target_tr<-HALF_RANGE?pos_target_tr+2*HALF_RANGE:pos_target_tr;
//    Motor_Dji_Control(Trigger, pos_target_tr);
//    Motor_Dji_Transmit(Trigger);
//		if(Trigger->message.torque_current>MAX_TORQUE||Trigger->message.torque_current<-MAX_TORQUE)break;
//    //或许可以来个超时检测，但是等看门狗什么的都完善了再一并加入吧
//    osDelay(1);
//}
	while(gimbal_ready_flag!=1||Quater.ins_ready!=1){
		osDelay(1);
	}
  osDelay(20);
  //测试用↓
  //Right_Wheel->velocity_pid->is_enabled = 0;
  //测试用↑
  /* Infinite loop */
  for(;;)
  { 
    
	#ifdef DEBUG
		//Shooter_State=SHOOTER_TEST;
		#endif 

    //Shooter_State_last=Shooter_State;

    switch(Shooter_State){
      case SHOOTER_READY:
				if(Shooter_State_last==SHOOTER_STOP){
        last_time=0;
        Shooter_State=SHOOTER_TRANS;
        break;
      }
				Pid_Enable(Left_Wheel->velocity_pid);
        Pid_Enable(Right_Wheel->velocity_pid);
				
				target_wheel_speed=SHOOTER_WHEEL_SPEED;
				
		
				
			break;
      case SHOOTER_STOP:
        //清空PID
        target_wheel_speed=0.0f;
//				Left_Wheel->output=0.0;
//			Right_Wheel->output=0.0;
//        Pid_Disable(Left_Wheel->velocity_pid);
//        Pid_Disable(Right_Wheel->velocity_pid);
				Motor_Dji_Control(Right_Wheel,target_wheel_speed);
		    Motor_Dji_Control(Left_Wheel,-target_wheel_speed);
        Motor_Dji_Transmit(Right_Wheel);  
    Motor_Dji_Transmit(Left_Wheel);
       break;
      case SHOOTER_STARTFIRE:
      
        if(Shooter_State_last==SHOOTER_STOP){
          last_time=0;
          Shooter_State=SHOOTER_TRANS;
          break;
        } 
        target_wheel_speed=SHOOTER_WHEEL_SPEED;
        //检查力矩，如果力矩依然保持很大,那么证明还在发弹，累加lasttime
        if(Right_Wheel->message.torque_current>MAX_TORQUE||Right_Wheel->message.torque_current<-MAX_TORQUE){
          last_time++;
        }else{
          Shooter_State=SHOOTER_READY;
          last_time=0;
        }
        //这里目标速度是负数所以是大于目标值加一个范围
        if(Right_Wheel->message.out_velocity>SHOOTER_WHEEL_SPEED+100.0f){
          //速度未达标，视为开火状态
          if(last_time>5){
            Shooter_State=SHOOTER_FIRING;
            break;
          }  
      }
      
      
     
      break;
      case SHOOTER_FIRING:
      //这里可能有个问题，实际上力矩正负应该是固定的，这个判断逻辑有问题或者压根没用
      //直到检测到电流小于某个值的时候再跳转回不稳定的初始状态
      if(Right_Wheel->message.torque_current<FIRING_TORQUE||Right_Wheel->message.torque_current>-FIRING_TORQUE){
        Shooter_State=SHOOTER_STARTFIRE;
        continue;
      }
      
      target_wheel_speed=SHOOTER_WHEEL_SPEED;
      break;
      
      case SHOOTER_STUCK:
        //逻辑已经转嫁到下板

        break;
      case SHOOTER_TRANS:
				
        Pid_Enable(Left_Wheel->velocity_pid);
        Pid_Enable(Right_Wheel->velocity_pid);
        
        break;
      case SHOOTER_AUTO:
      //连射模式
      if(Shooter_State_last==SHOOTER_STOP){
        last_time=0;
        Shooter_State=SHOOTER_TRANS;
        break;
      }
			
      target_wheel_speed=SHOOTER_WHEEL_SPEED;
        break;
      case SHOOTER_TEST:
        //测试模式，直接控制任何一个电机
			
				target_wheel_speed=-WHEEL_SPEED;//-6666.0f;
        break;
			default:
			Left_Wheel->output=0.0;
			Right_Wheel->output=0.0;
      Pid_Disable(Left_Wheel->velocity_pid);
      Pid_Disable(Right_Wheel->velocity_pid);
				break;
    }
    
		
		
		test_speed_left=Left_Wheel->message.out_velocity;
		test_speed_right=Right_Wheel->message.out_velocity;
		float test_left;

    LowpassFilter_Process(right_wheel_vel_filter, Right_Wheel->message.out_velocity, &filtered_right_vel);
    
    
		

		Motor_Dji_Control(Right_Wheel,target_wheel_speed);
		


		//Motor_Dji_Control(Left_Wheel,filtered_right_vel);
		Motor_Dji_Control(Left_Wheel,-target_wheel_speed);
		Motor_Dji_Transmit(Right_Wheel);  
    Motor_Dji_Transmit(Left_Wheel);
     
    osDelay(1);

  }
  /* USER CODE END StartShooterTask */
}