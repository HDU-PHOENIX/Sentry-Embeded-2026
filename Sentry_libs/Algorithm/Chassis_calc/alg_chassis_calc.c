#include "alg_chassis_calc.h"
#include "math.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"

// Fixed install/coordinate offset for chassis forward direction (radians).
// Use this to compensate constant heading bias (e.g. 45deg => 0.785398f) without altering gimbal_yaw_zero.
#ifndef CHASSIS_YAW_OFFSET_RAD
#define CHASSIS_YAW_OFFSET_RAD (0.0f)
#endif

float err=0.0f;
/**
 * @brief 检查底盘配置参数合法性
 * @param config 底盘配置结构体指针
 * @return 配置合法返回true，否则返回false
 * @note 根据底盘类型检查关键参数是否有效
 * @date 2025-07-03
 */
static Chassis_Speed Temp_Speed;
static bool Chassis_judgment(ChassisInitConfig_s *config)
{
    // 全向轮/舵轮底盘必须设置旋转半径
    if((config->type == Omni_Wheel || config->type == Steering_Wheel) && config->omni_steering_message.chassis_radius == 0) 
    {
        return false;
    }
    // 麦轮底盘必须设置几何尺寸
    if((config->type == Mecanum_Wheel)&& 
       (config->mecanum_message.length_a == 0 || config->mecanum_message.length_b == 0)) 
    {
        return false;
    }
    return true;
}

static bool Chassis_Apply_Power_Control(ChassisInstance_s *Chassis)
{
    DjiMotorInstance_s *wheel_motors[4] = {0};
    DjiMotorInstance_s *steering_motors[4] = {0};
    uint8_t wheel_motor_count = 4U;
    uint8_t steering_motor_count = 0U;
    uint8_t index;

    if (Chassis == NULL)
    {
        return false;
    }

    for (index = 0U; index < wheel_motor_count; ++index)
    {
        wheel_motors[index] = Chassis->chassis_motor[index];
    }

    if (Chassis->type == Steering_Wheel)
    {
        steering_motor_count = 4U;
        for (index = 0U; index < steering_motor_count; ++index)
        {
            steering_motors[index] = Chassis->chassis_motor[index + 4U];
        }
    }

    return Chassis_PowerControl_Apply(&Chassis->power_control,
                                      wheel_motors,
                                      wheel_motor_count,
                                      steering_motors,
                                      steering_motor_count,
                                      Chassis->Chassis_power_limit,
                                      Chassis->Chassis_power_limit_pid_config,
                                      Chassis->chassis_power_buffer);
}
/**
 * @brief 注册并初始化底盘实例
 * @param Chassis_config 底盘初始化配置结构体指针
 * @return 成功返回底盘实例指针，失败返回NULL
 * @note 根据底盘类型初始化不同数量的电机
 * @date 2025-07-03
 */
ChassisInstance_s *Chassis_Register(ChassisInitConfig_s *Chassis_config){
    // 分配内存并初始化
     ChassisInstance_s *Chassis_Instance = (ChassisInstance_s *)pvPortMalloc(sizeof(ChassisInstance_s));
     memset(Chassis_Instance, 0, sizeof(ChassisInstance_s));
     Chassis_Instance->type=Chassis_config->type;
     Chassis_Instance->gimbal_yaw_zero = Chassis_config->gimbal_yaw_zero;
     Chassis_Instance->Gyroscope_Speed = Chassis_config->Gyroscope_Speed;

     Chassis_Instance->Chassis_power_limit = Chassis_config->Chassis_power_limit;
     Chassis_Instance->gimbal_follow_pid = Pid_Register(&Chassis_config->gimbal_follow_pid_config);
     Chassis_Instance->Chassis_power_limit_pid_config = Pid_Register(&Chassis_config->Chassis_power_limit_pid_config);
     Chassis_Instance->omni_steering_message = Chassis_config->omni_steering_message;
     Chassis_Instance->mecanum_message = Chassis_config->mecanum_message;
     Chassis_Instance->Chassis_Mode =  Chassis_config->Chassis_Mode;

    Chassis_Instance->chassis_power_feedback = 0.0f;
    Chassis_Instance->chassis_power_buffer = 0.0f;
    Chassis_PowerControl_Init(&Chassis_Instance->power_control, &Chassis_config->power_control_config);
	
    for(int i = 0; i < 4; i++)
    {
        Chassis_Instance->motor_loss_config[i] = Chassis_config->motor_loss_config[i];
        Chassis_Instance->chassis_motor[i] = Motor_Dji_Register(&Chassis_config->motor_config[i]);
    }
    if(Chassis_Instance->type == Steering_Wheel)
    {
            // 初始化转向电机
    for(int i = 4; i < 8; i++)
    {
        Chassis_Instance->motor_loss_config[i] = Chassis_config->motor_loss_config[i];
        Chassis_Instance->omni_steering_message.chassis_steering_normal[i-4] = Chassis_config->omni_steering_message.chassis_steering_normal[i-4];
        Chassis_Instance->omni_steering_message.chassis_steering_zero[i-4] = Chassis_config->omni_steering_message.chassis_steering_zero[i-4];
        Chassis_Instance->chassis_motor[i] = Motor_Dji_Register(&Chassis_config->motor_config[i]);
    }
    }
    // 检查初始化是否成功
    if (Chassis_Instance == NULL || !Chassis_judgment(Chassis_config)) {
        vPortFree(Chassis_Instance);
        return NULL;
    }
    return Chassis_Instance;
}

/**
 * @brief 底盘运动学逆解计算
 * @param chassis 底盘实例指针
 * @note 根据底盘类型计算各轮速度和角度
 * @date 2025-07-09
 */
static void Chassis_IK_Calc(ChassisInstance_s *Chassis)
{   
    switch (Chassis->type)
    {
    case Omni_Wheel:  // 全向轮逆解
        Chassis->out_speed[0] = ( -0.707f * Temp_Speed.Vx  + 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f/(3.14f * Chassis->omni_steering_message.wheel_radius);
        Chassis->out_speed[1] = ( -0.707f * Temp_Speed.Vx - 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f/(3.14f * Chassis->omni_steering_message.wheel_radius);
        Chassis->out_speed[2] = ( 0.707f * Temp_Speed.Vx - 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f/(3.14f * Chassis->omni_steering_message.wheel_radius);
        Chassis->out_speed[3] = ( 0.707f * Temp_Speed.Vx + 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f /(3.14f * Chassis->omni_steering_message.wheel_radius);
        break;
    
    case Mecanum_Wheel:  // 麦轮逆解
        Chassis->out_speed[0] = (-Temp_Speed.Vx + Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius);
        Chassis->out_speed[1] = (-Temp_Speed.Vx - Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius) ;
        Chassis->out_speed[2] = ( Temp_Speed.Vx - Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius);
        Chassis->out_speed[3] = ( Temp_Speed.Vx + Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius);
        break;

    case Steering_Wheel:  // 舵轮逆解（保留实现）
        // 计算各轮速度 (欧几里得范数)
    Chassis->out_speed[0] = sqrtf(powf(Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + powf(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    Chassis->out_speed[1] = sqrtf(powf(Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + pow(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    Chassis->out_speed[2] = sqrtf(powf(Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + powf(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    Chassis->out_speed[3] = sqrtf(powf(Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + powf(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    // 计算各轮转向角度
    Chassis->out_angle[0] = atan2f(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[0];
    Chassis->out_angle[1] = atan2f(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[1];
    Chassis->out_angle[2] = atan2f(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[2];
    Chassis->out_angle[3] = atan2f(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[3];
    for(int i=0;i<4;i++)
    {   if(Temp_Speed.Vx==0&&Temp_Speed.Vy==0)
        {
            Chassis->out_angle[i]=Chassis->omni_steering_message.chassis_steering_normal[i];
        }
        // 速度方向调整
        if(fabsf(Chassis->out_angle[i] - Chassis->chassis_motor[i+4]->message.out_position) > 1.570796f)
        {
            Chassis->out_speed[i] = -Chassis->out_speed[i];
            if(Chassis->out_angle[i] > 0)
            {
                Chassis->out_angle[i] -= 3.141593f;
            }
            else
            {
                Chassis->out_angle[i] += 3.141593f;
            }
            Chassis->out_angle[i] = fmodf(Chassis->out_angle[i] + 3.141593f, 2.0f * 3.141593f);
            if (Chassis->out_angle[i] < 0.0f){
                Chassis->out_angle[i] += 2.0f * 3.141593f;
            }
            Chassis->out_angle[i] -= 3.141593f;// 角度归一化到[-π, π]
        }
    }
    break;

    default:
        break;
    }
}

/**
 * @brief 计算云台角度误差（相对于零点）
 * @param Chassis 底盘实例指针
 * @return 角度误差值（弧度）
 * @note 将角度误差限制在[-π, π]范围内，哨兵这里加个45度来修正试试看
 * @date 2025-12-17
 */
static float Find_Angle(ChassisInstance_s *Chassis)
{
	 err = (Chassis->gimbal_yaw_angle - Chassis->gimbal_yaw_zero) + CHASSIS_YAW_OFFSET_RAD;
	
    return err;
}

/*
 * @brief 底盘功率限制函数 (未实现)
 * @note 预留功能，用于防止底盘超功率
 * @date 2025-07-03
 */
 
/**
 * @brief 底盘运动控制主函数
 * @param chassis 底盘实例指针
 * @return 控制成功返回true，失败返回false
 * @note 执行逆解计算并控制电机
 * @date 2025-07-09
 */
bool Chassis_Control(ChassisInstance_s *Chassis)
{
     // 1.选择底盘工作模式
    switch(Chassis->Chassis_Mode)
    {
    case CHASSIS_FOLLOW_GIMBAL:  // 跟随云台模式
        Chassis->Chassis_speed.Vw = Pid_Calculate(Chassis->gimbal_follow_pid, 0, Find_Angle(Chassis));
        break;

    case CHASSIS_NORMAL:  // 独立运动模式
        Chassis->Chassis_speed.Vw = 0;
        break;

    case CHASSIS_GYROSCOPE:  // 小陀螺模式
        Chassis->Chassis_speed.Vw = Chassis->Gyroscope_Speed;
        break;
    default:
        break;
    }
    // 2. 坐标系转换
    Temp_Speed.Vx =  Chassis->Chassis_speed.Vx *cosf(Find_Angle(Chassis)) - Chassis->Chassis_speed.Vy * sinf(Find_Angle(Chassis));
    Temp_Speed.Vy =  Chassis->Chassis_speed.Vx *sinf(Find_Angle(Chassis)) + Chassis->Chassis_speed.Vy * cosf(Find_Angle(Chassis));
    Temp_Speed.Vw =  Chassis->Chassis_speed.Vw;
    // 3. 执行运动学逆解
    Chassis_IK_Calc(Chassis);

    // 4. 控制四个驱动电机
    for(uint8_t i = 0; i < 4; i++)
    {
        Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_speed[i]);
    }
    // 5. 发送CAN命令 (通过第一个电机实例)
   // Chassis_Power_Limit(Chassis,60);//功率限制函数(未实现)
    Motor_Dji_Transmit(Chassis->chassis_motor[0]);
    if(Chassis->type == Steering_Wheel)
    {
    for(uint8_t i = 4; i < 8; i++)
    {
    Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_angle[i-4]);
    }
    }

    Chassis_Apply_Power_Control(Chassis);

    Motor_Dji_Transmit(Chassis->chassis_motor[0]);
    if(Chassis->type == Steering_Wheel)
    {
        Motor_Dji_Transmit(Chassis->chassis_motor[4]);
    }
    return true;
}

bool Chassis_Update_Power_State(ChassisInstance_s *Chassis, float chassis_power, float power_buffer)
{
    if (Chassis == NULL)
    {
        return false;
    }

    Chassis->chassis_power_feedback = chassis_power;
    Chassis->chassis_power_buffer = power_buffer;
    return true;
}


/**
 * @brief 底盘工作模式选择函数
 * @param ChassisAction 底盘实例指针
 * @note 根据当前模式修改控制参数
 * @date 2025-07-09
 */
bool Chassis_Change_Mode(ChassisInstance_s* Chassis,ChassisAction target_mode)
{
  if(Chassis == NULL) {
        return false;
    }
    Chassis->Chassis_Mode = target_mode;
    return true;
}


