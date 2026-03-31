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

/**
 * @brief 底盘功率限制函数 (未实现)
 * @note 预留功能，用于防止底盘超功率
 * @date 2025-07-03
 */
static bool Chassis_Power_Limit(ChassisInstance_s* Chassis, float power_buffer)
{
    if (Chassis == NULL || Chassis->Chassis_power_limit <= 0) {
        return false;
    }

    float power_max = Pid_Calculate(Chassis->Chassis_power_limit_pid_config, 30, power_buffer) + Chassis->Chassis_power_limit;
    float total_steering_power = 0.0f;
    float motor_target_power[8] = {0};
    float steering_scale = 1.0f;

    // 舵组功率限制
    if (Chassis->omni_steering_message.Steering_Ratio > 0.0f) {
        for (int i = 4; i < 8; i++) {
            float x = Chassis->chassis_motor[i]->output;
            float y = Chassis->chassis_motor[i]->message.rotor_velocity;
            Chassis->motor_power[i] = 1.421e-5f * x * y
                + Chassis->motor_loss_config[i].K1 * x * x
                + Chassis->motor_loss_config[i].K2 * y * y
                + Chassis->motor_loss_config[i].Ka;
            total_steering_power += Chassis->motor_power[i];
        }
        
        // 分母保护并计算期望比例
        if (total_steering_power > 0.001f) {
            steering_scale = Chassis->omni_steering_message.Steering_Ratio * power_max / total_steering_power;
        } else {
            // 分母为0（或极小）说明未吃功率，赋予大于1的值跳过截断逻辑
            steering_scale = 10.0f; 
        }
        
        // 只有当算出来的比例小于 1.0 时（确实超功率了），才重新计算二次方程，覆盖并截断之前控制算法写好的电流
        if (steering_scale < 1.0f && steering_scale >= 0.0f) {
            for (int i = 4; i < 8; i++) {
                motor_target_power[i] = steering_scale * Chassis->motor_power[i];
                float a = Chassis->motor_loss_config[i].K1;
                float b = 1.421e-5f * Chassis->chassis_motor[i]->message.rotor_velocity;
                float c = Chassis->motor_loss_config[i].K2 * Chassis->chassis_motor[i]->message.rotor_velocity * Chassis->chassis_motor[i]->message.rotor_velocity
                        + Chassis->motor_loss_config[i].Ka - motor_target_power[i];
                
                if (a == 0.0f) {
                    if (b != 0.0f) {
                        Motor_Dji_SetCurrent(Chassis->chassis_motor[i], -c / b);
                    } else {
                        Motor_Dji_SetCurrent(Chassis->chassis_motor[i], 0.0f);
                    }
                } else {
                    float discriminant = b * b - 4.0f * a * c;
                    if (discriminant < 0.0f) {
                        Motor_Dji_SetCurrent(Chassis->chassis_motor[i], 0.0f); 
                    } else {
                        float sqrt_d = sqrtf(discriminant);
                        float target_torque1 = (-b + sqrt_d) / (2.0f * a);
                        float target_torque2 = (-b - sqrt_d) / (2.0f * a);
                        if (target_torque1 * Chassis->chassis_motor[i]->output > 0.0f) {
                            Motor_Dji_SetCurrent(Chassis->chassis_motor[i], target_torque1);
                        } else {
                            Motor_Dji_SetCurrent(Chassis->chassis_motor[i], target_torque2);
                        }
                    }
                }
            }
        }
        
        // 把实际用于当前舵组计算的比例限制在 1.0 内，以保证能将未用满的舵向配额顺利移交给后续的轮组
        if (steering_scale > 1.0f) {
            steering_scale = 1.0f;
        }
    }

    // 轮组功率限制
    float total_wheel_power = 0.0f;
    for (int i = 0; i < 4; i++) {
        float x = Chassis->chassis_motor[i]->output; 
        float y = Chassis->chassis_motor[i]->message.rotor_velocity;
        Chassis->motor_power[i] = 1.996e-6f * x * y
            + Chassis->motor_loss_config[i].K1 * x * x
            + Chassis->motor_loss_config[i].K2 * y * y
            + Chassis->motor_loss_config[i].Ka;
        total_wheel_power += Chassis->motor_power[i];
    }

    float wheel_scale = 1.0f;
    if (total_wheel_power > 0.001f) {
        wheel_scale = (power_max - steering_scale * total_steering_power) / total_wheel_power;
    } else {
        wheel_scale = 10.0f;
    }
    
    // 同样，不超过最大功率限制则跳过计算，不做修改，直接保留原有基于外环PID生成的底盘推进期望输出
    if (wheel_scale < 1.0f && wheel_scale >= 0.0f) {
        for (int i = 0; i < 4; i++) {
            motor_target_power[i] = wheel_scale * Chassis->motor_power[i];
            float a = Chassis->motor_loss_config[i].K1;
            float b = 1.996e-6f * Chassis->chassis_motor[i]->message.rotor_velocity;
            float c = Chassis->motor_loss_config[i].K2 * Chassis->chassis_motor[i]->message.rotor_velocity * Chassis->chassis_motor[i]->message.rotor_velocity
                    + Chassis->motor_loss_config[i].Ka - motor_target_power[i];
            
            if (a == 0.0f) {
                if (b != 0.0f) {
                    Motor_Dji_SetCurrent(Chassis->chassis_motor[i], -c / b);
                } else {
                    Motor_Dji_SetCurrent(Chassis->chassis_motor[i], 0.0f);
                }
            } else {
                float discriminant = b * b - 4.0f * a * c;
                if (discriminant < 0.0f) {
                    Motor_Dji_SetCurrent(Chassis->chassis_motor[i], 0.0f);
                } else {
                    float sqrt_d = sqrtf(discriminant);
                    float target_torque1 = (-b + sqrt_d) / (2.0f * a);
                    float target_torque2 = (-b - sqrt_d) / (2.0f * a);
                    if (target_torque1 * Chassis->chassis_motor[i]->output > 0.0f) {
                        Motor_Dji_SetCurrent(Chassis->chassis_motor[i], target_torque1);
                    } else {
                        Motor_Dji_SetCurrent(Chassis->chassis_motor[i], target_torque2);
                    }
                }
            }
        }
    }
    return true;
}

 
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
    Chassis_Power_Limit(Chassis,30);//功率限制函数(未实现)
    Motor_Dji_Transmit(Chassis->chassis_motor[0]);
    if(Chassis->type == Steering_Wheel)
    {
    for(uint8_t i = 4; i < 8; i++)
    {
    Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_angle[i-4]);
    }
    }

   // Chassis_Power_Limit(Chassis,30);

    //Motor_Dji_Transmit(Chassis->chassis_motor[0]);
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


