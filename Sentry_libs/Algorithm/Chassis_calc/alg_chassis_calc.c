#include "alg_chassis_calc.h"
#include "math.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
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
     Chassis_Instance->super_chassis_active = false;
    // Chassis_Instance->supercap = Supercap_Register(&Chassis_config->supercap_config);
     Chassis_Instance->Chassis_power_limit = Chassis_config->Chassis_power_limit;
     Chassis_Instance->gimbal_follow_pid = Pid_Register(&Chassis_config->gimbal_follow_pid_config);
     Chassis_Instance->Chassis_power_limit_pid_config = Pid_Register(&Chassis_config->Chassis_power_limit_pid_config);
     Chassis_Instance->omni_steering_message = Chassis_config->omni_steering_message;
     Chassis_Instance->mecanum_message = Chassis_config->mecanum_message;
     Chassis_Instance->Chassis_Mode =  Chassis_config->Chassis_Mode;

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
    if (!Chassis_judgment(Chassis_config)) {
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

    case Steering_Wheel:  // 长方形舵轮逆解
        {
            float Lx = Chassis->omni_steering_message.half_length; // 车体前后半长
            float Ly = Chassis->omni_steering_message.half_width;  // 车体左右半宽
            float Rw = Chassis->omni_steering_message.wheel_radius;
            float vx0 = Temp_Speed.Vx - Temp_Speed.Vw * Ly;
            float vy0 = Temp_Speed.Vy + Temp_Speed.Vw * Lx;
            Chassis->out_speed[0] = sqrtf(vx0 * vx0 + vy0 * vy0) * 60.0f / (3.1415926f * Rw);
            Chassis->out_angle[0] = atan2f(vy0, vx0) + Chassis->omni_steering_message.chassis_steering_zero[0];
            float vx1 = Temp_Speed.Vx - Temp_Speed.Vw * Ly;
            float vy1 = Temp_Speed.Vy - Temp_Speed.Vw * Lx;
            Chassis->out_speed[1] = sqrtf(vx1 * vx1 + vy1 * vy1) * 60.0f / (3.1415926f * Rw);
            Chassis->out_angle[1] = atan2f(vy1, vx1) + Chassis->omni_steering_message.chassis_steering_zero[1];
            float vx2 = Temp_Speed.Vx + Temp_Speed.Vw * Ly;
            float vy2 = Temp_Speed.Vy - Temp_Speed.Vw * Lx;
            Chassis->out_speed[2] = sqrtf(vx2 * vx2 + vy2 * vy2) * 60.0f / (3.1415926f * Rw);
            Chassis->out_angle[2] = atan2f(vy2, vx2) + Chassis->omni_steering_message.chassis_steering_zero[2];
            float vx3 = Temp_Speed.Vx + Temp_Speed.Vw * Ly;
            float vy3 = Temp_Speed.Vy + Temp_Speed.Vw * Lx;
            Chassis->out_speed[3] = sqrtf(vx3 * vx3 + vy3 * vy3) * 60.0f / (3.1415926f * Rw);
            Chassis->out_angle[3] = atan2f(vy3, vx3) + Chassis->omni_steering_message.chassis_steering_zero[3];
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
        }
    break;

    default:
        break;
    }
}
//static void Chassis_FK_Calc(ChassisInstance_s *Chassis)
//{
//    // 临时变量，存储轮子的线速度 (m/s)
//    float wheel_linear_speed[4] = {0.0f};
//		float r = Chassis->omni_steering_message.chassis_radius;
//    // 根据不同底盘类型，先将轮子转速(RPM)转换为线速度(m/s)
//    switch (Chassis->type)
//    {
//    case Omni_Wheel:  // 全向轮正解
//        // 1. 转速(RPM)转线速度(m/s): v = (2πr * RPM) / 60
//        for (int i = 0; i < 4; i++) {
//            wheel_linear_speed[i] = (PI * Chassis->omni_steering_message.wheel_radius * Chassis->chassis_motor[i]->message.out_velocity) / 30.0f;
//        }
//
//        // 2. 正解计算：基于逆解的线性方程组求逆
//        // 逆解：v1 = -0.707Vx + 0.707Vy + r*Vw
//        //      v2 = -0.707Vx - 0.707Vy + r*Vw
//        //      v3 =  0.707Vx - 0.707Vy + r*Vw
//        //      v4 =  0.707Vx + 0.707Vy + r*Vw
//        // 正解推导：
//
//        float sum_v = wheel_linear_speed[0] + wheel_linear_speed[1] + wheel_linear_speed[2] + wheel_linear_speed[3];
//        float temp1 = wheel_linear_speed[3] - wheel_linear_speed[2] - wheel_linear_speed[1] + wheel_linear_speed[0];
//        float temp2 = wheel_linear_speed[3] + wheel_linear_speed[2] - wheel_linear_speed[1] - wheel_linear_speed[0];
//
//        Chassis->Now_Chassis_speed.Vw = sum_v / 4.0f / r;
//        Chassis->Now_Chassis_speed.Vy = temp2 / (2.0f * 0.707f);
//        Chassis->Now_Chassis_speed.Vx = -temp1 / (2.0f * 0.707f);
//        break;
//
//    case Mecanum_Wheel:  // 麦克纳姆轮正解
//        // 1. 转速(RPM)转线速度(m/s): v = (2πr * RPM) / 60
//        for (int i = 0; i < 4; i++) {
//            wheel_linear_speed[i] = (PI * Chassis->mecanum_message.wheel_radius * Chassis->chassis_motor[i]->message.out_velocity) / 60.0f;
//        }
//
//        // 2. 正解计算：基于逆解的线性方程组求逆
//        // 逆解：v1 = -Vx + Vy + L*Vw
//        //      v2 = -Vx - Vy + L*Vw
//        //      v3 =  Vx - Vy + L*Vw
//        //      v4 =  Vx + Vy + L*Vw
//        // 其中 L = length_a + length_b
//        float L = Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b;
//
//        Temp_Speed.Vx = (wheel_linear_speed[3] + wheel_linear_speed[2] - wheel_linear_speed[1] - wheel_linear_speed[0]) / 4.0f;
//        Temp_Speed.Vy = (wheel_linear_speed[3] - wheel_linear_speed[2] - wheel_linear_speed[1] + wheel_linear_speed[0]) / 4.0f;
//        Temp_Speed.Vw = (wheel_linear_speed[0] + wheel_linear_speed[1] + wheel_linear_speed[2] + wheel_linear_speed[3]) / (4.0f * L);
//        break;
//
//    case Steering_Wheel:  // 舵轮正解
//        // 1. 转速(RPM)转线速度(m/s)
//        for (int i = 0; i < 4; i++) {
//            wheel_linear_speed[i] = (PI * Chassis->omni_steering_message.wheel_radius * Chassis->chassis_motor[i]->message.out_velocity) / 60.0f;
//        }
//
//        // 2. 舵轮正解：基于轮子实际角度和线速度，解线性方程组
//        // 每个轮子的线速度分解：
//        // v_wheel_i = (Vx + Vw*y_i) * cos(theta_i) + (Vy - Vw*x_i) * sin(theta_i)
//        // 其中 (x_i,y_i) 是轮子的位置坐标，theta_i 是轮子实际转向角度
//        float sqrt2_2 = 0.707107f;
//        // 轮子位置坐标 (基于原逆解的几何关系)
//        float wheel_x[4] = { -r*sqrt2_2,  r*sqrt2_2,  r*sqrt2_2, -r*sqrt2_2 };
//        float wheel_y[4] = { -r*sqrt2_2, -r*sqrt2_2,  r*sqrt2_2,  r*sqrt2_2 };
//
//        // 实际转向角度（减去零位补偿）
//        float theta[4];
//        for (int i = 0; i < 4; i++) {
//            theta[i] = Chassis->out_angle[i] - Chassis->omni_steering_message.chassis_steering_zero[i];
//            // 角度归一化到 [-π, π]
//            theta[i] = fmodf(theta[i] + PI, 2*PI) - PI;
//        }
//
//        // 构建超定线性方程组 Ax = b，用最小二乘法求解 Vx, Vy, Vw
//        // 方程形式：
//        // cos(theta0)*Vx + sin(theta0)*Vy + (y0*cos(theta0) - x0*sin(theta0))*Vw = v0
//        // cos(theta1)*Vx + sin(theta1)*Vy + (y1*cos(theta1) - x1*sin(theta1))*Vw = v1
//        // ...
//        float A[4][3] = {0};
//        float b[4] = {0};
//
//        for (int i = 0; i < 4; i++) {
//            A[i][0] = cosf(theta[i]);                          // Vx 系数
//            A[i][1] = sinf(theta[i]);                          // Vy 系数
//            A[i][2] = wheel_y[i]*cosf(theta[i]) - wheel_x[i]*sinf(theta[i]); // Vw 系数
//            b[i] = wheel_linear_speed[i];                      // 轮子线速度
//        }
//
//        // 最小二乘法求解 (ATA)x = ATb (简化实现，适用于4个方程3个未知数)
//        float AT[3][4], ATA[3][3], ATb[3];
//        // 计算 AT (A的转置)
//        for (int i = 0; i < 3; i++) {
//            for (int j = 0; j < 4; j++) {
//                AT[i][j] = A[j][i];
//            }
//        }
//
//        // 计算 ATA = AT * A
//        for (int i = 0; i < 3; i++) {
//            for (int j = 0; j < 3; j++) {
//                ATA[i][j] = 0;
//                for (int k = 0; k < 4; k++) {
//                    ATA[i][j] += AT[i][k] * A[k][j];
//                }
//            }
//        }
//
//        // 计算 ATb = AT * b
//        for (int i = 0; i < 3; i++) {
//            ATb[i] = 0;
//            for (int k = 0; k < 4; k++) {
//                ATb[i] += AT[i][k] * b[k];
//            }
//        }
//
//        // 解 3x3 线性方程组 ATA * [Vx; Vy; Vw] = ATb
//        // 使用克莱姆法则求解
//        float det = ATA[0][0]*(ATA[1][1]*ATA[2][2] - ATA[1][2]*ATA[2][1])
//                  - ATA[0][1]*(ATA[1][0]*ATA[2][2] - ATA[1][2]*ATA[2][0])
//                  + ATA[0][2]*(ATA[1][0]*ATA[2][1] - ATA[1][1]*ATA[2][0]);
//
//        // 防止除零
//        if (fabsf(det) < 1e-6f) {
//            Temp_Speed.Vx = Temp_Speed.Vy = Temp_Speed.Vw = 0.0f;
//            break;
//        }
//
//        // 计算 Vx
//        float det_x = ATb[0]*(ATA[1][1]*ATA[2][2] - ATA[1][2]*ATA[2][1])
//                    - ATA[0][1]*(ATb[1]*ATA[2][2] - ATA[1][2]*ATb[2])
//                    + ATA[0][2]*(ATb[1]*ATA[2][1] - ATA[1][1]*ATb[2]);
//
//        // 计算 Vy
//        float det_y = ATA[0][0]*(ATb[1]*ATA[2][2] - ATA[1][2]*ATb[2])
//                    - ATb[0]*(ATA[1][0]*ATA[2][2] - ATA[1][2]*ATA[2][0])
//                    + ATA[0][2]*(ATA[1][0]*ATb[2] - ATb[1]*ATA[2][0]);
//
//        // 计算 Vw
//        float det_w = ATA[0][0]*(ATA[1][1]*ATb[2] - ATb[1]*ATA[2][1])
//                    - ATA[0][1]*(ATA[1][0]*ATb[2] - ATb[1]*ATA[2][0])
//                    + ATb[0]*(ATA[1][0]*ATA[2][1] - ATA[1][1]*ATA[2][0]);
//
//        Temp_Speed.Vx = det_x / det;
//        Temp_Speed.Vy = det_y / det;
//        Temp_Speed.Vw = det_w / det;
//
//        break;
//    default:
//        // 默认情况：速度清零
//        Chassis->Now_Chassis_speed.Vx = Chassis->Now_Chassis_speed.Vy = Chassis->Now_Chassis_speed.Vw = 0.0f;
//        break;
//    }
//}
/**
 * @brief 计算云台角度误差（相对于零点）
 * @param Chassis 底盘实例指针
 * @return 角度误差值（弧度）
 * @note 将角度误差限制在[-π, π]范围内
 * @date 2025-07-09
 */
static float Find_Angle(ChassisInstance_s *Chassis)
{
	float err = Chassis->gimbal_yaw_angle-Chassis->gimbal_yaw_zero;
          err = fmodf(err + 3.141593f, 2.0f * 3.141593f);
    if (err < 0){
        err += 2 *3.141593f;
    }
        err -= 3.141593f;
    return err;
}
/**
 * @brief 底盘功率限制函数 (未实现)
 * @note 预留功能，用于防止底盘超功率
 * @date 2025-07-03
 */
/**
 * @brief 底盘功率限制函数 (未实现)
 * @note 预留功能，用于防止底盘超功率
 * @date 2025-07-03
 */
float  wheel_scale;
float target_torque1[8];
float target_torque2[8];
float total_wheel_power = 0.0f;
static bool Chassis_Power_Limit(ChassisInstance_s* Chassis, float power_buffer)
{
    if (Chassis == NULL || Chassis->Chassis_power_limit <= 0 || Chassis->super_chassis_active == 1 ) {
        return false;
    }
    else
    {
        float power_max = Chassis->Chassis_power_limit;
        //if (power_max < 60.0f) {
        //    power_max = 60.0f;
        //}
        float total_steering_power = 0.0f;
        float motor_target_power[8] = {0};
        float steering_scale = 1.0f;
        // 舵组功率限制,只有填写了舵向功率分配系数才有用
        if (Chassis->omni_steering_message.Steering_Ratio != 0) {
            for (int i = 4; i < 8; i++) {
                float x = Chassis->chassis_motor[i]->output;
                float y = Chassis->chassis_motor[i]->message.rotor_velocity;
                Chassis->motor_power[i] = 1.421e-5f * x * y
                    + Chassis->motor_loss_config[i].K1 * x * x
                    + Chassis->motor_loss_config[i].K2 * y * y
                    + Chassis->motor_loss_config[i].Ka;
                total_steering_power += Chassis->motor_power[i];
            }
            float steering_scale = Chassis->omni_steering_message.Steering_Ratio * power_max / total_steering_power;
            if (steering_scale <= 1.0f && steering_scale >= 0.0f) {
                for (int i = 4; i < 8; i++) {
                    motor_target_power[i] = steering_scale * Chassis->motor_power[i];
                    float a = Chassis->motor_loss_config[i].K1;
                    float b = 1.421e-5f * Chassis->chassis_motor[i]->message.rotor_velocity;
                    float c = Chassis->motor_loss_config[i].K2 * Chassis->chassis_motor[i]->message.rotor_velocity * Chassis->chassis_motor[i]->message.rotor_velocity
                            + Chassis->motor_loss_config[i].Ka - motor_target_power[i];
                    if (b * b - 4 * a * c > 0.0f) {
                        target_torque1[i] = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
                        target_torque2[i] = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a);
                        if (target_torque1[i] * Chassis->chassis_motor[i]->message.torque_current > 0) {
                            if (fabsf(target_torque1[i]) > fabsf(Chassis->chassis_motor[i]->output))
                            {
                                Motor_Dji_Control(Chassis->chassis_motor[i], sqrtf(wheel_scale)*Chassis->out_speed[i]);
                            }
                            else
                            {
                                Motor_Dji_SetCurrent(Chassis->chassis_motor[i], 2.0f*target_torque1[i]);
                            }
                        } else {
                            if (fabsf(target_torque1[i]) > fabsf(Chassis->chassis_motor[i]->output))
                            {
                                Motor_Dji_Control(Chassis->chassis_motor[i], sqrtf(wheel_scale)*Chassis->out_speed[i]);
                            }
                            else
                            {
                                Motor_Dji_SetCurrent(Chassis->chassis_motor[i], 2.0f*target_torque1[i]);
                            }
                        }
                    }
                }
            }
        }

        // 轮组功率限制
        total_wheel_power = 0.0f;
        for (int i = 0; i < 4; i++) {
            float x = Chassis->chassis_motor[i]->output;
            float y = Chassis->chassis_motor[i]->message.rotor_velocity;
            Chassis->motor_power[i] = 1.996e-6f * x * y
                + Chassis->motor_loss_config[i].K1 * x * x
                + Chassis->motor_loss_config[i].K2 * y * y
                + Chassis->motor_loss_config[i].Ka;
            total_wheel_power += Chassis->motor_power[i];
        }
        wheel_scale = (power_max - steering_scale * total_steering_power) /total_wheel_power;
        if (wheel_scale < 1.0f && wheel_scale > 0.0f) {
            if (Chassis->Chassis_speed.Vx > 0.1f||Chassis->Chassis_speed.Vy > 0.1f)
            {
                Chassis->Chassis_speed.Vw = 1.0f;
                Chassis_IK_Calc(Chassis);
            }
            for (int i = 0; i < 4; i++)
            {
                    motor_target_power[i] = wheel_scale * Chassis->motor_power[i];
                    float a = Chassis->motor_loss_config[i].K1;
                    float b = 1.996e-6f * Chassis->chassis_motor[i]->message.rotor_velocity;
                    float c = Chassis->motor_loss_config[i].K2 * Chassis->chassis_motor[i]->message.rotor_velocity * Chassis->chassis_motor[i]->message.rotor_velocity
                            + Chassis->motor_loss_config[i].Ka - motor_target_power[i];
                    if (b * b - 4 * a * c > 0.0f){
                        target_torque1[i] = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
                        target_torque2[i] = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a);
                        if (target_torque1[i] * Chassis->chassis_motor[i]->output > 0) {
                            if (fabsf(target_torque1[i]) > fabsf(Chassis->chassis_motor[i]->output))
                            {
                                Motor_Dji_Control(Chassis->chassis_motor[i], sqrtf(wheel_scale)*Chassis->out_speed[i]);
                            }
                            else
                            {
                                Motor_Dji_SetCurrent(Chassis->chassis_motor[i], target_torque1[i]);
                            }
                        } else {
                            if (fabsf(target_torque2[i]) > fabsf(Chassis->chassis_motor[i]->output))
                            {
                                Motor_Dji_Control(Chassis->chassis_motor[i], sqrtf(wheel_scale)*Chassis->out_speed[i]);
                            }
                            else
                            {
                                Motor_Dji_SetCurrent(Chassis->chassis_motor[i], target_torque2[i]);
                            }
                        }
                    }
                    else if (b * b - 4 * a * c  == 0.0f)
                    {
                        float target_torque = -b / 2 * a;
                        if (fabsf(target_torque) > fabsf(Chassis->chassis_motor[i]->output))
                        {
                            Motor_Dji_Control(Chassis->chassis_motor[i], sqrtf(wheel_scale)*Chassis->out_speed[i]);
                        }
                        else
                        {
                            Motor_Dji_SetCurrent(Chassis->chassis_motor[i], target_torque);
                        }
                    }
                }
            }
            return true;
        }
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
        Chassis->Chassis_speed.Vw = 0.0f;
        break;

    case CHASSIS_GYROSCOPE:  // 小陀螺模式
        Chassis->Chassis_speed.Vw = Chassis->Gyroscope_Speed;
        break;
    default:
        break;
    }
    // 2. 坐标系转换
    Temp_Speed.Vx = Chassis->Chassis_speed.Vx *cosf(Find_Angle(Chassis)) - Chassis->Chassis_speed.Vy * sinf(Find_Angle(Chassis));
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
    Chassis_Power_Limit(Chassis,0);//功率限制函数(未实现)
    Motor_Dji_Transmit(Chassis->chassis_motor[0]);
    if(Chassis->type == Steering_Wheel)
    {
    for(uint8_t i = 4; i < 8; i++)
    {
    Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_angle[i-4]);
    }
    Motor_Dji_Transmit(Chassis->chassis_motor[4]);
    };
		//Chassis_FK_Calc(Chassis);
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