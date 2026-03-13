# chassis_calc的使用文档
## 概述
chassis_calc是一个用于控制底盘的解算程序，提供了基础的麦克纳姆轮，全向轮的解算后的正常移动，底盘跟随，小陀螺等功能。
## 注意事项
1. 前置库文件依赖性：
    - `dev_motor_dji`：用于电机控制
    - `bsp_can`：用于CAN总线通信
    - `alg_pid`：用于PID控制
    - 注意正方向在1.4号电机
    - 本解算程序采用右手系,x轴正方向为前侧,y轴正方向为左方,且要求轮子正转为逆时针方向
    - 轮子电调为id为1——4
    - 为了实现坐标系转换的正确和底盘跟随,需要在云台task中将yaw轴电机编码值发给底盘实例的gimbal_yaw_angle(如果仅测试底盘能动性,则可无视)
    - 先调好使用陀螺仪下yaw轴的pid后再使用小陀螺与跟随模式
    - 如果发现底盘未注册成功,请检查config配置
2. 电机id数据现已删除,会根据电机rxid自动配置
3. 速度为云台坐标系的速度,使用时传给absolute_chassis_speed这个结构体
## 使用例程(一种config的方式)
### 全向轮
```C
ChassisInstance_s *Chassis;
extern GimbalInstance_s *Gimbal;
	static ChassisInitConfig_s Chassis_config = {
		.type = Omni_Wheel,
		.gimbal_yaw_zero = 1.82637596f,//-0.0312073231f,
		.omni_message={
		.wheel_radius= 0.0765f,
	  .chassis_radius= 0.230f,
		},    //如果这里是麦轮,则填mecanum_steering_message这个结构体里面的数据
		.gimbal_follow_pid_config={
		  .kp = -3.0f,
      .ki = 0.0f,
      .kd = 0.0f,
			.dead_zone = 0.2f,
      .i_max = 0.0f,
      .out_max = 2 * 3.141593f,
		},
		.motor_config[0] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
		.rx_id=0x201,	
		.tx_id=0x200,
    },
    .velocity_pid_config={
      .kp = 110.0f,
      .ki = 1.2f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[1] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "2",
    .can_config = {
    .can_number=1,
		.rx_id=0x202,	
		.tx_id=0x200,
    },
    .velocity_pid_config={
      .kp = 110.0f,
      .ki = 1.2f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[2] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "3",
    .can_config = {
    .can_number=1,
		.rx_id=0x203,	
		.tx_id=0x200,
		},
    .velocity_pid_config={
      .kp = 110.0f,
      .ki = 1.2f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[3] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .topic_name = "4",
   .can_config = {
    .can_number=1,
		.rx_id=0x204, 
		.tx_id=0x200,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 110.0f,
      .ki = 1.2f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  }
	};
void ChassisTask(void const * argument)
	{
	Chassis = Chassis_Register(&Chassis_config);
  for(;;){
    Chassis->gimbal_yaw_angle = Gimbal->yaw_motor->message.out_position;//这一步为将yaw轴云台编码值数据传给底盘    
    Chassis_Mode_Choose(Chassis,CHASSIS_NORMAL);
    Chassis_Control(Chassis);
     osDelay(1);
  }
  }

```
## 待开发

> 舵轮程序测试及完善

> 添加更多功率策略

## 功率控制挂载

当前目录新增了独立模块：

- `alg_chassis_power_control.h`
- `alg_chassis_power_control.c`

`alg_chassis_calc` 仅做接口融合，不直接在原文件里展开功率控制公式。

### 配置方法

在 `ChassisInitConfig_s` 中新增了 `power_control_config`，用于描述功率控制策略：

- `enabled`：是否启用功率控制
- `power_buffer_target`：功率缓冲目标值，会配合 `Chassis_power_limit_pid_config` 修正总功率上限
- `steering_power_ratio`：给舵向电机预留的功率比例
- `wheel_group`：轮向电机功率控制模型与方法
- `steering_group`：舵向电机功率控制模型与方法

其中 `wheel_group.method` / `steering_group.method` 支持：

- `CHASSIS_POWER_CONTROL_METHOD_DISABLED`
- `CHASSIS_POWER_CONTROL_METHOD_POWER_ATTENUATION`
- `CHASSIS_POWER_CONTROL_METHOD_CURRENT_ATTENUATION`

### 运行时更新

上层在循环中更新功率状态：

```c
Chassis_Update_Power_State(chassis, now_chassis_power, now_power_buffer);
Chassis_Control(chassis);
```

当前融合方式会在 `Chassis_Control` 中完成：

1. 底盘逆解
2. 驱动/舵向电机 PID 输出计算
3. 功率控制模块二次限幅
4. 发送 CAN

### 系数说明

功率模型默认使用如下形式：

`P = k0 + k1 * I + k2 * w + k3 * I * w + k4 * I^2 + k5 * w^2`

这里的 `I` 和 `w` 单位必须与工程实际输入保持一致：

- `I` 使用的是 `DjiMotorInstance_s.output` 的单位
- `w` 使用的是 `message.rotor_velocity` 的单位

因此在移植模型参数时，需要保证拟合时采用的输入单位与这里一致。



