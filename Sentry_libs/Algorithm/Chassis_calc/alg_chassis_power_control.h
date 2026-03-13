#ifndef __ALG_CHASSIS_POWER_CONTROL_H__
#define __ALG_CHASSIS_POWER_CONTROL_H__

#include "dev_motor_dji.h"
#include "alg_pid.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_POWER_CONTROL_MAX_GROUP_MOTORS 4U

typedef enum {
    CHASSIS_POWER_CONTROL_STATUS_OK = 0,
    CHASSIS_POWER_CONTROL_STATUS_INVALID_ARGUMENT = -1,
    CHASSIS_POWER_CONTROL_STATUS_COUNT_EXCEEDED = -2,
    CHASSIS_POWER_CONTROL_STATUS_NO_REAL_ROOT = -3,
    CHASSIS_POWER_CONTROL_STATUS_NO_VALID_SCALE = -4,
} ChassisPowerControlStatus_e;

typedef enum {
    CHASSIS_POWER_CONTROL_METHOD_DISABLED = 0,
    CHASSIS_POWER_CONTROL_METHOD_POWER_ATTENUATION = 1,
    CHASSIS_POWER_CONTROL_METHOD_CURRENT_ATTENUATION = 2,
} ChassisPowerControlMethod_e;

typedef struct {
    float k0;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
} ChassisPowerControlModel_s;

typedef struct {
    ChassisPowerControlMethod_e method;
    uint8_t motor_count;
    ChassisPowerControlModel_s model;
} ChassisPowerControlGroupConfig_s;

typedef struct {
    bool enabled;
    float power_buffer_target;
    float steering_power_ratio;
    ChassisPowerControlGroupConfig_s wheel_group;
    ChassisPowerControlGroupConfig_s steering_group;
} ChassisPowerControlConfig_s;

typedef struct {
    ChassisPowerControlMethod_e method;
    ChassisPowerControlStatus_e status;
    uint8_t motor_count;
    uint8_t fallback_motor_count;
    float limit_power;
    float total_origin_power;
    float total_limited_power;
    float scale_factor;
} ChassisPowerControlResult_s;

typedef struct {
    ChassisPowerControlConfig_s config;
    ChassisPowerControlResult_s wheel_result;
    ChassisPowerControlResult_s steering_result;
    float last_total_limit;
    float last_wheel_limit;
    float last_steering_limit;
} ChassisPowerControlInstance_s;

bool Chassis_PowerControl_Init(ChassisPowerControlInstance_s *instance, const ChassisPowerControlConfig_s *config);

bool Chassis_PowerControl_Apply(ChassisPowerControlInstance_s *instance,
                                DjiMotorInstance_s **wheel_motors,
                                uint8_t wheel_motor_count,
                                DjiMotorInstance_s **steering_motors,
                                uint8_t steering_motor_count,
                                float base_power_limit,
                                PidInstance_s *power_limit_pid,
                                float power_buffer);

#ifdef __cplusplus
}
#endif

#endif
