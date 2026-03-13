#include "alg_chassis_power_control.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define CHASSIS_POWER_CONTROL_EPSILON (1.0e-6f)

typedef struct {
    uint32_t count;
    float root0;
    float root1;
} ChassisPowerControlRoots_s;

static float Chassis_PowerControl_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Chassis_PowerControl_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float Chassis_PowerControl_Evaluate(const ChassisPowerControlModel_s *model, float current, float omega)
{
    return model->k0 + model->k1 * current + model->k2 * omega + model->k3 * current * omega + model->k4 * current * current + model->k5 * omega * omega;
}

static ChassisPowerControlRoots_s Chassis_PowerControl_Solve_Quadratic(float a, float b, float c)
{
    ChassisPowerControlRoots_s roots;
    roots.count = 0U;
    roots.root0 = 0.0f;
    roots.root1 = 0.0f;

    if (Chassis_PowerControl_Abs(a) <= CHASSIS_POWER_CONTROL_EPSILON) {
        if (Chassis_PowerControl_Abs(b) <= CHASSIS_POWER_CONTROL_EPSILON) {
            return roots;
        }
        roots.count = 1U;
        roots.root0 = -c / b;
        return roots;
    }

    {
        float discriminant = b * b - 4.0f * a * c;
        if (discriminant < -CHASSIS_POWER_CONTROL_EPSILON) {
            return roots;
        }
        if (discriminant < 0.0f) {
            discriminant = 0.0f;
        }
        if (discriminant <= CHASSIS_POWER_CONTROL_EPSILON) {
            roots.count = 1U;
            roots.root0 = -b / (2.0f * a);
            return roots;
        }

        discriminant = sqrtf(discriminant);
        roots.count = 2U;
        roots.root0 = (-b + discriminant) / (2.0f * a);
        roots.root1 = (-b - discriminant) / (2.0f * a);
    }

    return roots;
}

static float Chassis_PowerControl_Select_Closest_Root(const ChassisPowerControlRoots_s *roots, float origin_current)
{
    if (roots->count == 0U) {
        return 0.0f;
    }
    if (roots->count == 1U) {
        return roots->root0;
    }

    if (Chassis_PowerControl_Abs(roots->root0 - origin_current) <= Chassis_PowerControl_Abs(roots->root1 - origin_current)) {
        return roots->root0;
    }
    return roots->root1;
}

static ChassisPowerControlStatus_e Chassis_PowerControl_Select_Scale(const ChassisPowerControlRoots_s *roots, float *scale)
{
    float selected_scale = 0.0f;
    bool has_valid_scale = false;

    if (scale == NULL) {
        return CHASSIS_POWER_CONTROL_STATUS_INVALID_ARGUMENT;
    }

    if (roots->count >= 1U && roots->root0 >= -CHASSIS_POWER_CONTROL_EPSILON && roots->root0 <= 1.0f + CHASSIS_POWER_CONTROL_EPSILON) {
        selected_scale = Chassis_PowerControl_Clamp(roots->root0, 0.0f, 1.0f);
        has_valid_scale = true;
    }

    if (roots->count >= 2U && roots->root1 >= -CHASSIS_POWER_CONTROL_EPSILON && roots->root1 <= 1.0f + CHASSIS_POWER_CONTROL_EPSILON) {
        float candidate_scale = Chassis_PowerControl_Clamp(roots->root1, 0.0f, 1.0f);
        if (!has_valid_scale || candidate_scale > selected_scale) {
            selected_scale = candidate_scale;
        }
        has_valid_scale = true;
    }

    *scale = selected_scale;
    return has_valid_scale ? CHASSIS_POWER_CONTROL_STATUS_OK : CHASSIS_POWER_CONTROL_STATUS_NO_VALID_SCALE;
}

static void Chassis_PowerControl_Reset_Result(ChassisPowerControlResult_s *result,
                                              ChassisPowerControlMethod_e method,
                                              uint8_t motor_count,
                                              float limit_power)
{
    if (result == NULL) {
        return;
    }

    memset(result, 0, sizeof(ChassisPowerControlResult_s));
    result->method = method;
    result->status = CHASSIS_POWER_CONTROL_STATUS_OK;
    result->motor_count = motor_count;
    result->limit_power = limit_power;
    result->scale_factor = 1.0f;
}

static ChassisPowerControlStatus_e Chassis_PowerControl_Apply_Method(const ChassisPowerControlGroupConfig_s *group_config,
                                                                     DjiMotorInstance_s **motors,
                                                                     uint8_t motor_count,
                                                                     float limit_power,
                                                                     ChassisPowerControlResult_s *result)
{
    float origin_power[CHASSIS_POWER_CONTROL_MAX_GROUP_MOTORS];
    float limited_output[CHASSIS_POWER_CONTROL_MAX_GROUP_MOTORS];
    uint8_t index;

    if (group_config == NULL || motors == NULL) {
        return CHASSIS_POWER_CONTROL_STATUS_INVALID_ARGUMENT;
    }

    if (motor_count == 0U) {
        Chassis_PowerControl_Reset_Result(result, group_config->method, 0U, limit_power);
        return CHASSIS_POWER_CONTROL_STATUS_OK;
    }

    if (motor_count > CHASSIS_POWER_CONTROL_MAX_GROUP_MOTORS) {
        return CHASSIS_POWER_CONTROL_STATUS_COUNT_EXCEEDED;
    }

    Chassis_PowerControl_Reset_Result(result, group_config->method, motor_count, limit_power);

    for (index = 0U; index < motor_count; ++index) {
        if (motors[index] == NULL) {
            return CHASSIS_POWER_CONTROL_STATUS_INVALID_ARGUMENT;
        }

        origin_power[index] = Chassis_PowerControl_Evaluate(&group_config->model, motors[index]->output, motors[index]->message.rotor_velocity);
        result->total_origin_power += origin_power[index];
    }

    if (group_config->method == CHASSIS_POWER_CONTROL_METHOD_DISABLED || result->total_origin_power <= limit_power) {
        result->total_limited_power = result->total_origin_power;
        return CHASSIS_POWER_CONTROL_STATUS_OK;
    }

    if (group_config->method == CHASSIS_POWER_CONTROL_METHOD_POWER_ATTENUATION) {
        float regenerative_power = 0.0f;
        float consuming_power = 0.0f;
        ChassisPowerControlStatus_e status = CHASSIS_POWER_CONTROL_STATUS_OK;

        for (index = 0U; index < motor_count; ++index) {
            if (origin_power[index] <= 0.0f) {
                regenerative_power += origin_power[index];
            } else {
                consuming_power += origin_power[index];
            }
        }

        if (consuming_power <= CHASSIS_POWER_CONTROL_EPSILON) {
            result->scale_factor = 0.0f;
        } else {
            result->scale_factor = Chassis_PowerControl_Clamp((limit_power - regenerative_power) / consuming_power, 0.0f, 1.0f);
        }

        for (index = 0U; index < motor_count; ++index) {
            float target_power = origin_power[index];
            float omega = motors[index]->message.rotor_velocity;
            float a;
            float b;
            float c;
            ChassisPowerControlRoots_s roots;

            if (origin_power[index] > 0.0f) {
                target_power = origin_power[index] * result->scale_factor;
            }

            a = group_config->model.k4;
            b = group_config->model.k1 + group_config->model.k3 * omega;
            c = group_config->model.k0 + group_config->model.k2 * omega + group_config->model.k5 * omega * omega - target_power;
            roots = Chassis_PowerControl_Solve_Quadratic(a, b, c);

            if (roots.count == 0U) {
                limited_output[index] = 0.0f;
                result->fallback_motor_count++;
                status = CHASSIS_POWER_CONTROL_STATUS_NO_REAL_ROOT;
            } else {
                limited_output[index] = Chassis_PowerControl_Select_Closest_Root(&roots, motors[index]->output);
            }

            Motor_Dji_SetCurrent(motors[index], limited_output[index]);
            result->total_limited_power += Chassis_PowerControl_Evaluate(&group_config->model, motors[index]->output, omega);
        }

        result->status = status;
        return status;
    }

    if (group_config->method == CHASSIS_POWER_CONTROL_METHOD_CURRENT_ATTENUATION) {
        float a = 0.0f;
        float b = 0.0f;
        float c = -limit_power;
        float scale = 1.0f;
        ChassisPowerControlStatus_e status = CHASSIS_POWER_CONTROL_STATUS_OK;

        for (index = 0U; index < motor_count; ++index) {
            float current = motors[index]->output;
            float omega = motors[index]->message.rotor_velocity;
            a += group_config->model.k4 * current * current;
            b += group_config->model.k1 * current + group_config->model.k3 * current * omega;
            c += group_config->model.k0 + group_config->model.k2 * omega + group_config->model.k5 * omega * omega;
        }

        {
            ChassisPowerControlRoots_s roots = Chassis_PowerControl_Solve_Quadratic(a, b, c);
            status = Chassis_PowerControl_Select_Scale(&roots, &scale);
            if (status != CHASSIS_POWER_CONTROL_STATUS_OK) {
                scale = 0.0f;
            }
        }

        result->scale_factor = scale;
        result->status = status;

        for (index = 0U; index < motor_count; ++index) {
            limited_output[index] = motors[index]->output * scale;
            Motor_Dji_SetCurrent(motors[index], limited_output[index]);
            result->total_limited_power += Chassis_PowerControl_Evaluate(&group_config->model, motors[index]->output, motors[index]->message.rotor_velocity);
        }

        return status;
    }

    return CHASSIS_POWER_CONTROL_STATUS_INVALID_ARGUMENT;
}

bool Chassis_PowerControl_Init(ChassisPowerControlInstance_s *instance, const ChassisPowerControlConfig_s *config)
{
    if (instance == NULL || config == NULL) {
        return false;
    }

    memset(instance, 0, sizeof(ChassisPowerControlInstance_s));
    instance->config = *config;
    return true;
}

bool Chassis_PowerControl_Apply(ChassisPowerControlInstance_s *instance,
                                DjiMotorInstance_s **wheel_motors,
                                uint8_t wheel_motor_count,
                                DjiMotorInstance_s **steering_motors,
                                uint8_t steering_motor_count,
                                float base_power_limit,
                                PidInstance_s *power_limit_pid,
                                float power_buffer)
{
    float total_limit;
    float steering_limit = 0.0f;
    float wheel_limit;

    if (instance == NULL) {
        return false;
    }

    if (!instance->config.enabled) {
        return true;
    }

    total_limit = base_power_limit;
    if (power_limit_pid != NULL && instance->config.power_buffer_target > 0.0f) {
        total_limit += Pid_Calculate(power_limit_pid, instance->config.power_buffer_target, power_buffer);
    }
    total_limit = Chassis_PowerControl_Clamp(total_limit, 0.0f, FLT_MAX);

    instance->last_total_limit = total_limit;
    instance->last_steering_limit = 0.0f;
    instance->last_wheel_limit = total_limit;

    if (steering_motor_count > 0U && instance->config.steering_group.method != CHASSIS_POWER_CONTROL_METHOD_DISABLED) {
        steering_limit = total_limit * Chassis_PowerControl_Clamp(instance->config.steering_power_ratio, 0.0f, 1.0f);
        instance->last_steering_limit = steering_limit;
        if (Chassis_PowerControl_Apply_Method(&instance->config.steering_group,
                                              steering_motors,
                                              steering_motor_count,
                                              steering_limit,
                                              &instance->steering_result) == CHASSIS_POWER_CONTROL_STATUS_INVALID_ARGUMENT) {
            return false;
        }
    } else {
        Chassis_PowerControl_Reset_Result(&instance->steering_result,
                                          instance->config.steering_group.method,
                                          steering_motor_count,
                                          0.0f);
    }

    wheel_limit = total_limit - instance->steering_result.total_limited_power;
    if (wheel_limit < 0.0f) {
        wheel_limit = 0.0f;
    }
    instance->last_wheel_limit = wheel_limit;

    if (wheel_motor_count > 0U && instance->config.wheel_group.method != CHASSIS_POWER_CONTROL_METHOD_DISABLED) {
        if (Chassis_PowerControl_Apply_Method(&instance->config.wheel_group,
                                              wheel_motors,
                                              wheel_motor_count,
                                              wheel_limit,
                                              &instance->wheel_result) == CHASSIS_POWER_CONTROL_STATUS_INVALID_ARGUMENT) {
            return false;
        }
    } else {
        Chassis_PowerControl_Reset_Result(&instance->wheel_result,
                                          instance->config.wheel_group.method,
                                          wheel_motor_count,
                                          wheel_limit);
    }

    return true;
}
