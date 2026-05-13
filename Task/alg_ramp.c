#include "alg_ramp.h"
#include "cmsis_os.h"
#include <stddef.h>

float GenerateSimpleRampWithPause(float start, float end, int steps, uint32_t interval_ms, RampWithPauseState_s *state)
{
    uint32_t current_time = osKernelSysTick(); // 获取当前系统时间

    if (state == NULL) {
        return start;
    }

    // 1. 初始化或重置
    if (!state->initialized || start != state->last_start || end != state->last_end || steps != state->last_steps) {
        state->target_setpoint = start;
        state->current_step = 0;
        state->last_update_time = current_time;

        state->last_start = start;
        state->last_end = end;
        state->last_steps = steps;
        state->initialized = 1;
    }

    // 2. 检查是否到达更新时间
    if (current_time - state->last_update_time >= interval_ms)
    {
        // 如果还没有到达最后一步
        if (state->current_step < steps)
        {
            state->current_step++; // 移动到下一步
            state->last_update_time = current_time; // 更新时间戳
        }
    }
    
    // 3. 计算当前的目标设定点
    if (steps > 0) {
        state->target_setpoint = start + (end - start) * ((float)state->current_step / (float)steps);
    } else {
        state->target_setpoint = start; // 如果步数为0，则目标点始终为起点
    }

    // 确保最终目标点不会超过终点
    if ((end > start && state->target_setpoint > end) || (end < start && state->target_setpoint < end)) {
        state->target_setpoint = end;
    }

    // 4. 返回当前计算出的目标值
    return state->target_setpoint;
}

float GenerateReversingRamp(float min_pos, float max_pos, int steps, uint32_t interval_ms, uint32_t pause_ms, ReversingRampState_s *state)
{
    uint32_t current_time = osKernelSysTick();

    if (state == NULL) {
        return min_pos;
    }

    // 1. 仅在第一次调用时进行初始化
    if (!state->initialized
        || state->last_min_pos != min_pos
        || state->last_max_pos != max_pos
        || state->last_steps != steps
        || state->last_interval_ms != interval_ms
        || state->last_pause_ms != pause_ms) {
        state->target_setpoint = min_pos;
        state->current_step = 0;
        state->is_forward_trip = 1;
        state->is_paused = 0;
        state->last_update_time = current_time;
        state->last_min_pos = min_pos;
        state->last_max_pos = max_pos;
        state->last_steps = steps;
        state->last_interval_ms = interval_ms;
        state->last_pause_ms = pause_ms;
        state->initialized = 1;
    }

    // 2. 状态机逻辑
    if (state->is_paused) // 如果当前处于暂停状态
    {
        if (current_time - state->last_update_time >= pause_ms)
        {
            // 暂停结束，准备“掉头”
            state->is_forward_trip = (uint8_t)!state->is_forward_trip; // 切换方向
            state->current_step = 0;                                 // 重置步数
            state->is_paused = 0;                                    // 退出暂停状态
            state->last_update_time = current_time;                  // 更新时间戳，开始新的运动
        }
    }
    else // 如果当前处于运动状态
    {
        if (current_time - state->last_update_time >= interval_ms)
        {
            if (state->current_step < steps)
            {
                state->current_step++; // 移动到下一步
                state->last_update_time = current_time;
            }
            
            if (state->current_step >= steps)
            {
                // 到达端点，开始暂停
                state->is_paused = 1;
                state->last_update_time = current_time; // 重置暂停计时器
            }
        }
    }

    // 3. 根据当前状态计算目标点
    float start_pos = state->is_forward_trip ? min_pos : max_pos;
    float end_pos = state->is_forward_trip ? max_pos : min_pos;

    if (steps > 0) {
        state->target_setpoint = start_pos + (end_pos - start_pos) * ((float)state->current_step / (float)steps);
    } else {
        state->target_setpoint = start_pos;
    }

    // 4. 返回当前计算出的目标值
    return state->target_setpoint;
}

#ifdef GRAVITY_COMP_RECORD
#include <string.h>

void GravityCompTargetGenerator_Init(GravityCompTargetGenerator_s *gen,
                                     float min, float max,
                                     int steps,
                                     uint32_t interval_ms,
                                     uint32_t pause_ms)
{
    gen->min_pos     = min;
    gen->max_pos     = max;
    gen->steps       = steps;
    gen->interval_ms = interval_ms;
    gen->pause_ms    = pause_ms;
    gen->done        = 0;
    gen->last_pos    = min;
    memset(&gen->ramp, 0, sizeof(gen->ramp));
}

float GravityCompTargetGenerator_Update(GravityCompTargetGenerator_s *gen)
{
    // 生成极慢的往复斜坡
    float pos = GenerateReversingRamp(
        gen->min_pos, gen->max_pos,
        gen->steps,
        gen->interval_ms,
        gen->pause_ms,
        &gen->ramp
    );

    // 更新当前的稳态暂存
    gen->last_pos = pos;

    // 不再主动将 done 设为 1，使得整个扫描过程永远往复循环
    return pos;
}

#endif /* GRAVITY_COMP_RECORD */