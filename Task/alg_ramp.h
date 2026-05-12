#ifndef ALG_RAMP_H
#define ALG_RAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target_setpoint;
    int current_step;
    uint32_t last_update_time;
    float last_start;
    float last_end;
    int last_steps;
    uint8_t initialized;
} RampWithPauseState_s;

typedef struct {
    float target_setpoint;
    int current_step;
    uint32_t last_update_time;
    uint8_t is_forward_trip; // 1: min->max, 0: max->min
    uint8_t is_paused;       // 1: paused, 0: moving
    float last_min_pos;
    float last_max_pos;
    int last_steps;
    uint32_t last_interval_ms;
    uint32_t last_pause_ms;
    uint8_t initialized;
} ReversingRampState_s;

/**
 * @brief 生成一个简单的斜坡轨迹，在start和end之间切分出若干个点，每隔固定的时间间隔切换到下一个点。
 */
float GenerateSimpleRampWithPause(float start, float end, int steps, uint32_t interval_ms, RampWithPauseState_s *state);

/**
 * @brief 生成一个在端点暂停并自动往复的斜坡信号。
 */
float GenerateReversingRamp(float min_pos, float max_pos, int steps, uint32_t interval_ms, uint32_t pause_ms, ReversingRampState_s *state);

#ifdef __cplusplus
}
#endif

#endif // ALG_RAMP_H