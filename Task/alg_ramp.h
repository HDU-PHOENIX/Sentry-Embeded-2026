#ifndef ALG_RAMP_H
#define ALG_RAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
//#define GRAVITY_COMP_RECORD
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

#ifdef GRAVITY_COMP_RECORD
#define G_FEED_TEST
typedef struct {
    ReversingRampState_s ramp;              /**< 内部往复斜坡状态机 */
    uint8_t  done;                          /**< 1 = 扫描完成停机 */
    float    min_pos;                       /**< 扫描范围下限 */
    float    max_pos;                       /**< 扫描范围上限 */
    int      steps;                         /**< 单程步数 */
    uint32_t interval_ms;                   /**< 步进间隔(ms) */
    uint32_t pause_ms;                      /**< 端点暂停(ms) */
    float    last_pos;                      /**< 当前区间的暂存位置 */
} GravityCompTargetGenerator_s;

/**
 * @brief 初始化重力补偿标定用斜坡生成器
 * @param gen    实例指针
 * @param min    扫描下限 (rad)
 * @param max    扫描上限 (rad)
 * @param steps  单程步数（建议 50~200，越多越慢越精细）
 * @param interval_ms  步进间隔（建议 ≥100ms）
 * @param pause_ms     端点暂停（建议 ≥1000ms）
 */
void GravityCompTargetGenerator_Init(GravityCompTargetGenerator_s *gen,
                                     float min, float max,
                                     int steps,
                                     uint32_t interval_ms,
                                     uint32_t pause_ms);

/**
 * @brief 在控制循环中周期性调用
 * @param gen    实例指针
 * @return 斜坡生成的目标位置
 * @note 当 gen->done == 1 后返回终点位置维持，不会继续扫描
 */
float GravityCompTargetGenerator_Update(GravityCompTargetGenerator_s *gen);

#endif /* GRAVITY_COMP_RECORD */

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