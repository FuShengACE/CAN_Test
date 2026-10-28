//
// Created by 86183 on 2026/10/24.
//

#include "PID.h"
#include <stdbool.h>

void PID_Init(PID *p, float kp, float ki, float kd, float i_limit, float out_limit, float dt)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->i_limit = i_limit;
    p->out_limit = out_limit;
    p->dt = dt;
    PID_Reset(p, 0.0f);
}

void PID_Reset(PID *p, float meas)
{
    p->integral = 0.0f;
    p->prev_meas = meas;
    p->d_state = 0.0f;
    p->out = 0.0f;
}

float PID_Update_Limited(PID *p, float target, float meas,
                         float output_min, float output_max)
{
    float err = target - meas;

    float integral_candidate = p->integral + p->ki * err * p->dt;
    if (integral_candidate >  p->i_limit) integral_candidate =  p->i_limit;
    if (integral_candidate < -p->i_limit) integral_candidate = -p->i_limit;

    float d = (meas - p->prev_meas) / p->dt;
    p->d_state += 0.2f * (d - p->d_state);

    float unsaturated = p->kp * err + integral_candidate - p->kd * p->d_state;
    bool saturating_low = (unsaturated < output_min) && (err < 0.0f);
    bool saturating_high = (unsaturated > output_max) && (err > 0.0f);

    /* 输出饱和且误差仍推动输出向饱和方向时，暂停积分。 */
    if (!saturating_low && !saturating_high)
        p->integral = integral_candidate;

    float out = p->kp * err + p->integral - p->kd * p->d_state;
    if (out > output_max) out = output_max;
    if (out < output_min) out = output_min;

    p->prev_meas = meas;
    p->out = out;
    return out;
}

float PID_Update(PID *p, float target, float meas)
{
    return PID_Update_Limited(p, target, meas,
                              -p->out_limit, p->out_limit);
}
