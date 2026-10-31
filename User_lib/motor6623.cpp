//
// Created by 86183 on 2026/10/24.
//

#include "motor6623.h"
#include "drv_can.h"
#include "sys_timestamp.h"
#include "main.h"
#include <math.h>

#define TWO_PI 6.2831853f
#define CONTROL_DT 0.001f
#define ENCODER_COUNTS_PER_TURN 8192.0f
#define SPEED_UPDATE_PERIOD_US 5000ULL
#define SPEED_CONTROL_DT (SPEED_UPDATE_PERIOD_US * 1.0e-6f)
#define SPEED_FILTER_TAU 0.10f
#define COUNT_TO_RPM  (60.0f / 8192.0f)
#define MOTOR_DIR (-1.0f)

/* 当前正向旋转测试得到的单圈速度波动补偿。反向不启用。 */
#define FORWARD_SPEED_COMP_ENABLE 1
#define FORWARD_SPEED_COMP_RATIO  0.089f
#define FORWARD_SPEED_COMP_PHASE  (-0.4f)

/* 设置电流 */

static void Send_Current(int16_t ma)
{
    ma = (int16_t)(MOTOR_DIR * (float)ma);

    uint8_t d[8] = {0};
    d[2] = (uint8_t)((uint16_t)ma >> 8);
    d[3] = (uint8_t)ma;
    CAN_Transmit_Data(&hfdcan1, 0x1FF, d, 8);
}

/* 电机初始化 */

void Motor_Init(Motor6623 *m)
{
    m->encoder_raw_prev = 0;
    m->encoder_raw = 0;
    m->turns = 0;
    m->speed_count_prev = 0;
    m->speed_count_diff = 0;
    m->speed_update_seq = 0;
    m->position_pid_seq = 0;
    m->speed_pid_seq = 0;
    m->speed_filtered = 0.0f;
    m->speed_raw = 0.0f;
    m->dt_ms = 0.0f;
    m->time_prev_us = 0;
    m->initialized = 0;
    m->current = 0;
    m->current_command = 0.0f;
    m->current_sent = 0.0f;
    m->mode = 0;
    m->target = 0;
    m->last_feedback_us = SYS_Timestamp.Get_Current_Timestamp();

    PID_Init(&m->speed_pid, 0.032f, 0.3f, 0.0f, 2.7f, 5.0f, SPEED_CONTROL_DT);
    PID_Init(&m->position_pid, 0.0075f, 0.0f,
             0.000002f, 2.5f, 5.0f, CONTROL_DT);
    /* 串级位置外环：编码器计数输入，目标速度 rpm 输出。 */
    PID_Init(&m->cascade_position_pid, 0.10f, 0.0f, 0.0f,
             0.0f, 200.0f, SPEED_CONTROL_DT);
}



/* 电机状态更新 */

void Motor_Parse(Motor6623 *m, uint8_t *buf)
{
    uint64_t now_us = SYS_Timestamp.Get_Current_Timestamp();
    uint16_t encoder_raw = (uint16_t)((buf[0] << 8) | buf[1]); //当前计数
    if (encoder_raw > 8191) return;                           //多的扔掉
    int16_t  cur = (int16_t)((buf[2] << 8) | buf[3]);       //当前电流
    m->encoder_raw = encoder_raw;

    if (m->initialized) {
        int32_t diff = (int32_t)encoder_raw - (int32_t)m->encoder_raw_prev; //角度差
        if (diff < -4096)       //计算圈数
            m->turns++;
        else if (diff > 4096)
            m->turns--;
    }

    m->encoder_raw_prev = encoder_raw;
    int64_t count_total = (int64_t)m->turns * 8192LL + encoder_raw;
    /* 第一帧只建立计数和时间基准，不参与测速。 */
    if (!m->initialized)
    {
        m->initialized = 1;
        m->speed_count_prev = count_total;
        m->speed_count_diff = 0;
        m->time_prev_us = now_us;
        m->speed_filtered = 0.0f;
        m->speed_raw = 0.0f;
        m->dt_ms = 0.0f;
    }
    else
    {
        /* 每个约 5 ms 的测速窗口，用实际 dt 对原始角度差分测速。 */
        uint64_t dt_us = now_us - m->time_prev_us;

        if (dt_us >= SPEED_UPDATE_PERIOD_US && dt_us < 100000ULL)
        {
            float dt = (float)dt_us * 1.0e-6f;
            int64_t count_diff = count_total - m->speed_count_prev;
            m->speed_count_diff = (int32_t)count_diff;
            m->speed_raw = (float)count_diff * COUNT_TO_RPM / dt;

            /* 只对当前正向旋转方向补偿，speed_raw 保留原始测速值。 */
            float speed_for_filter = m->speed_raw;
#if FORWARD_SPEED_COMP_ENABLE
            if (count_diff > 0)
            {
                float encoder_phase =
                    (float)encoder_raw * (TWO_PI / 8192.0f);
                float speed_error =
                    m->speed_raw * FORWARD_SPEED_COMP_RATIO *
                    cosf(encoder_phase + FORWARD_SPEED_COMP_PHASE);
                speed_for_filter -= speed_error;
            }
#endif

            /* 对速度进行一阶低通，滤波时间常数为 100 ms。 */
            float alpha = 1.0f - expf(-dt / SPEED_FILTER_TAU);
            if (alpha < 0.0f) alpha = 0.0f;
            if (alpha > 1.0f) alpha = 1.0f;
            m->speed_filtered +=
                alpha * (speed_for_filter - m->speed_filtered);
            m->dt_ms = (float)dt_us * 0.001f;

            /* 完成一个测速窗口后才移动基准点。 */
            m->speed_count_prev = count_total;
            m->time_prev_us = now_us;
            m->speed_update_seq++;
        }
        else if (dt_us >= 100000ULL)
        {
            /* 长时间没有有效反馈时重新建立测速基准。 */
            m->speed_count_prev = count_total;
            m->time_prev_us = now_us;
            m->speed_raw = 0.0f;
            m->speed_filtered = 0.0f;
            m->dt_ms = 0.0f;
            m->speed_count_diff = 0;
        }
    }
    m->current = MOTOR_DIR * (cur / 1000.0f);//毫安转安
    m->current_command = MOTOR_DIR *(int16_t)((buf[4] << 8) | buf[5]) / 1000.0f;
    m->last_feedback_us = now_us;
}

/* 读状态 下发控制 */

void Motor_Update_1kHz(Motor6623 *m)
{
    uint64_t now = SYS_Timestamp.Get_Current_Timestamp();

    if (now - m->last_feedback_us > 100000ULL)
    {
        m->current_sent = 0.0f;
        Send_Current(0);
        return;
    }

    uint32_t speed_seq = m->speed_update_seq;
    bool speed_updated = (speed_seq != m->speed_pid_seq);
    bool position_updated = (speed_seq != m->position_pid_seq);
    bool speed_pid_updated = false;
    float out = 0.0f;
    switch (m->mode)
    {
        case 0: out = m->target; break;
        case 1:
            if (speed_updated && m->dt_ms > 0.0f)
            {
                /* 速度 PID 使用实际测速窗口，而不是固定的 1 ms。 */
                m->speed_pid.dt = m->dt_ms * 0.001f;
                PID_Update_Limited(&m->speed_pid,
                                   m->target,
                                   m->speed_filtered,
                                   -5.0f,
                                   5.0f);
                speed_pid_updated = true;
            }
            out = m->speed_pid.out;
            break;
        case 2:
            if (position_updated && m->dt_ms > 0.0f)
            {
                /* 位置 PID 与测速窗口同步，使用实际反馈时间间隔。 */
                m->position_pid.dt = m->dt_ms * 0.001f;
                PID_Update(&m->position_pid,
                           m->target, (float)m->encoder_raw);
                m->position_pid_seq = speed_seq;
            }
            out = m->position_pid.out;
            break;
        case 3:
        {
            if (speed_updated && m->dt_ms > 0.0f)
            {
                /* 位置外环和速度内环都按新的测速结果更新。 */
                m->cascade_position_pid.dt = m->dt_ms * 0.001f;
                float speed_target =
                    PID_Update_Limited(&m->cascade_position_pid,
                                       m->target,
                                       (float)m->encoder_raw,
                                       -200.0f,
                                       200.0f);

                /* 速度 PID 使用实际测速窗口，而不是固定的 1 ms。 */
                m->speed_pid.dt = m->dt_ms * 0.001f;
                PID_Update_Limited(&m->speed_pid,
                                   speed_target,
                                   m->speed_filtered,
                                   -5.0f,
                                   5.0f);
                speed_pid_updated = true;
            }
            out = m->speed_pid.out;
            break;
        }
    }

    if (speed_pid_updated)
    {
        m->speed_pid_seq = speed_seq;
    }

    if (out >  5.0f) out =  5.0f;
    if (out < -5.0f) out = -5.0f;

    m->current_sent = out * MOTOR_DIR;
    Send_Current((int16_t)(out * 1000.0f));
}

void Motor_Set_Mode(Motor6623 *m, uint8_t mode, float target)
{
    bool mode_changed = (m->mode != mode);

    m->mode = mode;
    m->target = target;

    if (mode_changed)
    {
        PID_Reset(&m->position_pid, (float)m->encoder_raw);
        PID_Reset(&m->cascade_position_pid, (float)m->encoder_raw);
        m->position_pid_seq = m->speed_update_seq;
        PID_Reset(&m->speed_pid, m->speed_filtered);
        m->speed_pid_seq = m->speed_update_seq;
    }
}
