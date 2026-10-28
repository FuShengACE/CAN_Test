//
// Created by 86183 on 2026/10/24.
//

#ifndef MOTOR6623_H
#define MOTOR6623_H

#include <stdint.h>
#include "PID.h"

typedef struct
{
    uint16_t encoder_raw_prev; /* 上一帧编码器原始角度 */
    uint16_t encoder_raw;      /* 当前编码器原始角度，0~8191 */
    int32_t  turns;         /* 圈数 */
    int64_t  speed_count_prev; /* 上一个测速窗口的累计编码器计数 */
    int32_t  speed_count_diff; /* 最近一个测速窗口的计数差 */
    volatile uint32_t speed_update_seq; /* 新测速结果序号，由CAN接收更新 */
    uint32_t speed_pid_seq;             /* 速度PID已处理的测速序号 */
    float    angle;         /* 当前展开角度，rad */
    float    speed_filtered; /* 对原始速度低通后的速度，rpm */
    float    speed_raw;      /* 未滤波速度，rpm */
    float    dt_ms;          /* 实际测速窗口时长，单位ms */
    uint64_t time_prev_us;   /* 上一帧 CAN 反馈时间 */
    uint8_t  initialized;    /* 是否已经收到第一帧有效反馈 */
    float    current;       /* A */
    float    current_command; /* 电调反馈的转矩电流给定，A */
    float    current_sent;    /* 主控实际发送的电流给定，A */

    uint8_t  mode;          /* 0=电流 1=速度 2=位置 3=串级 */
    float    target;        /* 速度模式为 rpm，位置模式为 rad，电流模式为 A */

    PID      position_pid;  /* 位置环PID */
    PID      speed_pid;     /* 速度环PID */

    uint64_t last_feedback_us;
} Motor6623;

void Motor_Init(Motor6623 *m);
void Motor_Parse(Motor6623 *m, uint8_t *buf);          /* 0x205 的 8 字节 */
void Motor_Update_1kHz(Motor6623 *m);        /* PID + 下发 */

#ifdef __cplusplus
extern "C" {
#endif
void Motor_Set_Mode(Motor6623 *m, uint8_t mode, float target);
#ifdef __cplusplus
}
#endif

#endif
