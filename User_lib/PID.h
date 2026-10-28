//
// Created by 86183 on 2026/10/24.
//

#ifndef CAN_TEST_PID_H
#define CAN_TEST_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp, ki, kd;
    float i_limit;      /* 积分限幅（输出单位） */
    float out_limit;    /* 输出限幅 */
    float dt;           /* 控制周期 */
    float integral;     /* 积分值 */
    float prev_meas;    /* 上一次测量值 */
    float d_state;      /* 去噪后的测量值变化率 */
    float out;          /* 上一次的输出值 */
} PID;

void  PID_Init(PID *p, float kp, float ki, float kd, float i_limit, float out_limit, float dt);
void  PID_Reset(PID *p, float meas);
float PID_Update(PID *p, float target, float meas);
float PID_Update_Limited(PID *p, float target, float meas,
                         float output_min, float output_max);

#ifdef __cplusplus
}
#endif

#endif
