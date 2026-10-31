#include "main.h"
#include "tim.h"
#include "usart.h"
#include "drv_can.h"
#include "app.h"
#include "sys_timestamp.h"
#include <stdio.h>
#include "motor6623.h"

/* 创建结构体 */
Motor6623 motor;


/* ---------- 接收回调 ---------- */
void App_CAN_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer) {
    if (Header.IdType != FDCAN_STANDARD_ID) return;
    if (Header.RxFrameType != FDCAN_DATA_FRAME) return;
    if (Header.DataLength < 8) return;

    if (Header.Identifier == 0x206)
    {
        Motor_Parse(&motor, Buffer);
    }
}


void App_CAN_Init(void)
{
    SYS_Timestamp.Init(&htim5);
    HAL_TIM_Base_Start_IT(&htim5);

    Motor_Init(&motor);
    Motor_Set_Mode(&motor, 2, 0.0f);

    CAN_Init(&hfdcan1, App_CAN_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    HAL_Delay(2000);

    Motor_Set_Mode(&motor, 2, 2000.0f);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
    }
    else if (htim->Instance == TIM6)
    {
        Motor_Update_1kHz(&motor);
    }
}


/* 执行任务 */

void App_Log_Task(void)
{
    char buf[160];
    float pid_output = (motor.mode == 2) ?
                       motor.position_pid.out : motor.speed_pid.out;
    int n = snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                     (float)motor.encoder_raw,
                     motor.speed_filtered,
                     motor.speed_raw,
                     motor.dt_ms,
                     motor.current,
                     motor.current_command,
                     motor.current_sent,
                     pid_output,
                     motor.target);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 10);
}

void App_Task_Speed_Step(void)
{
    static uint32_t last_step_ms = 0;
    static bool started = false;
    static bool high_speed = true;

    uint32_t now = HAL_GetTick();

    /* 从进入主循环开始计时，保证初始位置保持完整 7 秒。 */
    if (!started)
    {
        last_step_ms = now;
        started = true;
        return;
    }

    if ((uint32_t)(now - last_step_ms) < 7000U)
        return;

    last_step_ms = now;
    high_speed = !high_speed;

    Motor_Set_Mode(&motor, 1, high_speed ? 200.0f : 100.0f);
}

void App_Task_Position_Step(void)
{
    static uint32_t last_step_ms = 0;
    static bool started = false;
    static uint8_t step_index = 0;
    static const float position_targets[] =
        {2000.0f, 4000.0f, 6000.0f, 2000.0f};

    uint32_t now = HAL_GetTick();

    if (!started)
    {
        last_step_ms = now;
        started = true;
        Motor_Set_Mode(&motor, 2, position_targets[step_index]);
        return;
    }

    if ((uint32_t)(now - last_step_ms) < 7000U)
        return;

    last_step_ms = now;
    step_index++;
    if (step_index >= 4)
        step_index = 0;

    Motor_Set_Mode(&motor, 2, position_targets[step_index]);
}
