#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

void App_CAN_Init(void);
void App_Log_Task(void);
void App_Task_Speed_Step(void);
void App_Task_Position_Step(void);

#ifdef __cplusplus
}
#endif

#endif
