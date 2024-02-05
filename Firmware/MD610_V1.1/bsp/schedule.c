#include "schedule.h"
#include "control.h"

#include "board.h"

#include "parameters_conversion.h"
#include "mc_tasks.h"
#include "main.h"
#include "hqfoc_app.h"

void HQ_Loop_1000Hz(void) // 1ms执行一次
{
    float loop_time_1000hz = Get_Cycle_T(1); /*获取5ms准确时间*/
    UNUSED(loop_time_1000hz);

}

void HQ_Loop_500Hz(void) // 2ms执行一次
{
    float loop_time_500hz;
    loop_time_500hz = Get_Cycle_T(2); /*获取5ms准确时间*/
    UNUSED(loop_time_500hz);
}

void HQ_Loop_200Hz(void) // 5ms执行一次
{
    float loop_time_200hz;
    loop_time_200hz = Get_Cycle_T(3); /*获取5ms准确时间*/
    UNUSED(loop_time_200hz);
}

void HQ_Loop_100Hz(void) // 10ms执行一次
{
    float loop_time_100hz = Get_Cycle_T(4);
    UNUSED(loop_time_100hz);
}

void HQ_Loop_50Hz(void) // 20ms执行一次
{
    float loop_time_50hz = Get_Cycle_T(5);
    UNUSED(loop_time_50hz);
}

uint8_t drv_fault = 0;
void HQ_Loop_20Hz(void) // 50ms执行一次
{
    static u8 timer_50ms = 0; // 记录50ms次数
    float loop_time_20hz = Get_Cycle_T(5);
    UNUSED(loop_time_20hz);
	
	drv_fault = drv_fault_check();	//nFAULT will go high when the gate driver is ready for PWM inputs during start-up.
    if (++timer_50ms > 10)
    {
        timer_50ms = 0;
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
		
    }
}

volatile float Cycle_T[GET_TIME_NUM][3];

float Get_Cycle_T(u8 item)
{
    Cycle_T[item][OLD] = Cycle_T[item][NOW];                          // 上一次的时间
    Cycle_T[item][NOW] = GetSysTime_us() / 1000000.0f;                // 本次的时间
    Cycle_T[item][NEW] = ((Cycle_T[item][NOW] - Cycle_T[item][OLD])); // 间隔的时间（周期）
    return Cycle_T[item][NEW];
}

void HQ_Cycle_Time_Init(void)
{
    u8 i;
    for (i = 0; i < GET_TIME_NUM; i++)
    {
        Get_Cycle_T(i);
    }
}


#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY / 1000)
void HQ_SYSTICK_IRQHandler(void)
{
    static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
    static schedule infantrySchedule;

    if (SystickDividerCounter == SYSTICK_DIVIDER)
    {
        sysTickUptime++;
        infantrySchedule.cnt_1ms++;
        infantrySchedule.cnt_2ms++;
        infantrySchedule.cnt_5ms++;
        infantrySchedule.cnt_10ms++;
        infantrySchedule.cnt_20ms++;
        infantrySchedule.cnt_50ms++;

        if (Init_OK)
            HQ_Loop(&infantrySchedule);
        SystickDividerCounter = 0;
    }
    SystickDividerCounter++;
}

void HQ_Loop(schedule *robotSchdule)
{
    if (robotSchdule->cnt_1ms >= 1)
    {
        HQ_Loop_1000Hz();
        robotSchdule->cnt_1ms = 0;
    }
    if (robotSchdule->cnt_2ms >= 2)
    {
        HQ_Loop_500Hz();
        robotSchdule->cnt_2ms = 0;
    }
    if (robotSchdule->cnt_5ms >= 5)
    {
        HQ_Loop_200Hz();
        robotSchdule->cnt_5ms = 0;
    }
    if (robotSchdule->cnt_10ms >= 10)
    {
        HQ_Loop_100Hz();
        robotSchdule->cnt_10ms = 0;
    }
    if (robotSchdule->cnt_20ms >= 20)
    {
        HQ_Loop_50Hz();
        robotSchdule->cnt_20ms = 0;
    }
    if (robotSchdule->cnt_50ms >= 50)
    {
        HQ_Loop_20Hz();
        robotSchdule->cnt_50ms = 0;
    }
}