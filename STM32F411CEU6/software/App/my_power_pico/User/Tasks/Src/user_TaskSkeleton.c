/**
 * @file user_TaskSkeleton.c
 * @brief STEP6B暂未接入正式业务的任务骨架。
 *
 * STEP6E中USB接收任务已经迁移到user_MessageTask.c。
 * 本文件目前只保留尚未接入业务的按键和PD任务骨架。
 */
/* Private includes -----------------------------------------------------------*/
//includes
#include "user_TaskSkeleton.h"
#include "user_TasksInit.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/*
 * 当前不会设置这个标志。
 * 任务进入后会阻塞，不会空转占用CPU。
 */
#define STEP6B_ACTIVATE_FLAG    (1UL << 0)

/* Private variables ---------------------------------------------------------*/


/* Timers --------------------------------------------------------------------*/



/* Tasks ---------------------------------------------------------------------*/


/**
 * @brief 让尚未接入业务的任务保持阻塞
 */
static void Step6B_WaitForMigration(void)
{
    for (;;)
    {
        (void)osThreadFlagsWait(
            STEP6B_ACTIVATE_FLAG,
            osFlagsWaitAny,
            osWaitForever);
    }
}


/**
 * @brief 按键任务骨架
 */
void KeyTask(void *argument)
{
    (void)argument;
    Step6B_WaitForMigration();
}

/**
 * @brief PD协议任务骨架
 */
void PDUFPTask(void *argument)
{
    (void)argument;
    Step6B_WaitForMigration();
}
