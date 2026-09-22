/* Private includes -----------------------------------------------------------*/
//includes

#include "user_LVGLTask.h"
#include "user_TasksInit.h"
#include "test.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 消费PowerDataQueue并刷新LCD
 * @param argument 未使用
 */
void LvHandlerTask(void *argument)
{
    PowerData_T received_data;
    PowerData_T latest_data = {0};
    bool has_new_data;

    (void)argument;

    /*
     * LCD只能由本任务初始化和访问。
     * 其他任务不能同时调用LCD绘图函数。
     */
    Step6E_Test_Run();

    for (;;)
    {
        has_new_data = false;

        /*
         * 一次刷新前把队列中的旧数据全部取完。
         * latest_data最终留下的是队列中最新的一组数据。
         *
         * 大白话：
         * LCD刷新慢没关系，旧数据跳过，直接显示最新值。
         */
        if (PowerDataQueue != NULL)
        {
            while (osMessageQueueGet(PowerDataQueue,
                                     &received_data,
                                     NULL,
                                     0U) == osOK)
            {
                latest_data = received_data;
                has_new_data = true;
            }
        }

        if (has_new_data)
        {
            Step6E_Test_Update(latest_data.current_ua,
                               latest_data.range);
        }

        /*
         * USB计数单独刷新，不依赖PowerDataQueue是否刚好有新数据。
         * 这样即使ADC链路异常，仍可观察USB接收任务是否正常工作。
         */
        Step6E_Test_UpdateUsb();

        /*
         * LCD约每200ms刷新一次。
         * 阻塞只影响显示任务，不影响ADC处理。
         */
        osDelay(200U);
    }
}
