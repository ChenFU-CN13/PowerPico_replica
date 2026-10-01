/**
 * @file user_HardwareInitTask.c
 * @brief STEP6C一次性硬件启动任务
 */


/* Private includes -----------------------------------------------------------*/
#include "user_HardwareInitTask.h"
#include "user_TasksInit.h"
#include "test.h"
#include "key.h"
#include "beep.h"
#include "main.h"
#include "fusb302_dev.h"
// includes
// sys


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


/**
 * @brief 完成系统启动后必须执行的一次性初始化
 * @param argument 当前未使用
 *
 * 执行顺序：
 * 1. 加载校准参数；
 * 2. 设置和复位自动量程；
 * 3. 启动ADC DMA；
 * 4. 通知LCD初始化已经完成；
 * 5. 删除当前任务。
 */
 void HardwareInitTask(void *argument)
 {
   
    (void)argument;

     /*
     * 真正的硬件启动过程。
     * 函数返回时，ADC要么已经启动成功，
     * 要么s_step5_start_status已经记录失败状态。
     */
    Step6C_HardwareInit_Run();
    /* 初始化五个物理按键，确保蜂鸣器默认关闭 */
    Key_Init();
    beep_close();
    /*
    * STEP9-1：
    * 只初始化FUSB302，不连接外部CC，不进行PD协商。
    */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    (void)fusb302_dev_init();
    /*
     * 该标志表示HardwareInitTask已经执行完毕。
     * LCD上的BOK依靠这个状态产生。
     */
    User_Tasks_MarkHardwareInitDone();
    /*
     * HardwareInitTask只运行一次。
     *
     * 当前CMSIS-RTOS2适配层中的osThreadExit()
     * 最终会调用FreeRTOS的vTaskDelete(NULL)。
     */
    osThreadExit();
    
}
