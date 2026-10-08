/**
 * @file user_HardwareInitTask.c
 * @brief 一次性硬件启动任务
 */


/* Private includes -----------------------------------------------------------*/
#include "user_HardwareInitTask.h"
#include "user_TasksInit.h"
#include "BL24C02.h"
#include "gate.h"
#include "user_AdcDataStrategy.h"
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
 * 1. 加载EEPROM设置和校准参数；
 * 2. 恢复用户量程模式并复位自动量程；
 * 3. 启动ADC DMA和TIM2触发；
 * 4. 初始化按键并关闭蜂鸣器；
 * 5. 保持CC_EN断开并初始化FUSB302；
 * 6. 标记硬件初始化完成；
 * 7. 删除当前任务。
 */
void HardwareInitTask(void *argument)
{
    (void)argument;

    /*
     * 第一步：从EEPROM读取校准参数和系统设置。
     * 如果EEPROM内容无效，加载函数会继续使用程序内的默认值。
     */
    (void)EEPROM_SysSetting_Load();

    /*
     * 第二步：恢复上次保存的量程模式。
     * main()中已经提前把真实硬件切到HIGH安全档。
     * 这里切换的只是软件控制模式，不会立即改变当前硬件档位。
     */
    Gate_Set_Mode(Sys_Get_CurrentRangeMode());

    /*
     * 第三步：根据当前真实档位重新初始化自动量程状态，
     * 同时清空升档、降档的连续确认计数。
     */
    ADC_AutoRange_Reset();

    /*
     * 第四步：先启动ADC DMA，再启动TIM2触发。
     * 启动失败由ADC模块保留错误状态，本任务不额外保存临时调试变量。
     */
    (void)ADC_Sampling_Start();

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
