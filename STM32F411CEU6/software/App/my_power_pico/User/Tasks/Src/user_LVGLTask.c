/* Private includes -----------------------------------------------------------*/
//includes
#include "user_LVGLTask.h"
#include "user_TasksInit.h"

#include "main.h"
#include "tim.h"
#include "cmsis_os.h"

#include "lvgl.h"
#include "lv_port_disp.h"
#include "ui.h"
#include "key.h"


#include <stdbool.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/*
 * LVGL初始化完成后才允许Tick Hook调用lv_tick_inc()。
 *
 * FreeRTOS调度器可能先产生Tick，
 * 然后LvHandlerTask才得到运行机会。
 */
static volatile bool s_lvgl_tick_ready;


/**
 * @brief 给LVGL增加1ms软件时基
 */
void TaskTickHook(void)
{
    if (s_lvgl_tick_ready)
    {
        lv_tick_inc(1);
    }
    
}




/**
 * @brief 初始化并独占运行LVGL
 * @param argument 未使用
 */
void LvHandlerTask(void *argument)
{
    PowerData_T received_data;                 //接收到的数据
    PowerData_T latest_data = {0};              //最新数据
    bool has_new_data;                          //是否有新数据
    uint32_t lvgl_wait_ms;                      //等待时间

    key_event_t key_event;                      //按键事件

    (void)argument;


    /*
     * 1. 等待HardwareInitTask完成。
     *
     * 必须先加载EEPROM校准参数、初始化自动量程并启动ADC，
     * 然后再启动正式UI。
     */
    while (User_Tasks_GetStatus()
           != USER_TASKS_STATUS_READY)
    {
        osDelay(1U);
    }

    /*
     * 2. 初始化LVGL核心。
     * 从这里开始，FreeRTOS Tick Hook才允许推进LVGL时间。
     */
    lv_init();
    s_lvgl_tick_ready = true;

    /*
     * 3. 启动TIM1_CH3背光PWM。
     * LCD_Set_Light()只修改比较值，
     * 所以必须先启动PWM通道。
     */
    if (HAL_TIM_PWM_Start(&htim1,
                          TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 4. 初始化ST7789显示端口。
     * lv_port_disp_init()内部会：
     * - 初始化LCD；
     * - 创建240×240显示设备；
     * - 注册SPI DMA刷新回调；
     * - 配置RGB565字节交换；
     * - 关闭背光，等待启动动画渐亮。
     */
    lv_port_disp_init();

    /*
     * 5. 创建官方启动页和主页。
     */
    ui_init();

    for (;;)
    {
        has_new_data = false;
        /*
         * 6. 获取PowerDataQueue和Key_MessageQueue 中的数据。
         *
         * 队列里如果积压多组数据，只留下最后一组，
         * 避免主页依次显示已经过期的数据。
         */    
        
        if (PowerDataQueue != NULL)
        {
            while (osMessageQueueGet(PowerDataQueue,
                                    &received_data,
                                    NULL,
                                    0) == osOK)
            {
                latest_data = received_data;
                has_new_data = true;
            }

        }
        
        /*物理按键
            ↓
            Key_Scan()
            ↓
            KeyTask
            ↓
            Key_MessageQueue
            ↓
            LvHandlerTask
            ↓
            lv_lib_pm_handle_key_event()
            ↓
            当前页面的key_event_handler()*/
        if (Key_MessageQueue != NULL)
        {
            while (osMessageQueueGet(Key_MessageQueue,&key_event,NULL,0) == osOK)
            {
                lv_lib_pm_handle_key_event(&key_event);
            }
            
        }
        


        /*
         * 7. 更新UI缓存。
         *
         * 注意：
         * 这里只更新普通变量，不直接修改Label。
         * 主页的250ms定时器会统一刷新控件。
         */
        if (has_new_data)
        {
            ui_update_vol_cur_varables(latest_data.voltage_v,latest_data.current_ua);
        }
      
        /*
         * 8. 执行LVGL定时器、动画和重绘。
         */
        lvgl_wait_ms = lv_timer_handler();

        /*
         * 9. 把等待时间限制在1～30ms。
         *
         * 不能长期不调用lv_timer_handler，
         * 否则动画、主页刷新和DMA刷新都会停顿。
         */
        if (lvgl_wait_ms == 0U)
        {
            lvgl_wait_ms = 1U;
        }
        else if (lvgl_wait_ms > 30U)
        {
            lvgl_wait_ms = 30U;
        }

        osDelay(lvgl_wait_ms);
    }
    

    
}


