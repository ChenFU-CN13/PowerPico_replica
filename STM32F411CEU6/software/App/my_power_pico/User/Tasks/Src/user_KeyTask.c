/* Private includes -----------------------------------------------------------*/
//includes
#include "user_TasksInit.h"
#include "main.h"
#include "key.h"
#include "beep.h"
#include "BL24C02.h" // settings

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


/**
 * @brief 周期扫描按键，把按键事件交给 LVGL 任务处理。
 * @param argument 未使用。
 * @return 无。
 * @note 本任务只采集输入和控制按键音，不直接切换页面。
 */
void KeyTask(void *argument)
{
	key_event_t key_event; // 保存 Key_Scan() 输出的按键编号和事件类型。
	while(1)
	{
		if(Key_Scan(&key_event)) // 返回 true 表示本轮产生了 DOWN、CLICK、LONG 等事件。
		{
			osMessageQueuePut(Key_MessageQueue, &key_event, 0, 1); // 交给 LvHandlerTask 做页面按键分发。
			if(Sys_Get_KeySoundEnable()) // 用户设置允许按键音时，蜂鸣 50 ms。
			{
				beep_open();  // 打开 TIM4 PWM 输出。
				osDelay(50);  // 当前任务阻塞，其他任务仍可运行。
				beep_close(); // 关闭 PWM，结束按键提示音。
			}
		}
		osDelay(10); // 约 10 ms 扫描一次，底层 key.c 再完成消抖和长按判断。
	}
}
