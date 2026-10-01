/**
 * @file user_PDUFPTask.c
 * @brief 连接 UI 命令与底层 FUSB302/PD 状态机。
 *
 * 谁写谁读：UI 把启动、停止、固定档、PPS 设置写入 PD_cmd_MessageQueue；
 * 本任务执行命令并轮询 FUSB302；协商结果再经 PD_handle_event_MsgQueue 返回 UI。
 */
/* Private includes -----------------------------------------------------------*/
//includes
#include "user_TasksInit.h"
#include "user_PDUFPTask.h"
#include "main.h"
#include "fusb302_dev.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t _pd_working_status = 0;       // 0：CC 断开且状态机停用；1：PD 协商正在运行。
osTimerId_t PD_UFP_Task_timer_id;     // 每秒检查一次协商结果，最多等待 5 次。

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 每秒检查 PD 协商是否成功，或是否已经等待满 5 秒。
 * @param argument 未使用。
 * @return 无。
 * @note 回调只发 UI 事件和改变少量状态，不在这里跑完整协议。
 */
static void _timer_callback(void *argument)
{
    PD_handle_event_t ready;       // 准备发给 UI 的最终结果。
    static uint8_t time_count = 0; // 已检查次数；定时器周期为 1 秒。
    time_count++;                  // 每进一次回调，累计等待时间约增加 1 秒。
    if(is_PPS_ready()) {           // 底层状态机已收到 PPS 请求对应的 PS_RDY。
        ready = PD_EVT_PPS_READY;  // 通知 UI 进入 PPS 页面。
        osMessageQueuePut(PD_handle_event_MsgQueue, &ready, 0, 1); // 最多等待 1 tick 投递结果。
        time_count = 0;            // 为下次启动清零超时计数。
        osTimerStop(PD_UFP_Task_timer_id); // 成功后不再重复检查。
    } else if (is_PD_Fixed_ready()) {
        ready = PD_EVT_FIXED_READY; // 通知 UI 进入固定 PDO 页面。
        osMessageQueuePut(PD_handle_event_MsgQueue, &ready, 0, 1);
        time_count = 0;
        osTimerStop(PD_UFP_Task_timer_id);
    } else if(time_count >= 5) { // 5秒超时
        ready = PD_EVT_PPS_FAILED; // 当前事件名叫 PPS_FAILED，但固定档启动超时也会走这里。
        osMessageQueuePut(PD_handle_event_MsgQueue, &ready, 0, 1);
        time_count = 0;
        osTimerStop(PD_UFP_Task_timer_id);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // 拉低 CC_EN，让外部 Type-C CC 与 FUSB302 断开。
        _pd_working_status = 0; // 标记本轮结束，等待 UI 再次发送 START。
    }
}

/**
 * @brief 处理 UI 下发的 PD 命令，并驱动 FUSB302 事件状态机。
 * @param argument 未使用。
 * @return 无。
 */
void PDUFPTask(void *argument)
{
  // 快充诱骗时需要另外一个端口也供电，因为快充口会有可能断电
  // 可以检测 status_power 来判断是否有PPS供电
  PD_command_msg_t ui_msg; // 保存从 UI 队列取出的单条命令。
  PD_UFP_Task_timer_id = osTimerNew(_timer_callback, osTimerPeriodic, NULL, NULL); // 仅创建，START 时才启动。
	while(1)
	{
    if(osMessageQueueGet(PD_cmd_MessageQueue, &ui_msg, NULL, 0)==osOK) { // 非阻塞取命令，避免卡住 1 ms 轮询。
      if(!_pd_working_status && ui_msg.event == PD_CMD_START) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // 拉高 CC_EN，把外部 CC 连接到 FUSB302。
        PD_protocol_init(&app_pd.protocol);                  // 清空上一轮协议上下文和消息 ID。
        PD_protocol_set_power_option(&app_pd.protocol, PD_POWER_OPTION_MAX_5V); // 固定档默认不超过 5 V。
        PD_protocol_set_PPS(&app_pd.protocol, PPS_V(5.0), PPS_A(1.0), false);   // 同时准备 PPS 5 V/1 A 请求值。
        // send_power_request();
        if(osTimerIsRunning(PD_UFP_Task_timer_id)) {
            osTimerStop(PD_UFP_Task_timer_id);
        }
        osTimerStart(PD_UFP_Task_timer_id, 1000); // 每 1000 tick 检查一次结果。
        _pd_working_status = 1;                   // 从此开始执行底部的 FUSB302 轮询。

      } else if(_pd_working_status && ui_msg.event == PD_CMD_STOP) {
        // FUSB CC pin disconnect
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // 主动断开 CC。
        _pd_working_status = 0;                                // 停止协议事件处理。

      } else if(_pd_working_status && ui_msg.event == PD_CMD_SET_PPS) {
        // 设置PPS电压电流
        if(ui_msg.pps_set_voltage > 0 && ui_msg.pps_set_current > 0 && ui_msg.pps_set_voltage <= 20.0 && ui_msg.pps_set_current <= 5.0) {
          PD_protocol_set_PPS(&app_pd.protocol, PPS_V(ui_msg.pps_set_voltage), PPS_A(ui_msg.pps_set_current), false); // 转为协议单位并重选 APDO。
          send_power_request(); // 只置发送标志，真正组 Request 并发包在 fusb302_timer() 中完成。
        }
      } else if(_pd_working_status && ui_msg.event == PD_CMD_SET_PD_FIXED) {
        // 设置固定电压电流
        bool need_send = false;
        switch (ui_msg.pd_fixed_level)
        {
          case PD_FIXED_VOL_LEVEL_5V:
            need_send = PD_protocol_set_power_option(&app_pd.protocol, PD_POWER_OPTION_MAX_5V);
            break;
          case PD_FIXED_VOL_LEVEL_9V:
            need_send = PD_protocol_set_power_option(&app_pd.protocol, PD_POWER_OPTION_MAX_9V);
            break;
          case PD_FIXED_VOL_LEVEL_12V:
            need_send = PD_protocol_set_power_option(&app_pd.protocol, PD_POWER_OPTION_MAX_12V);
            break;
          case PD_FIXED_VOL_LEVEL_15V:
            need_send = PD_protocol_set_power_option(&app_pd.protocol, PD_POWER_OPTION_MAX_15V);
            break;
          case PD_FIXED_VOL_LEVEL_20V:
            need_send = PD_protocol_set_power_option(&app_pd.protocol, PD_POWER_OPTION_MAX_20V);
            break;
          default:
            break;
        }

        if (need_send) {       // 已收到 Source_Capabilities 时，改变档位会返回 true。
          send_power_request(); // 延后到统一的定时处理里发送 Request。
        }
      }
    }

    if(_pd_working_status) // 只有 CC 已接入且协议启用时才访问事件状态机。
    {
      if (fusb302_timer() || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
      {
          FUSB302_event_t FUSB302_events = 0; // 位图可同时包含连接、断开、收包等多个事件。
          // 定时轮询到期或 INT_N(PB4) 拉低时读中断寄存器；I2C 失败最多重试 3 次。
          for (uint8_t i = 0; i < 3 && FUSB302_alert(&fusb302_dev, &FUSB302_events) != FUSB302_SUCCESS; i++) {}
          if (FUSB302_events) {
              handle_FUSB302_event(FUSB302_events); // 处理连接、收包、GoodCRC 等事件。
          }
      }
    }
		osDelay(1); // 约 1 ms 执行一轮，避免空转占满 CPU。
	}
}
