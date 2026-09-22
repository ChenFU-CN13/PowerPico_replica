#ifndef __USER_TASKSINIT_H__
#define __USER_TASKSINIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>


/*
 * USB命令队列暂时沿用官方数据尺寸。
 * STEP6B只创建队列，不修改USB接收回调。
 */
#define CMD_RX_CHUNK_SIZE      64U
#define CMD_RX_QUEUE_DEPTH     8U

/*
 * MessageSendTask使用的USB发送事件标志。
 *
 * bit0、bit1已经被ADC_FLAG_BLOCK_READY和ADC_FLAG_ERROR使用，
 * 所以USB相关事件从bit2开始。
 */
#define FLAG_CMD_TX_READY        (1UL << 2)  // CmdTxQueue中有命令回复
#define FLAG_USB_TX_COMPLETE     (1UL << 3)  // 上一帧USB数据发送完成


#define CMD_TX_FRAME_SIZE      58U
#define CMD_TX_QUEUE_DEPTH     4U

/**
 * @brief MessageSendTask发送给UI的测量数据
 *
 * STEP6D由MessageSendTask写入PowerDataQueue，
 * LvHandlerTask读取后刷新LCD。
 */
typedef struct
{
    float voltage_v;       // 输入电压，单位V
    float current_ua;      // 电流，单位uA
    uint8_t range;         // 产生本次数据时的量程
    uint8_t reserved[3];   // 保留并补齐结构体对齐
} PowerData_T;

/**
 * @brief USB接收队列中的一块原始数据
 */
typedef struct
{
    uint16_t length;                    // 本次收到的有效字节数
    uint8_t data[CMD_RX_CHUNK_SIZE];    // USB原始数据
} CmdRxChunk_T;

/**
 * @brief 等待MessageSendTask发送的完整回复帧
 */
typedef struct
{
    uint16_t length;                    // 回复帧有效长度
    uint8_t data[CMD_TX_FRAME_SIZE];    // 回复帧内容
} CmdTxFrame_T;

/**
 * @brief UI发送给PD任务的命令
 *
 * 本轮只确定消息格式，不启动PD协议。
 */
typedef enum
{
    PD_CMD_START = 0,
    PD_CMD_STOP,
    PD_CMD_SET_PPS,
    PD_CMD_SET_FIXED
} PDCommand_T;


/**
 * @brief 一条完整的PD控制命令
 */
typedef struct
{
    PDCommand_T event;       // 启动、停止、PPS或固定档命令
    float voltage_v;         // PPS目标电压
    float current_a;         // PPS目标电流
    uint8_t fixed_level;     // 固定PDO档位编号
    uint8_t reserved[3];     // 结构体对齐
} PDCommandMsg_T;

/**
 * @brief 用户任务框架状态
 */
typedef enum
{
    USER_TASKS_STATUS_WAIT = 0,     // 对象已创建，等待一次性任务执行
    USER_TASKS_STATUS_READY,        // 队列、任务及一次性任务全部正常
    USER_TASKS_STATUS_CREATE_FAIL   // 至少一个对象创建失败
} UserTasksStatus_T;

/*
 * 创建失败位图。
 * 如果LCD显示BERR，可以根据十六进制值定位具体对象。
 */
#define USER_TASK_ERR_KEY_QUEUE       (1UL << 0)
#define USER_TASK_ERR_PD_CMD_QUEUE    (1UL << 1)
#define USER_TASK_ERR_PD_EVT_QUEUE    (1UL << 2)
#define USER_TASK_ERR_POWER_QUEUE     (1UL << 3)
#define USER_TASK_ERR_CMD_RX_QUEUE    (1UL << 4)
#define USER_TASK_ERR_CMD_TX_QUEUE    (1UL << 5)

#define USER_TASK_ERR_HW_TASK         (1UL << 8)
#define USER_TASK_ERR_MSG_SEND_TASK   (1UL << 9)
#define USER_TASK_ERR_MSG_RECV_TASK   (1UL << 10)
#define USER_TASK_ERR_KEY_TASK        (1UL << 11)
#define USER_TASK_ERR_PD_TASK         (1UL << 12)
#define USER_TASK_ERR_LVGL_TASK       (1UL << 13)







/* 用户任务句柄 */
extern osThreadId_t HardwareInitTaskHandle;
extern osThreadId_t MessageSendTaskHandle;
extern osThreadId_t MessageReceiveTaskHandle;
extern osThreadId_t KeyTaskHandle;
extern osThreadId_t PDUFPTaskHandle;
extern osThreadId_t LvHandlerTaskHandle;



/* 六条业务消息队列 */
extern osMessageQueueId_t Key_MessageQueue;
extern osMessageQueueId_t PD_cmd_MessageQueue;
extern osMessageQueueId_t PD_handle_event_MsgQueue;
extern osMessageQueueId_t PowerDataQueue;
extern osMessageQueueId_t CmdRxQueue;
extern osMessageQueueId_t CmdTxQueue;

/**
 * @brief 创建当前阶段使用的任务和消息队列
 * @note 必须在osKernelStart()之前调用一次
 */
void User_Tasks_Init(void);

/**
 * @brief 获取任务框架当前状态
 */
UserTasksStatus_T User_Tasks_GetStatus(void);

/**
 * @brief 获取创建失败位图
 */
uint32_t User_Tasks_GetErrorMask(void);

/**
 * @brief 标记一次性HardwareInitTask已经运行
 * @note 只给HardwareInitTask调用
 */
void User_Tasks_MarkHardwareInitDone(void);

/* 各任务入口 */
void HardwareInitTask(void *argument);
void MessageSendTask(void *argument);
void MessageReceiveTask(void *argument);
void KeyTask(void *argument);
void PDUFPTask(void *argument);





#ifdef __cplusplus
}
#endif

#endif
