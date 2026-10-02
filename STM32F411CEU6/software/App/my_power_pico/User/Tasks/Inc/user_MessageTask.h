#ifndef __USER_MESSAGETASK_H__
#define __USER_MESSAGETASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 处理ADC数据并向PowerDataQueue发布最新测量结果
 * @param argument 未使用
 */
void MessageSendTask(void *argument);

/**
 * @brief 从CmdRxQueue读取USB数据并交给命令解析器
 * @param argument 未使用
 */
void MessageReceiveTask(void *argument);

/**
 * @brief 在USB接收回调中把本次数据复制到CmdRxQueue
 * @param data USB本次收到的数据地址
 * @param length USB本次收到的字节数
 * @return true 已成功入队；false 数据无效、超长或队列已满
 * @note 只能进行非阻塞调用，不能在USB中断上下文等待队列空位。
 */
bool MessageReceive_EnqueueFromISR(const uint8_t *data,
                                   uint32_t length);

/**
 * @brief 通知MessageSendTask释放当前USB发送缓冲区
 *
 * @note
 * 本函数由USB发送完成、CDC初始化或CDC反初始化回调调用。
 * 除了正常发送完成，USB复位或重新枚举也表示旧发送缓冲区
 * 不再由上一轮CDC会话占用。
 * 回调运行在中断上下文，所以这里只允许置任务标志，
 * 不能等待、延时、拼包或继续发送下一帧。
 */
void MessageSend_NotifyTxCompleteFromISR(void);

#ifdef __cplusplus
}
#endif

#endif
