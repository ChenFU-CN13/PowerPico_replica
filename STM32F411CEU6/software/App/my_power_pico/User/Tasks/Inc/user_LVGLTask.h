#ifndef __USER_LVGLTASK_H__
#define __USER_LVGLTASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LVGL任务入口
 * @param argument 未使用
 */
void LvHandlerTask(void *argument);

/**
 * @brief 给LVGL增加1ms软件时基
 *
 * 本函数由FreeRTOS Tick Hook在中断上下文调用，
 * 里面不能延时、不能刷屏、不能访问SPI。
 */
void TaskTickHook(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_LVGLTASK_H__ */

