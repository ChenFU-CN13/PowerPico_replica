#ifndef TEST_H
#define TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 执行Step 2硬件测试
 * @note  需要在TIM1、TIM5和FreeRTOS调度器启动后，由任务上下文调用一次。
 */
void Step2_Test_Run(void);
/**
 * @brief 启动STEP3的ADC+DMA测试
 */
void Step3_Test_Run(void);

/**
 * @brief 刷新STEP3测试数据显示
 */
void Step3_Test_Update(void);

/**
 * @brief 启动Step 4手动档换算测试
 */
void Step4_Test_Run(void);

/**
 * @brief 刷新Step 4测量页面
 */
void Step4_Test_Update(void);

/**
 * @brief 初始化STEP6E的LCD观察页面
 */
void Step6E_Test_Run(void);

/**
 * @brief 使用PowerDataQueue中的数据刷新STEP6E测试页面
 * @param current_ua 队列发送的电流值，单位uA
 * @param sample_range 产生该电流数据时的量程
 */
void Step6E_Test_Update(float current_ua,
                        uint8_t sample_range);

/**
 * @brief 刷新STEP6E USB接收链路计数
 * @note 由LvHandlerTask每200ms调用一次，不依赖ADC是否产生新数据。
 */
void Step6E_Test_UpdateUsb(void);

/**
 * @brief 执行STEP6C的一次性硬件启动流程
 *
 * 由HardwareInitTask调用，负责：
 * 1. 加载EEPROM中的校准参数；
 * 2. 设置自动量程模式；
 * 3. 复位自动量程状态机；
 * 4. 启动ADC DMA循环采样。
 *
 * @note 只能在任务上下文中调用一次。
 */
void Step6C_HardwareInit_Run(void);

#ifdef __cplusplus
}
#endif

#endif
