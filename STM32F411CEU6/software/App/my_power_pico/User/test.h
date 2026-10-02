#ifndef TEST_H
#define TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 执行STEP6C的一次性硬件启动流程
 *
 * 由HardwareInitTask调用，负责：
 * 1. 加载EEPROM中的校准参数；
 * 2. 恢复量程模式；
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
