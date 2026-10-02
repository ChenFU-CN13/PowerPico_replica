#include "test.h"
#include "BL24C02.h"
#include "gate.h"
#include "user_AdcDataStrategy.h"

/**
 * @brief 执行STEP6C的一次性硬件启动流程
 *
 * 该函数由HardwareInitTask调用。
 * 必须先准备校准参数和量程状态，最后才能启动ADC采样。
 */
void Step6C_HardwareInit_Run(void)
{
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
     * 启动失败由ADC模块保留错误状态，本层不再保存临时显示变量。
     */
    (void)ADC_Sampling_Start();
}
