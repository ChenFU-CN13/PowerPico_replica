#ifndef __GATE_H__
#define __GATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"

// EN1
#define EN1_PORT    GPIOB
#define EN1_PIN     GPIO_PIN_3

// EN2
#define EN2_PORT    GPIOA
#define EN2_PIN     GPIO_PIN_15

// Flow route selection
/*这里定义了流路选择的枚举值，分别对应高电流、中电流、低电流3个档位的测量档位*/
#define HIGH_CUR 3
#define MID_CUR 2
#define LOW_CUR 1

// Range mode selection
/*这里定义了范围模式选择的枚举值，分别对应自动模式、低电流模式、中电流模式、高电流模式*/
#define GATE_MODE_AUTO 0
#define GATE_MODE_LOW  LOW_CUR
#define GATE_MODE_MID  MID_CUR
#define GATE_MODE_HIGH HIGH_CUR

/*补充说明：
1.HIGH_CUR 代表 当前测量量程是使用电阻R0，R1和R2被短路，范围是 50ma-5a，
2.MID_CUR 代表 当前测量量程是使用电阻R1和R2，R0被短路，范围是 500ua-50ma
3.LOW_CUR 代表 当前测量量程是使用电阻R0,R1,R2，电流流经3个电阻，范围是 1ua-500ua
*/


/*
 * @brief  初始化门的GPIO引脚
 * @param None
 * @retval None
 */
void Gate_Port_Init(void);

/*
 * @brief  选择流路
 * @param selection: 选择的流路，LOW_CUR、MID_CUR、HIGH_CUR
 * @retval None
 */
void flow_route_selection(uint8_t selection);

/*
 * @brief  获取门的当前状态
 * @param None
 * @retval 门的当前状态，LOW_CUR、MID_CUR、HIGH_CUR
 */
uint8_t Gate_get_status(void);

/*
 * @brief  设置门的范围模式
 * @param mode: 选择的范围模式，GATE_MODE_AUTO、GATE_MODE_LOW、GATE_MODE_MID、GATE_MODE_HIGH
 * @retval None
 */
void Gate_Set_Mode(uint8_t mode);

/*
 * @brief  获取门的当前范围模式
 * @param None
 * @retval 门的当前范围模式，GATE_MODE_AUTO、GATE_MODE_LOW、GATE_MODE_MID、GATE_MODE_HIGH
 */
uint8_t Gate_Get_Mode(void);

#ifdef __cplusplus
}
#endif
#endif

