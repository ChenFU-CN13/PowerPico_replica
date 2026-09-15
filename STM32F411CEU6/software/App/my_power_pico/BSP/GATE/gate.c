#include "gate.h"  // 引入量程定义、GPIO 引脚定义和 HAL 接口。
#include "math.h"  // 当前文件未使用数学函数，保留原有头文件引用。

static uint8_t gate_status = HIGH_CUR;      // 记录已经实际写入 GPIO 的硬件量程；上电默认按高电流档记录。
static uint8_t gate_mode = GATE_MODE_AUTO;  // 记录用户选择的模式；上电默认允许自动量程策略切换。

/*
 * @brief  初始化门的GPIO引脚
 * @param None
 * @retval None
 */
void Gate_Port_Init(void)  // 初始化 EN1、EN2 两个量程控制 GPIO。
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // 创建 HAL GPIO 配置结构体，并清零未使用字段。

  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();  // 打开 EN2 所在 GPIOA 外设时钟。
	__HAL_RCC_GPIOB_CLK_ENABLE();  // 打开 EN1 所在 GPIOB 外设时钟。

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, GPIO_PIN_SET);    // 先给 EN1 写高，和默认 HIGH_CUR 的组合保持一致。
	HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, GPIO_PIN_RESET);  // 先给 EN2 写低，最终组合为 EN1=1、EN2=0。

  /*Configure GPIO pin : PB3 */
	GPIO_InitStruct.Pin = EN1_PIN;                    // 选择 PB3，即 EN1 引脚。
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // 配置为推挽输出，MCU 主动输出高或低电平。
	GPIO_InitStruct.Pull = GPIO_NOPULL;               // 不启用内部上拉或下拉。
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;      // 量程控制不需要高速翻转，使用低速可降低边沿干扰。
	HAL_GPIO_Init(EN1_PORT, &GPIO_InitStruct);        // 将上述配置真正写入 GPIOB 的 EN1 引脚。

	/*Configure GPIO pin : Pa15 */
	GPIO_InitStruct.Pin = EN2_PIN;                    // 改为选择 PA15，即 EN2 引脚。
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // EN2 同样配置为推挽输出。
	GPIO_InitStruct.Pull = GPIO_NOPULL;               // EN2 同样不启用内部上下拉。
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;      // EN2 同样使用低速 GPIO 输出。
	HAL_GPIO_Init(EN2_PORT, &GPIO_InitStruct);        // 将上述配置真正写入 GPIOA 的 EN2 引脚。
}

// 根据选择设置流路
/*
 * @brief  根据选择设置流路
 * @param selection: 选择的流路，LOW_CUR、MID_CUR、HIGH_CUR
 * @retval None
 */
void flow_route_selection(uint8_t selection)  // 根据目标量程组合 EN1、EN2，完成真实硬件流路切换。
{
	/* 第 1 步：把非法输入夹到有效范围 LOW_CUR~HIGH_CUR 内。 */
	if (selection > HIGH_CUR)  // 大于 3 的值不是有效量程。
		selection = HIGH_CUR;    // 非法偏大时，安全地限制为高电流档。
	if (selection < LOW_CUR)   // 小于 1 的值不是有效量程。
		selection = LOW_CUR;     // 非法偏小时，限制为低电流档。

	/* 第 2 步：按量程输出对应的 EN1、EN2 电平组合。 */
	if (selection == HIGH_CUR)  // 高电流档：使用 5 mΩ 采样路径。
	{
		HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, GPIO_PIN_SET);    // HIGH_CUR 的 EN1=1。
		HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, GPIO_PIN_RESET);  // HIGH_CUR 的 EN2=0。
	}
	else if (selection == MID_CUR)  // 中电流档：使用 0.5 Ω 采样路径。
	{
		HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, GPIO_PIN_RESET);  // MID_CUR 的 EN1=0。
		HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, GPIO_PIN_SET);    // MID_CUR 的 EN2=1。
	}
	else  // 前面已做范围夹紧，剩下的唯一合法情况就是 LOW_CUR。
	{
		HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, GPIO_PIN_RESET);  // LOW_CUR 的 EN1=0。
		HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, GPIO_PIN_RESET);  // LOW_CUR 的 EN2=0。
	}
	gate_status = selection;  // GPIO 切换完成后，同步记录当前实际硬件量程。
}

// 获取当前档位状态
/*
 * @brief  获取门的当前状态
 * @param None
 * @retval 门的当前状态，LOW_CUR、MID_CUR、HIGH_CUR
 */
uint8_t Gate_get_status(void)  // 返回当前已经切到硬件上的量程。
{
	return gate_status;  // 返回 LOW_CUR、MID_CUR 或 HIGH_CUR。
}

// 这个函数允许用户直接设置档位模式，自动模式会根据当前电流情况自动切换档位
void Gate_Set_Mode(uint8_t mode)  // 设置用户希望的量程模式；手动模式会立即切 GPIO。
{
	if (mode == GATE_MODE_AUTO)  // 用户选择自动量程。
	{
		gate_mode = GATE_MODE_AUTO;  // 只改变工作模式，不强制改变当前硬件量程。
		return;                       // 后续由 Process_ADC_Chunk() 的自动策略决定何时切档。
	}
	if (mode == GATE_MODE_LOW || mode == GATE_MODE_MID || mode == GATE_MODE_HIGH)  // 仅接受三种手动档。
	{
		gate_mode = mode;              // 记录用户锁定的手动模式。
		flow_route_selection(mode);     // 手动模式立即落到统一的 GPIO 切档函数。
	}
}

// 获取当前档位模式
/*
 * @brief  获取门的当前范围模式
 * @param None
 * @retval 门的当前范围模式，GATE_MODE_AUTO、GATE_MODE_LOW、GATE_MODE_MID、GATE_MODE_HIGH
 */
uint8_t Gate_Get_Mode(void)  // 返回用户当前选择的量程工作模式。
{
	return gate_mode;  // 返回 GATE_MODE_AUTO 或某一个手动档位。
}
