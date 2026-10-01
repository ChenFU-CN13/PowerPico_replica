#include "gate.h"  // 引入量程定义、GPIO 引脚定义和 HAL 接口。
#include "math.h"  // 当前文件未使用数学函数，保留原有头文件引用。

static uint8_t gate_status = HIGH_CUR;      //记录已经实际写入 GPIO 的硬件量程，上电默认用高电流档
static uint8_t gate_mode = GATE_MODE_AUTO;        //记录用户选择的模式，上电默认允许自动量程切换


/*
 * @brief  初始化门的GPIO引脚
 * @param None
 * @retval None
 */
void Gate_Port_Init(void)
{
    /*1.先创建外设结构体，默认初始化*/
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*2.打开时钟*/
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(EN1_PORT,EN1_PIN,GPIO_PIN_SET);   // 先给 EN1 写高，和默认 HIGH_CUR 的组合保持一致。
    HAL_GPIO_WritePin(EN2_PORT,EN2_PIN,GPIO_PIN_RESET); // 先给 EN2 写低，最终组合为 EN1=1、EN2=0。

    /*3.配置GPIO结构体*/
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = EN1_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN1_PORT,&GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = EN2_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN2_PORT,&GPIO_InitStruct);
}




/*
 * @brief  选择流路
 * @param selection: 选择的流路，LOW_CUR、MID_CUR、HIGH_CUR
 * @retval None
 */
void flow_route_selection(uint8_t selection)
{
    /*1.判断非法输入值，并作处理*/
    if (selection>HIGH_CUR)
    {
        selection = HIGH_CUR;
    }
    if(selection<LOW_CUR)
    {
        selection = LOW_CUR;
    }


    /*2.按测量量程输出对应的EN1和EN2组合，控制档位切换*/
    if (selection == HIGH_CUR)   // 高电流档：使用 5 mΩ 采样路径。
    {
        HAL_GPIO_WritePin(EN1_PORT,EN1_PIN,GPIO_PIN_SET);
        HAL_GPIO_WritePin(EN2_PORT,EN2_PIN,GPIO_PIN_RESET);
    }
    else if (selection == MID_CUR)    // 中电流档：使用 0.5 Ω 采样路径。
    {
        HAL_GPIO_WritePin(EN1_PORT,EN1_PIN,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN2_PORT,EN2_PIN,GPIO_PIN_SET);
    }
    else                        // 小电流档：使用 3电阻均打通 采样路径。
    {
        HAL_GPIO_WritePin(EN1_PORT,EN1_PIN,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN2_PORT,EN2_PIN,GPIO_PIN_RESET);
    }
    

    gate_status = selection;    //同步记录当前实际的硬件量程



}



/*
 * @brief  获取门的当前实际的工作模式状态
 * @param None
 * @retval 门的当前实际的工作模式状态，LOW_CUR、MID_CUR、HIGH_CUR
 */
uint8_t Gate_get_status(void)
{
    return gate_status;         
}





/*
 * @brief  设置门的工作模式
 * @param mode: 用户选择的工作模式，GATE_MODE_AUTO、
 * GATE_MODE_LOW、GATE_MODE_MID、GATE_MODE_HIGH
 * @retval None
 */
void Gate_Set_Mode(uint8_t mode)
{
    if (mode == GATE_MODE_AUTO)
    {
        gate_mode = GATE_MODE_AUTO;     //只改变工作模式，不强制改变当前量程
        return;                         //后续由 Process_ADC_Chunk() 自动策略决定何时切换。
    }
    if (mode == GATE_MODE_LOW || mode == GATE_MODE_MID || mode == GATE_MODE_HIGH)   
    {
        gate_mode = mode;               //先更新门模式
        flow_route_selection(mode);     //再切换量程
    }

    
}






/*
 * @brief  获取门的当前范围模式
 * @param None
 * @retval 门的当前范围模式，GATE_MODE_AUTO、GATE_MODE_LOW、GATE_MODE_MID、GATE_MODE_HIGH
 */
uint8_t Gate_Get_Mode(void)  // 返回用户当前选择的量程工作模式。
{
	return gate_mode;  // 返回 GATE_MODE_AUTO 或某一个手动档位。
}










































