/**
 * @file test.c
 * @brief 用于复刻过程中，存放测试代码，供外部调用。
 *
 */
#include "test.h"

#include "BL24C02.h"
#include "user_AdcDataStrategy.h"
#include "user_CmdStrategy.h"
#include "user_TasksInit.h"
#include "user_MessageTask.h"
#include "cmsis_os.h"
#include "lcd.h"
#include "lcd_init.h"
#include "tim.h"
#include "gate.h"  
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>



/* 设置为1时执行EEPROM写入和读回测试，设置为0时只读回掉电保存的数据。 */
#define STEP2_EEPROM_WRITE_TEST 1U
#define STEP2_SETTINGS_ADDRESS  0x40U
#define STEP2_SETTINGS_MAGIC    0x5049434FU

/**
 * @brief Step 2阶段使用的EEPROM测试数据
 */
typedef struct
{
    uint32_t magic;
    uint32_t save_count;
    uint16_t rotation;
    uint8_t backlight;
    uint8_t key_sound_enable;
    uint8_t reserve[12];
} Step2Settings_T;

/* 测试结果保留为静态变量，方便在Keil调试窗口中观察。 */
static volatile bool s_eeprom_write_ok;
static volatile bool s_eeprom_read_ok;
static volatile bool s_eeprom_data_ok;
static volatile bool s_tim5_wrap_test_ok;
static volatile uint64_t s_tim5_begin_us;
static volatile uint64_t s_tim5_end_us;
static volatile uint64_t s_tim5_elapsed_us;

/**
 * @brief 测试EEPROM跨页写入、读回和掉电保存
 * @return true 数据读取成功且内容一致
 * @return false 写入、读取或数据比较失败
 */
static bool Step2_EEPROM_Test(void)
{
    static const Step2Settings_T expected_settings =
    {
        .magic = STEP2_SETTINGS_MAGIC,
        .save_count = 1234U,
        .rotation = 0U,
        .backlight = 50U,
        .key_sound_enable = 1U,
        .reserve = {
            0x10, 0x11, 0x12, 0x13,
            0x14, 0x15, 0x16, 0x17,
            0x18, 0x19, 0x1A, 0x1B
        }
    };
    Step2Settings_T readback_settings;

#if STEP2_EEPROM_WRITE_TEST
    /* 第一轮测试执行写入，用于验证跨页写功能。 */
    s_eeprom_write_ok = EEPROM_WriteBytes(
        STEP2_SETTINGS_ADDRESS,
        (const uint8_t *)&expected_settings,
        sizeof(expected_settings));
#endif

    memset(&readback_settings, 0, sizeof(readback_settings));
    s_eeprom_read_ok = EEPROM_ReadBytes(
        STEP2_SETTINGS_ADDRESS,
        (uint8_t *)&readback_settings,
        sizeof(readback_settings));

#if STEP2_EEPROM_WRITE_TEST
    s_eeprom_data_ok = s_eeprom_write_ok
        && s_eeprom_read_ok
        && memcmp(&expected_settings,
                  &readback_settings,
                  sizeof(expected_settings)) == 0;
#else
    /* 掉电保存测试只读数据，不重新写入EEPROM。 */
    s_eeprom_data_ok = s_eeprom_read_ok
        && memcmp(&expected_settings,
                  &readback_settings,
                  sizeof(expected_settings)) == 0;
#endif

    return s_eeprom_data_ok;
}

/**
 * @brief 测试TIM5在32位计数器回绕后是否继续累计微秒数
 * @return true 回绕后时间继续递增，且2ms延时结果在允许范围内
 * @return false 微秒计数结果异常
 */
static bool Step2_TIM5_WrapTest(void)
{
    ClearMicrosecondCounter();

    /* 距离0xFFFFFFFF只剩1024个计数，延时2ms会经过一次32位回绕。 */
    __HAL_TIM_SET_COUNTER(&htim5, 0xFFFFFC00U);
    s_tim5_begin_us = GetMicrosecondCounter();

    osDelay(2U);

    s_tim5_end_us = GetMicrosecondCounter();
    s_tim5_elapsed_us = s_tim5_end_us - s_tim5_begin_us;

    s_tim5_wrap_test_ok = s_tim5_end_us > s_tim5_begin_us
        && s_tim5_elapsed_us >= 1500U
        && s_tim5_elapsed_us <= 10000U;

    return s_tim5_wrap_test_ok;
}

/**
 * @brief 在LCD上显示Step 2测试结果
 * @param eeprom_ok EEPROM测试结果
 * @param tim5_ok TIM5回绕测试结果
 */
static void Step2_ShowResult(bool eeprom_ok, bool tim5_ok)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    LCD_Set_Light(50);
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLUE);
    LCD_ShowString(20, 20, (const u8 *)"PowerPico", WHITE, BLUE, 24, 0);

    LCD_ShowString(20, 60,
                   (const u8 *)(eeprom_ok ? "EEPROM_PASS" : "EEPROM_FAIL"),
                   eeprom_ok ? GREEN : RED,
                   BLUE, 16, 0);

    LCD_ShowString(20, 120,
                   (const u8 *)(tim5_ok ? "TIM5_WRAP_PASS" : "TIM5_WRAP_FAIL"),
                   tim5_ok ? GREEN : RED,
                   BLUE, 16, 0);
}

/**
 * @brief 执行Step 2阶段的EEPROM、LCD和TIM5硬件测试
 * @note  本函数内部使用osDelay，只能在FreeRTOS任务上下文中调用。
 */
void Step2_Test_Run(void)
{
    bool eeprom_ok;
    bool tim5_ok;

    eeprom_ok = Step2_EEPROM_Test();
    tim5_ok = Step2_TIM5_WrapTest();
    Step2_ShowResult(eeprom_ok, tim5_ok);
}


/***************************************STEP3阶段测试代码*************************************************/


static volatile HAL_StatusTypeDef s_step3_start_status = HAL_ERROR;
/**
 * @brief 计算某一路ADC的20次采样平均值(即2轮)
 * @param channel 通道在DMA每一行中的位置
 * @return ADC平均原始值，范围0～4095
 * @note 仅用于STEP3观察，不作为正式数据处理代码
 */
static uint16_t Step3_ADC_GetAverage(uint16_t channel)
{
    uint32_t sum=0;
    uint32_t i;

    if (channel >= ADC_CHANNELS)
    {
        return 0U;
    }

    /*这里把每一例的：VOL LOW MID HIGH REF的ADC值累加起来，一共20行，一行有5列*/
    for (i = 0U; i < ADC_DMA_ROWS; i++)
    {
        sum += adc_raw_buffer[i][channel];
    }

    return (uint16_t)(sum / ADC_DMA_ROWS);  //最后求每个通道ADC值的平均值
}



/**
 * @brief 执行Step 3阶段ADC1+dma+tim2 10khz采样一次 硬件测试
 * @note  本函数内部使用osDelay，只能在FreeRTOS任务上下文中调用。
 */
void Step3_Test_Run(void)
{


    Gate_Port_Init();

    flow_route_selection(HIGH_CUR);


    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    LCD_Set_Light(100);
    LCD_Init();
    LCD_Fill(0,0,LCD_W,LCD_H,BLUE);

    LCD_ShowString(10, 5,
                   (const u8 *)"STEP3 ADC DMA",
                   WHITE, BLACK, 16, 0);
    s_step3_start_status  = ADC_Sampling_Start();
    if (s_step3_start_status != HAL_OK)
    {
        LCD_ShowString(10, 30,
                       (const u8 *)"START FAIL",
                       RED, BLACK, 16, 0);
        return;
    }

    LCD_ShowString(10, 30,  (const u8 *)"VOL :",  BLUE, WHITE, 16, 0);
    LCD_ShowString(10, 54,  (const u8 *)"LOW :",  BLUE, WHITE, 16, 0);
    LCD_ShowString(10, 78,  (const u8 *)"MID :",  BLUE, WHITE, 16, 0);
    LCD_ShowString(10, 102, (const u8 *)"HIGH:",  BLUE, WHITE, 16, 0);
    LCD_ShowString(10, 126, (const u8 *)"REF :",  BLUE, WHITE, 16, 0);
    LCD_ShowString(10, 150, (const u8 *)"HALF:",  BLUE, WHITE, 16, 0);
    LCD_ShowString(10, 174, (const u8 *)"FULL:",  BLUE, WHITE, 16, 0);
    LCD_ShowString(10, 198, (const u8 *)"ERR :",  BLUE, WHITE, 16, 0);
}




/**
 * @brief 刷新LCD上的ADC测试数据
 * @note 在任务中调用，禁止在ADC中断回调里调用
 */
void Step3_Test_Update(void)
{
    uint16_t value;
    if (s_step3_start_status != HAL_OK)
    {
        return;
    }
    
    value = Step3_ADC_GetAverage(IDX_VOL);
    LCD_ShowIntNum(80,30,value,4,BLUE,WHITE,16);

    value = Step3_ADC_GetAverage(IDX_LOW);
    LCD_ShowIntNum(80,54,value,4,BLUE,WHITE,16);

    value = Step3_ADC_GetAverage(IDX_MID);
    LCD_ShowIntNum(80,78,value,4,BLUE,WHITE,16);

    value = Step3_ADC_GetAverage(IDX_HIGH);
    LCD_ShowIntNum(80,102,value,4,BLUE,WHITE,16);

    value = Step3_ADC_GetAverage(IDX_REF);
    LCD_ShowIntNum(80,126,value,4,BLUE,WHITE,16);
    



    LCD_ShowIntNum(80,150,(uint16_t)(adc_half_count%10000),4,BLUE,WHITE,16);

    LCD_ShowIntNum(80,174,(uint16_t)(adc_full_count%10000),4,BLUE,WHITE,16);


    value = (adc_error_count > 9999U) ?
            9999U : (uint16_t)adc_error_count;

    LCD_ShowIntNum(80, 198, value, 4,
                   adc_error_count == 0U ? GREEN : RED,
                   BLACK, 16);
    


    


    
}


/******************************STEP4测试代码*******************************************************/
/******************************** STEP4手动档换算测试 ******************************/

/*
 * 未知负载先使用HIGH档。
 *
 * 测试另外两档时分别改成：
 * GATE_MODE_MID
 * GATE_MODE_LOW
 */
#define STEP4_MANUAL_MODE    GATE_MODE_MID

static volatile HAL_StatusTypeDef s_step4_start_status = HAL_ERROR;

/**
 * @brief 把浮点数四舍五入成有符号整数
 */
static int32_t Step4_RoundFloat(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

/**
 * @brief 取得有符号整数的绝对值
 */
static uint32_t Step4_AbsI32(int32_t value)
{
    if (value < 0)
    {
        return (uint32_t)(-value);
    }

    return (uint32_t)value;
}

/**
 * @brief 根据档位选择对应的电流ADC通道
 */
static uint16_t Step4_SelectCurrentRaw(const ADC_RawAverage_T *average,
                                      uint8_t range)
{
    switch (range)
    {
        case LOW_CUR:
            return average->low_raw;

        case MID_CUR:
            return average->mid_raw;

        case HIGH_CUR:
            return average->high_raw;

        default:
            return average->high_raw;
    }
}

/**
 * @brief 获取档位名称
 */
static const char *Step4_GetRangeName(uint8_t range)
{
    switch (range)
    {
        case LOW_CUR:
            return "LOW ";

        case MID_CUR:
            return "MID ";

        case HIGH_CUR:
            return "HIGH";

        default:
            return "ERR ";
    }
}

/**
 * @brief 清除并显示一行测量值
 */
static void Step4_ShowValue(uint16_t y,
                            const char *text,
                            uint16_t color)
{
    /*
     * 值从x=88开始显示。
     * 每次先清除旧内容，防止新字符串比旧字符串短时留下残影。
     */
    LCD_Fill(88, y, 240, y + 16U, BLACK);

    LCD_ShowString(88,
                   y,
                   (const u8 *)text,
                   color,
                   BLACK,
                   16,
                   0);
}

/**
 * @brief 绘制Step 4固定UI
 */
static void Step4_DrawUI(void)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

    /* 顶部标题栏 */
    LCD_Fill(0, 0, LCD_W, 27, DARKBLUE);

    LCD_ShowString(72, 6,
                   (const u8 *)"STEP4 MANUAL",
                   WHITE, DARKBLUE, 16, 0);

    LCD_DrawLine(0, 28, 239, 28, GBLUE);

    /* 左侧固定标签 */
    LCD_ShowString(10, 36,  (const u8 *)"RANGE :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 62,  (const u8 *)"VRAW  :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 88,  (const u8 *)"IRAW  :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 114, (const u8 *)"REF   :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 140, (const u8 *)"DELTA :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 168, (const u8 *)"VBUS  :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 198, (const u8 *)"CURR  :", GRAY, BLACK, 16, 0);
}

/**
 * @brief 启动Step 4手动档测试
 */
void Step4_Test_Run(void)
{
    Gate_Port_Init();

    /*
     * 使用Gate_Set_Mode而不是直接调用flow_route_selection。
     * 这样软件记录的手动模式和真实GPIO档位保持一致。
     */
    Gate_Set_Mode(STEP4_MANUAL_MODE);

    /* 给MOS流路和INA190输出一点稳定时间。 */
    osDelay(10U);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    LCD_Set_Light(70U);
    LCD_Init();

    Step4_DrawUI();

    Step4_ShowValue(36,
                    Step4_GetRangeName(Gate_get_status()),
                    YELLOW);

    s_step4_start_status = ADC_Sampling_Start();

    if (s_step4_start_status != HAL_OK)
    {
        Step4_ShowValue(198, "ADC START FAIL", RED);
    }
}

/**
 * @brief 刷新Step 4测量页面
 */
void Step4_Test_Update(void)
{
    ADC_RawAverage_T average;
    uint8_t range;              // 当前真实档位
    uint16_t current_adc_raw;   // 当前档位对应的电流ADC值
    int32_t delta_adc_raw;      // 电流ADC值减去REF值

    float voltage_v;            // VBUS电压，单位V
    float current_uA;           // 电流，单位µA
    
    /*这三个LCD格式化的变量*/
    uint32_t voltage_mV;
    int32_t  current_display;
    uint32_t magnitude;
    char text[24];

    //1.先检查ADC采集是否成功启动
    if (s_step4_start_status != HAL_OK)
    {
        return;
    }
    
    /*获取ADC平均值*/
    if (!ADC_GetRawAverage(&average))
    {
        Step4_ShowValue(198,"WAIT_DMA",YELLOW);
         return;
    }

    /*2.获取当前电流档位*/
    range = Gate_get_status();
    /*获取当前档位对应的ADC通道平均值*/
    current_adc_raw = Step4_SelectCurrentRaw(&average,range);
    /*计算电流ADC差值*/
    delta_adc_raw =
        (int32_t)current_adc_raw - (int32_t)average.ref_raw;
    /*3.将ADC值转换为电压*/
    voltage_v = ADC_ConvertVoltage_V(average.vol_raw);
    /*4.将ADC值转换为电压，配合公式计算*/
    current_uA = ADC_ConvertCurrent_uA(current_adc_raw,
                                       average.ref_raw,
                                       range);
    
    /*当前手动档位*/
    Step4_ShowValue(36,Step4_GetRangeName(range),YELLOW);

    /* 电压原始值 */
    snprintf(text, sizeof(text),
             "%4u",
             (unsigned int)average.vol_raw);
    Step4_ShowValue(62, text, WHITE);

    /* 当前档电流原始值 */
    snprintf(text, sizeof(text),
             "%4u",
             (unsigned int)current_adc_raw);

    Step4_ShowValue(88, text, WHITE);


    /* 参考通道原始值 */
    snprintf(text, sizeof(text),
             "%4u",
             (unsigned int)average.ref_raw);

    Step4_ShowValue(114, text, WHITE);



    /* 有符号ADC差值 */
    snprintf(text, sizeof(text),
             "%c%04lu",
             delta_adc_raw >= 0 ? '+' : '-',
             (unsigned long)Step4_AbsI32(delta_adc_raw));

    Step4_ShowValue(140,
                    text,
                    delta_adc_raw >= 0 ? GREEN : RED);

    /* VBUS显示为x.xxx V */
    voltage_mV = (uint32_t)(voltage_v * 1000.0f + 0.5f);

    snprintf(text, sizeof(text),
             "%lu.%03lu V",
             (unsigned long)(voltage_mV / 1000U),
             (unsigned long)(voltage_mV % 1000U));

    Step4_ShowValue(168, text, CYAN);


    
    if (range == LOW_CUR)
    {
        /*
         * LOW档显示uA，保留两位小数。
         * current_display单位为0.01uA。
         */
        current_display = Step4_RoundFloat(current_uA * 100.0f);
        magnitude = Step4_AbsI32(current_display);

        snprintf(text, sizeof(text),
                 "%c%lu.%02lu uA",
                 current_display >= 0 ? '+' : '-',
                 (unsigned long)(magnitude / 100U),
                 (unsigned long)(magnitude % 100U));
    }
    else
    {
        /*
         * MID和HIGH档显示mA，保留三位小数。
         * 1uA正好等于0.001mA，因此这里直接把uA四舍五入。
         */
        current_display = Step4_RoundFloat(current_uA);

        magnitude = Step4_AbsI32(current_display);

        snprintf(text, sizeof(text),
                 "%c%lu.%03lu mA",
                 current_display >= 0 ? '+' : '-',
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U));
    }

    Step4_ShowValue(198,
                    text,
                    current_display >= 0 ? GREEN : RED);



}



/***************************************STEP4阶段测试代码*************************************************/






/************************ STEP5自动换挡测试 ************************/
        /*
        程序设计思路：
        ADC + DMA循环采样
                ↓
        半满/全满中断
                ↓
        发布完整ADC数据块
                ↓
        唤醒ADC处理任务
                ↓
        Process_ADC_Chunk()
                ↓
        自动换挡判断
                ↓
        更新调试快照
                ↓
        LCD任务读取并显示*/
static volatile HAL_StatusTypeDef s_step5_start_status = HAL_ERROR;

/*
 * STEP6A校准实验模式：
 *
 * LOAD：只读取上一次掉电保存的系数，不擦写EEPROM。
 * WRITE：写入测试系数，再从EEPROM读回核对。
 * INVALID：提交非法倍率，检查RAM中的有效参数是否保持不变。
 *
 * 当前执行LOAD，用来验证断电后能否恢复上一轮保存的校准系数。
 */
#define STEP6A_CAL_MODE_LOAD       0U
#define STEP6A_CAL_MODE_WRITE      1U
#define STEP6A_CAL_MODE_INVALID    2U
#define STEP6A_CAL_TEST_MODE       STEP6A_CAL_MODE_LOAD

/**
 * @brief STEP6A校准实验结果
 * @note  该状态直接供LCD页面使用，不是Keil Watch调试变量。
 */
typedef enum
{
    STEP6A_CAL_DEFAULT = 0,
    STEP6A_CAL_EEPROM_OK,
    STEP6A_CAL_WRITE_OK,
    STEP6A_CAL_WRITE_FAIL,
    STEP6A_CAL_REJECT_OK,
    STEP6A_CAL_REJECT_FAIL
} Step6A_CalStatus_T;

static Step6A_CalStatus_T s_step6a_cal_status = STEP6A_CAL_DEFAULT;

/**
 * @brief 获取自动量程状态名称
 */
static const char *Step5_GetStateName(ADC_AutoRangeState_T state)
{
    switch (state)
    {
        case ADC_RANGE_STATE_HOLD:
            return "HOLD";
            
        case ADC_RANGE_STATE_UP_WAIT:
            return "UP_WAIT";

        case ADC_RANGE_STATE_DOWN_WAIT:
            return "DOWN_WAIT";

        case ADC_RANGE_STATE_SWITCH:
            return "SWITCH";

        case ADC_RANGE_STATE_SKIP:
            return "SKIP";

        case ADC_RANGE_STATE_COOL:
            return "COOL";

        default:
            return "ERROR";
    }
}

/**
 * @brief 获取单字符档位名称
 */
static char Step5_GetRangeChar(uint8_t range)
{
    switch (range)
    {
        case LOW_CUR:
            return 'L';

        case MID_CUR:
            return 'M';

        case HIGH_CUR:
            return 'H';

        default:
            return '?';
    }
}


/**
 * @brief 获取最近一次换挡原因名称
 */
static const char *Step5_GetReasonName(ADC_SwitchReason_T reason)
{
    switch (reason)
    {
        case ADC_SWITCH_REASON_OVERLOAD:
            return "UP";

        case ADC_SWITCH_REASON_UNDERLOAD:
            return "DN";

        case ADC_SWITCH_REASON_NEAR_RAIL:
            return "RAIL";

        case ADC_SWITCH_REASON_RANGE_ERROR:
            return "ERR";

        default:
            return "--";
    }
}

/**
 * @brief 获取STEP6A校准实验结果文字
 * @param status 校准实验结果
 * @return LCD显示用的短字符串
 */
static const char *Step6A_GetCalStatusName(Step6A_CalStatus_T status)
{
    switch (status)
    {
        case STEP6A_CAL_EEPROM_OK:
            return "EEPROM OK";

        case STEP6A_CAL_WRITE_OK:
            return "WRITE OK";

        case STEP6A_CAL_WRITE_FAIL:
            return "WRITE FAIL";

        case STEP6A_CAL_REJECT_OK:
            return "REJECT OK";

        case STEP6A_CAL_REJECT_FAIL:
            return "REJECT FAIL";

        default:
            return "DEFAULT";
    }
}

/**
 * @brief 获取STEP6A校准实验结果颜色
 * @param status 校准实验结果
 * @return LCD颜色值
 */
static uint16_t Step6A_GetCalStatusColor(Step6A_CalStatus_T status)
{
    if (status == STEP6A_CAL_EEPROM_OK
        || status == STEP6A_CAL_WRITE_OK
        || status == STEP6A_CAL_REJECT_OK)
    {
        return GREEN;
    }

    if (status == STEP6A_CAL_WRITE_FAIL
        || status == STEP6A_CAL_REJECT_FAIL)
    {
        return RED;
    }

    return YELLOW;
}

/**
 * @brief 在LCD上长期显示STEP6A校准实验结果
 */
/**
 * @brief 显示校准加载结果和STEP6B任务框架状态
 */
static void Step6A_ShowCalStatus(void)
{
    UserTasksStatus_T task_status;
    char text[24];
    uint16_t color;

    task_status = User_Tasks_GetStatus();

    if (task_status == USER_TASKS_STATUS_CREATE_FAIL)
    {
        /*
         * BERR后面的十六进制数是创建失败位图。
         * 可以对照user_TasksInit.h中的错误位定义定位。
         */
        snprintf(text,
                 sizeof(text),
                 "BERR %04lX",
                 (unsigned long)User_Tasks_GetErrorMask());

        color = RED;
    }
    else
    {
        snprintf(text,
                 sizeof(text),
                 "%s %s",
                 Step6A_GetCalStatusName(
                     s_step6a_cal_status),
                 task_status == USER_TASKS_STATUS_READY
                     ? "BOK"
                     : "BWAIT");

        color =
            task_status == USER_TASKS_STATUS_READY
                ? Step6A_GetCalStatusColor(
                      s_step6a_cal_status)
                : YELLOW;
    }

    Step4_ShowValue(180, text, color);
}

/**
 * @brief 执行STEP6A校准系数实验
 *
 * WRITE模式的测试参数故意与默认值拉开差距，方便确认读回内容确实改变。
 * 三档偏移量仍在当前“正负32个ADC码”的软件保护范围内。
 *
 * INVALID模式提交1.50倍的非法倍率。当前合法上限是1.25倍，
 * 因此正确结果应是拒绝更新，并保留原来有效的RAM参数。
 */
static void Step6A_Calibration_Test(void)
{
#if STEP6A_CAL_TEST_MODE == STEP6A_CAL_MODE_LOAD

    bool load_ok;

    /*
     * LOAD模式不写EEPROM，只检查之前保存的整份设置能否正常恢复。
     * EEPROM为空或数据非法时，加载函数会自动恢复RAM默认值。
     */
    load_ok = EEPROM_SysSetting_Load();

    s_step6a_cal_status = load_ok
                            ? STEP6A_CAL_EEPROM_OK
                            : STEP6A_CAL_DEFAULT;

#elif STEP6A_CAL_TEST_MODE == STEP6A_CAL_MODE_WRITE

    const ADC_Calibration_T test_calibration =
    {
        .low_scale_multiplier = 1.10f,
        .mid_scale_multiplier = 1.10f,
        .high_scale_multiplier = 1.10f,
        .low_offset_ua = 5.0f,
        .mid_offset_ua = 500.0f,
        .high_offset_ua = 50000.0f
    };

    ADC_Calibration_T calibration_after;
    bool set_ok;
    bool save_ok;

    /*
     * 先加载旧设置；读取失败时，加载函数会先恢复安全默认值。
     * 再更新RAM、保存整份设置，最后重新加载并逐字节核对。
     */
    (void)EEPROM_SysSetting_Load();
    set_ok = Sys_Set_AdcCalibration(&test_calibration);
    save_ok = set_ok && EEPROM_SysSetting_Save();

    if (save_ok && EEPROM_SysSetting_Load())
    {
        Sys_Get_AdcCalibration(&calibration_after);

        s_step6a_cal_status =
            (memcmp(&calibration_after,
                    &test_calibration,
                    sizeof(test_calibration)) == 0)
                ? STEP6A_CAL_WRITE_OK
                : STEP6A_CAL_WRITE_FAIL;
    }
    else
    {
        s_step6a_cal_status = STEP6A_CAL_WRITE_FAIL;
    }

#elif STEP6A_CAL_TEST_MODE == STEP6A_CAL_MODE_INVALID

    ADC_Calibration_T calibration_before;
    ADC_Calibration_T calibration_after;
    bool set_ok;

    /* 先加载EEPROM；数据无效时会恢复默认参数。 */
    (void)EEPROM_SysSetting_Load();

    /* 保存当前有效参数，后面确认非法写入没有污染RAM。 */
    Sys_Get_AdcCalibration(&calibration_before);
    calibration_after = calibration_before;
    calibration_after.low_scale_multiplier = 1.50f;

    set_ok = Sys_Set_AdcCalibration(&calibration_after);
    Sys_Get_AdcCalibration(&calibration_after);

    s_step6a_cal_status =
        (!set_ok
         && memcmp(&calibration_before,
                   &calibration_after,
                   sizeof(calibration_before)) == 0)
            ? STEP6A_CAL_REJECT_OK
            : STEP6A_CAL_REJECT_FAIL;

#else
#error "STEP6A_CAL_TEST_MODE配置错误"
#endif
}

/**
 * @brief 执行STEP6C的一次性硬件启动流程
 *
 * 该函数由HardwareInitTask调用。
 * 必须先准备校准参数和量程状态，最后才能启动ADC采样。
 */
void Step6C_HardwareInit_Run(void)
{
    /*
     * 当前测试模式必须保持STEP6A_CAL_MODE_LOAD。
     * 启动时只读取EEPROM，不再重复执行写入实验。
     */
    Step6A_Calibration_Test();

    /*
     * main()中已经提前把真实硬件切到HIGH安全档。
     * 这里切换的只是软件控制模式，不会立即改变当前硬件档位。
     */
    Gate_Set_Mode(Sys_Get_CurrentRangeMode());

    /*
     * 根据当前真实档位重新初始化自动量程状态、
     * 连续确认计数和调试信息。
     */
    ADC_AutoRange_Reset();

    /*
     * 先启动ADC DMA，再启动TIM2触发。
     * 返回值保存下来，供LCD显示ADC启动是否成功。
     */
    s_step5_start_status = ADC_Sampling_Start();
}


/**
 * @brief 绘制STEP7 USB波形与命令通信观察页面
 */
static void Step6E_DrawUI(void)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

    LCD_Fill(0, 0, LCD_W, 27, DARKBLUE);

    LCD_ShowString(56, 6,
                (const u8 *)"STEP7 USB STREAM",
                WHITE, DARKBLUE, 16, 0);

    LCD_DrawLine(0, 28, 239, 28, GBLUE);

    LCD_ShowString(10, 36,  (const u8 *)"RANGE :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 60,  (const u8 *)"RAW I/R:", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 84,  (const u8 *)"DELTA :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 108, (const u8 *)"CURR  :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 132, (const u8 *)"STATE :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 156, (const u8 *)"COUNT :", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 180, (const u8 *)"CAL/USB:", GRAY, BLACK, 16, 0);
    LCD_ShowString(10, 204, (const u8 *)"LAST  :", GRAY, BLACK, 16, 0);
}


/**
 * @brief 初始化STEP6E的LCD观察页面
 *
 * 校准加载、量程初始化和ADC启动
 * 已经交给HardwareInitTask完成。
 *
 * 本函数只由LvHandlerTask调用，不再启动ADC。
 */
void Step6E_Test_Run(void)
{
    /* 启动LCD背光PWM。 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 设置背光并初始化LCD。 */
    LCD_Set_Light(70U);
    LCD_Init();

    /* 绘制固定界面并显示校准、任务状态。 */
    Step6E_DrawUI();
    /*
    * 收到第一条PowerDataQueue数据前保持该提示。
    * 如果一直不消失，说明生产者或队列链路没有工作。
    */
    Step4_ShowValue(108, "WAIT QUEUE", YELLOW);
    Step6E_Test_UpdateUsb();

    /*
     * ADC启动失败时立即显示。
     * 成功时由Step6E_Test_Update()持续刷新测量数据。
     */
    if (s_step5_start_status != HAL_OK)
    {
        Step4_ShowValue(132, "ADC FAIL", RED);
    }
}





/**
 * @brief 使用PowerDataQueue中的数据刷新STEP6E观察页面
 * @param current_uA MessageSendTask换算并通过队列发送的电流，单位uA
 * @param sample_range 产生本次电流数据时的采样量程
 */
void Step6E_Test_Update(float current_uA, uint8_t sample_range)
{
    ADC_AutoRangeDebug_T debug;

    int32_t current_display;
    uint32_t magnitude;

    char text[32];

    if (s_step5_start_status != HAL_OK)
    {
        return;
    }

    if (!ADC_AutoRange_GetDebug(&debug))
    {
        Step4_ShowValue(132, "WAIT_DMA", YELLOW);
        return;
    }

    /* 显示与队列电流属于同一组数据的采样档位。 */
    Step4_ShowValue(36,
                    Step4_GetRangeName(sample_range),
                    YELLOW);

    /* 当前档电流ADC值和REF值。 */
    snprintf(text, sizeof(text),
             "%4u/%4u",
             (unsigned int)debug.current_adc_raw,
             (unsigned int)debug.ref_adc_raw);

    Step4_ShowValue(60, text, WHITE);

    /* 有符号差值和绝对差值。 */
    snprintf(text, sizeof(text),
             "%c%04lu A%04u",
             debug.delta_adc_raw >= 0 ? '+' : '-',
             (unsigned long)Step4_AbsI32(debug.delta_adc_raw),
             (unsigned int)debug.abs_delta_adc_raw);

    Step4_ShowValue(84,
                    text,
                    debug.delta_adc_raw >= 0 ? GREEN : RED);

    /*
     * LOW档显示uA。
     * MID和HIGH档显示mA。
     */
    if (sample_range == LOW_CUR)
    {
        current_display =
            Step4_RoundFloat(current_uA * 100.0f);

        magnitude = Step4_AbsI32(current_display);

        snprintf(text, sizeof(text),
                 "%c%lu.%02lu uA",
                 current_display >= 0 ? '+' : '-',
                 (unsigned long)(magnitude / 100U),
                 (unsigned long)(magnitude % 100U));
    }
    else
    {
        current_display = Step4_RoundFloat(current_uA);

        magnitude = Step4_AbsI32(current_display);

        snprintf(text, sizeof(text),
                 "%c%lu.%03lu mA",
                 current_display >= 0 ? '+' : '-',
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U));
    }

    Step4_ShowValue(108,
                    text,
                    current_display >= 0 ? GREEN : RED);

    /* 当前自动量程阶段、ADC错误和丢块累计次数。 */
    snprintf(text, sizeof(text),
             "%s E%lu Q%lu",
             Step5_GetStateName(debug.state),
             (unsigned long)adc_error_count,
             (unsigned long)debug.lost_block_count);

    Step4_ShowValue(132, text, CYAN);

    /*
     * U：当前升档连续确认次数
     * D：当前降档连续确认次数
     * S：累计丢弃的数据块数量
     * C：累计冷却的数据块数量
     */
    snprintf(text, sizeof(text),
             "U%u D%u S%lu C%lu",
             (unsigned int)debug.up_count,
             (unsigned int)debug.down_count,
             (unsigned long)debug.skipped_block_count,
             (unsigned long)debug.cooldown_block_count);

    Step4_ShowValue(156, text, WHITE);

    /* 最近一次换挡方向、原因和累计换挡次数。 */
    snprintf(text, sizeof(text),
             "%c>%c %s #%lu",
             Step5_GetRangeChar(debug.last_from),
             Step5_GetRangeChar(debug.last_to),
             Step5_GetReasonName(debug.last_reason),
             (unsigned long)debug.switch_count);

    Step4_ShowValue(204, text, YELLOW);
}

/**
 * @brief 显示STEP6E USB回调、队列和接收任务的运行计数
 *
 * R：USB回调成功放入CmdRxQueue的数据块数量；
 * P：MessageReceiveTask已经取出并解析的数据块数量；
 * D：队列未就绪、数据超长或队列满造成的丢包数量。
 */
void Step6E_Test_UpdateUsb(void)
{
    MessageRxDebug_T rx_debug;
    char text[24];
    uint16_t color;

    MessageReceive_GetDebug(&rx_debug);

    /*
     * 尚未收到USB数据时，继续显示已经验证过的校准和任务状态。
     * 串口助手发送任意非空文本后，本行自动切换为R/P/D计数。
     */
    if (rx_debug.received_chunks == 0U
        && rx_debug.processed_chunks == 0U
        && rx_debug.dropped_chunks == 0U)
    {
        Step6A_ShowCalStatus();
        return;
    }

    /* 三个计数只显示低三位，保证字符串不会超出LCD右侧。 */
    snprintf(text,
             sizeof(text),
             "R%03lu P%03lu D%03lu",
             (unsigned long)(rx_debug.received_chunks % 1000U),
             (unsigned long)(rx_debug.processed_chunks % 1000U),
             (unsigned long)(rx_debug.dropped_chunks % 1000U));

    if (rx_debug.dropped_chunks != 0U)
    {
        color = RED;
    }
    else if (rx_debug.received_chunks != rx_debug.processed_chunks)
    {
        color = YELLOW;
    }
    else
    {
        color = GREEN;
    }

    Step4_ShowValue(180, text, color);
}

/*************************************************************************************** */











































