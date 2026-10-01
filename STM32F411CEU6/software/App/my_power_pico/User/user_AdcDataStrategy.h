#ifndef __USER_ADCDATASTRATEGY_H__
#define __USER_ADCDATASTRATEGY_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "adc.h"
#include "tim.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>


/* 每个DMA半区保存10轮五通道扫描结果。 */
#define ADC_TIMES           10      //每次采样10次（100us采样1次，则10次要1ms）
#define ADC_CHANNELS        5       //通道5采样电流


/* STEP7 USB波形包固定包头。 */
#define PACKET_HEADER_0          0xAAU
#define PACKET_HEADER_1          0x55U

/*
 * 包长度：
 * 2字节包头
 * + 8字节时间戳
 * + 1字节有效点数
 * + 10组 × 7字节采样数据
 * = 81字节
 */
#define USB_ADC_PACKET_SIZE      (11U + 7U * ADC_TIMES) //81字节

#pragma pack(push, 1)       // 1字节对齐

/**
 * @brief 一组USB波形采样点
 */
typedef struct
{
    uint8_t range;         // 本采样点对应的真实量程
    uint16_t vol_adc;      // VBUS电压通道原始码值
    uint16_t cur_adc;      // 当前量程对应的电流通道原始码值
    uint16_t ref_adc;      // REF参考通道原始码值
} USB_ADC_Sample_T;

/**
 * @brief PowerPico上位机使用的ADC波形包
 */
typedef struct
{
    uint8_t header[2];                         // 固定为AA 55
    uint64_t timestamp;                        // 微秒时间戳
    uint8_t data_count;                        // 本包有效点数，正常为10
    USB_ADC_Sample_T samples[ADC_TIMES];        // 10组采样点
} USB_ADC_Packet_T;

#pragma pack(pop)           // 恢复默认对齐方式

/* 编译期检查结构体必须正好为81字节。 */
typedef char USB_ADC_Packet_SizeCheck[
    (sizeof(USB_ADC_Packet_T) == USB_ADC_PACKET_SIZE) ? 1 : -1
];

// Rank1=PA5（channel5采样VBUS电压）, Rank2=PA6（channel6采样小电流档）, Rank3=PA7（channel7采样中电流档）
//, Rank4=PB0（channel8采样大电流档）, Rank5=PB1（channel9采样参考电压）

/*DMA缓冲区：分2个半区*/
#define ADC_DMA_ROWS        (ADC_TIMES*2)
#define ADC_DMA_LENGTH      (ADC_DMA_ROWS*ADC_CHANNELS)

/*5个ADC 通道在每一行中数据的位置*/
#define IDX_VOL  0
#define IDX_LOW  1
#define IDX_MID  2
#define IDX_HIGH 3
#define IDX_REF  4

/* ADC及模拟前端参数 */
#define ADC_VREF_V                  3.0f         //ADC满量程参考电压为3.0V
#define ADC_MAX_CODE                4095.0f      //12位ADC最大值=0-4095
#define ADC_CODE_COUNT              4096.0f     //这个是代表ADC从0-4095有4096个数，后面用来计算电流，表示每一个ADC LSB对应多少种输入电压变化
#define VBUS_DIVIDER_GAIN           11.0f       //这个代表原理图中vofrt区那里的1MΩ和100kΩ的分压比例
#define INA190_GAIN                 50.0f       //这个代表INA190A2的增益为50，用于放大采集到电阻的电压


/* 三档采样电阻，单位Ω */
#define HIGH_CUR_RES           0.005f
#define MID_CUR_RES            0.5f
#define LOW_CUR_RES            50.0f

/*
 * 三档理论换算比例，单位都是 uA/LSB。
 *
 * 含义：
 * ADC每变化1个码，理论上代表多少uA电流变化。
 * 推导过程：
 *
 * 1. ADC每变化1 LSB对应的电压：
 *
 *                Vref              3.0
 *   V_LSB = ------------- =  ------------- ≈ 0.000732422 V
 *               N_ADC              4096
 *
 * 2. 由于该电压经过INA190放大，因此采样电阻上的实际电压变化：
 *
 *                 V_LSB             0.000732422
 *   V_shunt = ------------- =  ------------- ≈ 0.000014648 V
 *                   G                   50
 *
 * 3. 根据欧姆定律 I = V / R，得到每LSB对应的电流：
 *
 *                 Vref
 *   I_LSB = ---------------------
 *            N_ADC × G × Rshunt
 *
 * 4. 上式单位为 A/LSB，乘以10^6转换为 uA/LSB：
 *
 *                    Vref
 *   I_LSB(uA) = ------------------- × 10^6
 *               N_ADC × G × Rshunt
 * 
 * 注意：\ 符号在预定义中表示“这一行还没结束，下一行继续”，这个符号通常是用来规范代码形式的
 */

//这三个宏定义是三档理论换算比例，单位都是 uA/LSB。
//LOW档：
#define SCALE_LOW_UA_PER_LSB  \
    (ADC_VREF_V / ADC_CODE_COUNT / INA190_GAIN / LOW_CUR_RES * 1000000.0f)

//MID档：
#define SCALE_MID_UA_PER_LSB  \
    (ADC_VREF_V / ADC_CODE_COUNT / INA190_GAIN / MID_CUR_RES * 1000000.0f)

//HIGH档：
#define SCALE_HIGH_UA_PER_LSB \
    (ADC_VREF_V / ADC_CODE_COUNT / INA190_GAIN / HIGH_CUR_RES * 1000000.0f)



/**
 * @brief 三档电流校准参数
 *
 * 每个档位都有两个参数：
 *
 * scale_multiplier：
 *     修正测量比例。
 *     1.0f表示不修正；
 *     1.1f表示理论电流放大10%。
 *
 * offset_ua：
 *     修正零点偏移，单位固定为uA。
 *     例如空载显示+30uA，可暂时设置为-30uA。
 */
typedef struct
{
    float low_scale_multiplier;   // LOW档比例修正系数
    float mid_scale_multiplier;   // MID档比例修正系数
    float high_scale_multiplier;  // HIGH档比例修正系数

    float low_offset_ua;          // LOW档零点补偿，单位uA
    float mid_offset_ua;          // MID档零点补偿，单位uA
    float high_offset_ua;         // HIGH档零点补偿，单位uA
} ADC_Calibration_T;






/* ADC处理任务事件标志。事件只负责唤醒任务，真实数据放在静态环形队列中。 */
#define ADC_FLAG_BLOCK_READY            (1UL << 0)
#define ADC_FLAG_ERROR                  (1UL << 1)
#define ADC_FLAG_ALL_EVENTS             (ADC_FLAG_BLOCK_READY | ADC_FLAG_ERROR)

/* 队列实际最多保存7块，留一个空槽用于区分队列满和队列空。 */
#define ADC_BLOCK_QUEUE_DEPTH           8U


/*
 * 自动换挡ADC阈值。
 *
 * 升档：
 * LOW -> MID
 * MID -> HIGH
 *
 * 降档：
 * HIGH -> MID
 * MID -> LOW
 * 
 * 二、为什么升档快、降档慢
升档采用较少确认次数，例如连续 4 点。
原因：电流突然增大时，当前量程可能接近 ADC 饱和，因此要尽快切换到更大量程。
降档采用较多确认次数，例如连续 10 点。
原因：电流变小时，当前大档仍然可以正常测量，
只是灵敏度较低，因此可以多观察几个采样点，确认电流确实降低后再切档，避免噪声或瞬态变化导致频繁换挡。

补充说明一下：切换量程本质上是当前采样电阻的电压差，对应产生的ADC太大了或者太小了，逼近了12位ADC的上下限
所以如果档位测量的ADC值位快满了，那说明此时采样电阻两端的电压大，要减小采样电阻大小，而高量程经过的电阻值小，所以切换高量程
低量程的经过电阻值大，所以在ADC值快逼近下限值时，才切换低量程，需要更大的采样电阻分压，输出更大的ADC值。
 */

#define AUTO_RANGE_LOW_UP_CODE          1952U
#define AUTO_RANGE_MID_UP_CODE          1952U
#define AUTO_RANGE_MID_DOWN_CODE        15U
#define AUTO_RANGE_HIGH_DOWN_CODE       15U

/* MID通道接近ADC上下限时，强制按过载处理。 */
#define AUTO_RANGE_MID_RAIL_MARGIN      15U

/* 连续满足多少个采样点后，才允许换挡。 */
#define AUTO_RANGE_UP_CONFIRM_TIMES     4U
#define AUTO_RANGE_DOWN_CONFIRM_TIMES   10U


/* 切档后丢弃1个数据块，再冷却5个数据块。 */
#define AUTO_RANGE_SKIP_CHUNKS          1U
#define AUTO_RANGE_COOLDOWN_CHUNKS      5U

/**
 * @brief 自动量程观察状态
 * @note  这些状态只用于LCD调试显示，真正换挡仍由计数器和标志控制。
 * 正常情况：
HOLD
 → UP_WAIT / DOWN_WAIT
 → SWITCH
 → SKIP
 → COOL
 → HOLD
 */
typedef enum
{
    ADC_RANGE_STATE_HOLD = 0,       //保持当前档位，没有满足切档条件。
    ADC_RANGE_STATE_UP_WAIT,        //达到升档阈值，正在连续确认，例如 U2/4。
    ADC_RANGE_STATE_DOWN_WAIT,      //达到降档阈值，正在连续确认，例如 D6/10。
    ADC_RANGE_STATE_SWITCH,         //确认完成，本次真正执行档位切换。
    ADC_RANGE_STATE_SKIP,           //切档后的下一整个 ADC 数据块作废，避免模拟开关、运放等切换瞬态产生的脏数据影响判断。
    ADC_RANGE_STATE_COOL            //切档后的冷却期。ADC 数据可以继续使用，但暂时禁止再次换挡，防止档位来回抖动。
} ADC_AutoRangeState_T;


/**
 * @brief 最近一次换挡原因
 */
typedef enum
{
    ADC_SWITCH_REASON_NONE = 0,         //无切档原因，正常保持当前档位
    ADC_SWITCH_REASON_OVERLOAD,         //当前档位电流太大，比如LOW到MID，MID到HIGH
    ADC_SWITCH_REASON_UNDERLOAD,        //相反
    ADC_SWITCH_REASON_NEAR_RAIL,        //ADC值接近ADC的上下限时（0为下，4095为上），比如当前采样电阻ADC=4090,而最大是4095，所以要逼近了，换挡
    ADC_SWITCH_REASON_RANGE_ERROR       //防止某个BUG出现导致切换到错误档位，可以记录
} ADC_SwitchReason_T;

/**
 * @brief STEP5自动量程调试数据
 */
typedef struct 
{
    bool valid;

     /* 本数据对应的档位，以及当前真实硬件档位。 */
    uint8_t sample_range;           //记录某一次的ADC值是在什么档位采集的
    uint8_t current_range;          //记录当前档位
    uint8_t request_range;          //记录自动切换期望的档位


    /* 当前使用的原始ADC数据。 */
    uint16_t current_adc_raw;       //记录当前量程对应的ADC原始值
    uint16_t ref_adc_raw;           //VREF基准ADC值
    int32_t delta_adc_raw;          //ADC值与VREF基准的ADC差值，即ADC变化值
    uint16_t abs_delta_adc_raw;     //ADC变化值取绝对值

    /* 自动换挡内部状态。 */
    ADC_AutoRangeState_T state;     //记录当前自动量程状态机处于什么状态
    uint8_t up_count;               //记录升档条件达标了几次
    uint8_t down_count;             //记录降档条件达标了几次
    uint8_t skip_chunks;            //切换档位后，由于可能下1ms来的ADC数据，是上一个档位测量得到的，所以丢弃
    uint8_t cooldown_chunks;        //切换档位后，至少要经过几次的ADC值后，才恢复自动切档判断，目的是为了数据的稳定性

    /* 专门用于记录上一次切档的具体情况，方便调试对比用。 */
    uint8_t last_from;              //记录从哪个档位切换
    uint8_t last_to;                //记录切到哪个档位
    uint8_t last_confirm_count;     //记录条件达标的次数
    ADC_SwitchReason_T last_reason; //标记上次为什么切档的原因

    /* 累计统计，防止LCD刷新慢而漏看短暂状态。 */
    uint32_t switch_count;
    uint32_t skipped_block_count;       //累计丢弃过多少个 ADC block。比如：发生2次切档，那么就为2
    uint32_t cooldown_block_count;      //累计从开机到现在累计经历了多少个 cooldown block。如果发生两次切档，每次 COOL 5 block：即10
    uint32_t lost_block_count;          //队列满、ADC错误或档位已变化时，累计丢弃的数据块数量


}ADC_AutoRangeDebug_T;


/*
 * ADC采样率为10kHz。
 * LCD每250ms更新一次平均值，对应约2500个采样点。
 */
#define DATA_MONITOR_SAMPLE_RATE_HZ      10000U
#define DATA_MONITOR_PERIOD_MS           250U
#define DATA_MONITOR_SAMPLE_COUNT        \
    ((DATA_MONITOR_SAMPLE_RATE_HZ * DATA_MONITOR_PERIOD_MS) / 1000U)

/**
 * @brief 获取LCD使用的平均电压和平均电流
 * @param out_voltage_v 平均电压，单位V
 * @param out_current_ua 平均电流，单位uA
 * @param out_range 最近一个有效采样点使用的量程
 * @return true 已经产生第一组平均值
 * @return false 尚未累计够一个统计窗口或参数错误
 */
bool Data_Monitor_GetValues(float *out_voltage_v,
                            float *out_current_ua,
                            uint8_t *out_range);



/**
 * @brief 一个DMA半区的五通道平均原始值
 */
typedef struct 
{
    uint16_t vol_raw;
    uint16_t low_raw;
    uint16_t mid_raw;
    uint16_t high_raw;
    uint16_t ref_raw;
}ADC_RawAverage_T;


/*
 * DMA原始采样缓冲区：
 * 第0～9行是前半区；
 * 第10～19行是后半区；
 * 每行依次保存VOL、LOW、MID、HIGH、REF五个通道。
 */
extern uint16_t adc_raw_buffer[ADC_DMA_ROWS][ADC_CHANNELS];

/* 临时变量：调试计数器，可在Keil Watch窗口中观察。 */
extern volatile uint32_t adc_half_count;
extern volatile uint32_t adc_full_count;
extern volatile uint32_t adc_error_count;

/**
 * @brief 启动ADC定时触发和DMA循环采样
 * @return HAL_OK 启动成功
 * @return 其他值 ADC或TIM2启动失败
 */
HAL_StatusTypeDef ADC_Sampling_Start(void);
void ADC_Sampling_Stop(void);

/**
 * @brief 获取最近一个完整DMA半区的五通道平均值
 * @param average 用于保存平均结果
 * @return true 已经取得有效采样
 * @return false ADC还没有完成第一个半区
 */
bool ADC_GetRawAverage(ADC_RawAverage_T *average);

/**
 * @brief 把电压通道VADC值换算成VBUS电压
 * @param vol_raw 电压通道平均原始值
 * @return VBUS电压，单位V
 */
float ADC_ConvertVoltage_V(uint16_t vol_raw);

/**
 * @brief 将ADC原始值换算成经过校准后的电流
 * @param current_adc_raw 当前量程下，电流检测通道的ADC原始值
 * @param ref_adc_raw     1.5V参考通道的ADC原始值
 * @param range           当前采样档位（LOW / MID / HIGH）
 * @return 校准后的电流值，单位：uA
 * @note
 * ============================================================
 * 整体计算流程：
 *  ① 计算ADC差值
 *      ΔADC = ADC_current - ADC_ref
 *  ② ADC差值 → INA190输出端电压差
 *                         Vref
 *      ΔV_INA = ΔADC × -----------
 *                        N_ADC
 *  ③ 去除INA190增益，得到采样电阻两端的真实电压
 *                   ΔV_INA
 *      V_shunt = ------------
 *                      G
 *
 *  ④ 根据欧姆定律 I = U / R，计算理论电流
 *
 *                     V_shunt
 *      I_theory = ---------------- × 10^6
 *                     R_shunt
 *
 *  ⑤ 使用当前档位的校准参数修正理论电流
 *
 *      I_cal = I_theory × K + Offset
 *
 *
 * 因此完整公式为：
 *
 *                    Vref              1
 * I_cal = ΔADC × ----------- × ---------------- × 10^6 × K + Offset
 *                   N_ADC        G × R_shunt
 *
 * 其中：
 *
 *   ΔADC       : 电流ADC值与1.5V参考ADC值之差
 *   Vref       : ADC参考电压
 *   N_ADC      : ADC总码数
 *   G          : INA190增益
 *   R_shunt    : 当前档位采样电阻
 *   K          : 当前档位比例修正系数 scale_multiplier
 *   Offset     : 当前档位零点补偿，单位uA
 *
 * 最终输出单位：uA
 * ============================================================
 */
float ADC_ConvertCurrent_uA(uint16_t current_adc_raw,
                           uint16_t ref_adc_raw,
                           uint8_t range);

/**
 * @brief 复位自动量程算法
 */
void ADC_AutoRange_Reset(void);

/**
 * @brief 处理一个DMA半区的10组ADC数据
 * @param chunk_ptr DMA半区首地址
 * @return true 本块已经正常处理
 * @return false 本块为空或因切档稳定时间被丢弃
 */
bool Process_ADC_Chunk(const uint16_t *chunk_ptr);

/**
 * @brief 处理ADC中断已经发布到队列中的全部数据块
 * @return 本次实际处理的数据块数量
 * @note 只能由ADC处理任务调用，不能在中断或LCD任务中调用
 */
uint32_t ADC_ProcessPendingBlocks(void);

/**
 * @brief 获取STEP5自动量程调试数据
 * @param debug 用于保存调试数据
 * @return true 已经有有效数据
 * @return false 参数错误或ADC尚未产生数据
 */
bool ADC_AutoRange_GetDebug(ADC_AutoRangeDebug_T *debug);



/**
 * @brief 填入默认校准参数
 * @param cfg 校准参数输出地址
 *
 * 默认倍率为1.0，偏移为0，表示完全使用理论换算值。
 */
void ADC_Calibration_SetDefault(ADC_Calibration_T *cfg);

/**
 * @brief 检查三档校准参数是否合法
 * @param cfg 待检查的校准参数
 * @return true 参数有效；false 参数异常
 */
bool ADC_Calibration_IsValid(const ADC_Calibration_T *cfg);





/**
 * @brief 获取最近生成的一包USB波形数据
 * @param packet 波形包输出地址，允许传NULL
 * @param sequence 波形包序号输出地址，允许传NULL
 * @return true 当前已有有效波形包
 * @return false 尚无有效数据，或者两个输出参数都为NULL
 *
 * @note 目前只允许MessageSendTask调用。
 */
bool ADC_GetLatestUsbPacket(USB_ADC_Packet_T *packet,
                            uint32_t *sequence);








#ifdef __cplusplus
}
#endif

#endif
