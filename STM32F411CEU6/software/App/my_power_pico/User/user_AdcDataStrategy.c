


#include "user_AdcDataStrategy.h"
#include "user_TasksInit.h"
#include "BL24C02.h"
#include "gate.h"





/* DMA循环采样缓冲区，共100个uint16_t，占用200字节。 */
uint16_t adc_raw_buffer[ADC_DMA_ROWS][ADC_CHANNELS];


/*
 * 最近生成的一包USB波形数据。
 * 该缓存只在MessageSendTask上下文中产生和读取，不由ADC中断访问。
 */
static USB_ADC_Packet_T s_usb_packet;
static uint32_t s_usb_packet_sequence;
static bool s_usb_packet_valid;


/*
 * 中断生产、ADC任务消费的单生产者/单消费者环形队列。
 * 每个槽保存一块10组五通道数据和它的发布序号。
 */
typedef struct
{
    uint32_t sequence;          //数据序号
    uint8_t sample_range;       //当前真实档位
    uint8_t reserved;           //保留位
    uint16_t samples[ADC_TIMES][ADC_CHANNELS]; //10组五通道ADC数据，即1个半区
} ADC_BlockSlot_T;

static ADC_BlockSlot_T s_adc_block_queue[ADC_BLOCK_QUEUE_DEPTH];
static volatile uint8_t s_adc_block_write_index;
static volatile uint8_t s_adc_block_read_index;
static volatile uint32_t s_adc_block_sequence;
static uint32_t s_adc_error_seen;
static uint32_t s_adc_last_processed_sequence;
static volatile uint32_t s_adc_error_count;

/*ADC采集启动标志*/
static bool s_adc_sampling_started;

/* 连续过载和连续欠载确认次数。 */
static uint8_t s_auto_up_count;
static uint8_t s_auto_down_count;

/* 切档后的数据丢弃和冷却计数。 */
static uint8_t s_auto_skip_chunks;
static uint8_t s_auto_cooldown_chunks;

/**
 * @brief LCD显示数据统计结构
 *
 * 电压使用mV累加。
 * 电流使用10nA累加，1uA等于100个10nA。
 */
typedef struct
{
    uint64_t sum_voltage_mv;
    int64_t sum_current_10na;
    uint32_t sample_count;

    float average_voltage_v;
    float average_current_ua;

    uint8_t last_range;
    bool average_valid;
} Data_Monitor_T;

static Data_Monitor_T s_data_monitor;



/***********增加内部辅助函数，仅在本文中使用*************/
/**
 * @brief 计算两个ADC码值的绝对差值
 * @param a 第一个ADC码值
 * @param b 第二个ADC码值
 * @return 两个码值相差多少个LSB
 */
static uint16_t ADC_AbsDiffU16(uint16_t a,uint16_t b)
{
    if (a>=b)
    {
        return a-b;
    }
    return b-a;
}

/**
 * @brief 根据当前真实档位选择对应电流通道
 * @param sample_row 一组五通道ADC数据
 * @param range 当前真实硬件档位
 * @return 当前档位对应的电流ADC码值
 */
static uint16_t ADC_SelectCurrentRaw(const uint16_t *sample_row, uint8_t range)
{
        /*注意：这里用了return之后，就不用break了*/
    switch (range)
    {
        case LOW_CUR:
            return sample_row[IDX_LOW];

        case MID_CUR:
            return sample_row[IDX_MID];

        case HIGH_CUR:
            return sample_row[IDX_HIGH];

        default:
            /*
             * 档位记录异常时，临时选择HIGH通道。
             * HIGH档测量范围最大，是相对安全的兜底。
             */
            return sample_row[IDX_HIGH];
    }       
    
}

/**
 * @brief 把一个稳定ADC数据块封装成USB波形包
 * @param slot 已经由ADC任务取出的稳定数据块
 */
static void ADC_BuildUsbPacket(const ADC_BlockSlot_T *slot)
{
    uint8_t i;
    const uint16_t *sample_row;

    if (slot == NULL)
    {
        return;
    }

    s_usb_packet.header[0] = PACKET_HEADER_0;
    s_usb_packet.header[1] = PACKET_HEADER_1;
    s_usb_packet.timestamp = GetMicrosecondCounter();
    s_usb_packet.data_count = ADC_TIMES;

    for (i = 0U; i < ADC_TIMES; i++)
    {
        sample_row = &slot->samples[i][0];

        /*
         * 使用slot记录的采样档位，不能重新读取当前GPIO档位。
         * 因为处理本块期间，自动量程算法可能已经完成切档。
         */
        s_usb_packet.samples[i].range =
            slot->sample_range;

        s_usb_packet.samples[i].vol_adc =
            sample_row[IDX_VOL];

        s_usb_packet.samples[i].cur_adc =
            ADC_SelectCurrentRaw(sample_row,
                                 slot->sample_range);

        s_usb_packet.samples[i].ref_adc =
            sample_row[IDX_REF];
    }

    s_usb_packet_sequence++;
    s_usb_packet_valid = true;
}

/**
 * @brief 获取最近生成的一包USB波形数据
 */
bool ADC_GetLatestUsbPacket(USB_ADC_Packet_T *packet,
                            uint32_t *sequence)
{
    if (!s_usb_packet_valid
        || (packet == NULL && sequence == NULL))
    {
        return false;
    }

    if (packet != NULL)
    {
        memcpy(packet,
               &s_usb_packet,
               sizeof(*packet));
    }

    if (sequence != NULL)
    {
        *sequence = s_usb_packet_sequence;
    }

    return true;
}

/**
 * @brief 清除连续确认次数
 */
static void ADC_AutoRange_ClearConfirmCount(void)
{
    s_auto_up_count = 0U;
    s_auto_down_count = 0U;
}



/**
 * @brief 复位自动量程算法
 */
void ADC_AutoRange_Reset(void)
{
    ADC_AutoRange_ClearConfirmCount();
    /*初始化计数值*/
    s_auto_skip_chunks = 0;
    s_auto_cooldown_chunks = 0;
}



/**
 * @brief 填入一组未修正理论值的默认校准参数
 * @param cfg 校准参数输出地址
 */
void ADC_Calibration_SetDefault(ADC_Calibration_T *cfg)
{
    if (cfg==NULL)
    {
        return;
    }
    /*
     * multiplier = 1.0：
     * 理论电流保持原值，不放大也不缩小。
     */
    cfg->low_scale_multiplier = 1.0f;
    cfg->mid_scale_multiplier = 1.0f;
    cfg->high_scale_multiplier = 1.0f;
    /*
     * offset = 0：
     * 暂时不补偿三档零点偏移。
     */
    cfg->low_offset_ua = 0.0f;
    cfg->mid_offset_ua = 0.0f;
    cfg->high_offset_ua = 0.0f;
}

/**
 * @brief 检查单个档位的校准参数
 * @param hardware_scale_ua_per_lsb 当前档位理论比例，单位uA/LSB
 * @param multiplier 比例修正系数
 * @param offset_ua 零点补偿，单位uA
 * @return true 参数有效；false 参数异常
 *
 * @note
 * 0.75~1.25和正负32LSB是官方源码采用的软件保护范围，
 * 不是STM32或INA190的硬性规定。
 */
static bool ADC_Calibration_RangeIsValid(
    float hardware_scale_ua_per_lsb,
    float multiplier,
    float offset_ua)
{
    float offset_lsb;
    /*
     * isfinite()用于排除NaN和正负无穷。
     *
     * 同时把倍率限制在理论值的75%~125%，
     * 避免EEPROM损坏后产生极端显示值。
     */

    if (!isfinite(multiplier)
        || !isfinite(offset_ua)
        || multiplier < 0.75f
        || multiplier > 1.25f)
    {
        return false;
    }

    /*
     * 把offset从uA反算成等效ADC码差。
     *
     * 例如LOW档offset=5uA，
     * 再除以LOW档每LSB对应的电流，
     * 就能判断它大约相当于多少个ADC码。
     */
    offset_lsb =
        offset_ua /
        (hardware_scale_ua_per_lsb * multiplier);

    /*
     * 三档统一限制在正负32个ADC码以内。
     * 这样虽然三档允许的uA偏移不同，
     * 但对应的硬件零点偏差范围相同。
     */
    return isfinite(offset_lsb)
        && fabsf(offset_lsb) <= 32.0f;

}


/**
 * @brief 检查低、中、高三档校准参数
 * @param cfg 待检查的校准参数
 * @return true 三档全部有效；false 任意一档异常
 */
bool ADC_Calibration_IsValid(const ADC_Calibration_T *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    return ADC_Calibration_RangeIsValid(
               SCALE_LOW_UA_PER_LSB,
               cfg->low_scale_multiplier,
               cfg->low_offset_ua)
        && ADC_Calibration_RangeIsValid(
               SCALE_MID_UA_PER_LSB,
               cfg->mid_scale_multiplier,
               cfg->mid_offset_ua)
        && ADC_Calibration_RangeIsValid(
               SCALE_HIGH_UA_PER_LSB,
               cfg->high_scale_multiplier,
               cfg->high_offset_ua);
}

/************************************************************************/

/**
 * @brief 清空LCD显示统计值
 */
static void Data_Monitor_Clear(void)
{
    memset(&s_data_monitor, 0, sizeof(s_data_monitor));
}

/**
 * @brief 累加一个有效ADC采样点
 * @param voltage_adc 电压通道ADC原始值
 * @param current_adc 当前档电流通道ADC原始值
 * @param ref_adc 1.5V参考通道ADC原始值
 * @param range 本采样点对应的真实量程
 */
static void Data_Monitor_Update(uint16_t voltage_adc,
                                uint16_t current_adc,
                                uint16_t ref_adc,
                                uint8_t range)
{
    float voltage_mv;
    float current_ua;
    float divider_error_10na;
    float final_current_10na;

    /*
     * 将VBUS电压换算成mV。
     */
    voltage_mv = (float)voltage_adc
               * (ADC_VREF_V * 1000.0f / ADC_MAX_CODE)
               * VBUS_DIVIDER_GAIN;


    /*
     * 每个采样点按照产生它的真实档位进行换算。
     * 校准倍率和零点偏移会在该函数内生效。
     */
    current_ua = ADC_ConvertCurrent_uA(current_adc,ref_adc,range);


    /*
     * 在官方代码中会扣除VBUS分压电阻消耗的少量电流。
     *例如
     * voltage_mv / 11得到的单位正好是10nA。
     * 例如VBUS=5V时，修正量约为4.55uA。
     */
    divider_error_10na  = voltage_mv / 11.0f;

    final_current_10na = current_ua * 100.0f - divider_error_10na;

    s_data_monitor.sum_voltage_mv += (uint64_t)voltage_mv;

    /* 电流允许为负值，必须按有符号数累加。 */
    s_data_monitor.sum_current_10na += (int64_t)final_current_10na;

    s_data_monitor.sample_count++;
    s_data_monitor.last_range = range;

    /*累加2500次后发布一次新的平均值*/
    if (s_data_monitor.sample_count >= DATA_MONITOR_SAMPLE_COUNT)
    {
        s_data_monitor.average_voltage_v = (float)s_data_monitor.sum_voltage_mv / (float)s_data_monitor.sample_count /1000.0F;
        s_data_monitor.average_current_ua = (float)s_data_monitor.sum_current_10na / (float)s_data_monitor.sample_count / 100.0F;

        s_data_monitor.sum_voltage_mv = 0;
        s_data_monitor.sum_current_10na = 0;
        s_data_monitor.sample_count = 0;
        s_data_monitor.average_valid = true;
    }

}


/**
 * @brief 获取LCD使用的平均测量值
 */
bool Data_Monitor_GetValues(float *out_voltage_v,
                            float *out_current_ua,
                            uint8_t *out_range)
{
    if ((out_voltage_v == NULL)
        || (out_current_ua == NULL)
        || (out_range == NULL)
        || !s_data_monitor.average_valid)
    {
        return false;
    }
    /*
     * 本函数与更新函数都运行在MessageSendTask中，
     * 不需要关闭中断。
     */
    *out_voltage_v = s_data_monitor.average_voltage_v;
    *out_current_ua = s_data_monitor.average_current_ua;
    *out_range = s_data_monitor.last_range;

    return true;
}
















/********************************自动切换量程核心代码****************************************/

/**
 * @brief 判断LOW档是否需要升到MID档
 * @param abs_delta_adc_raw 电流通道与参考通道的ADC绝对差值
 * @return 继续保持LOW档，或者请求切换到MID档
 */
static uint8_t ADC_AutoRange_CheckLow(uint16_t abs_delta_adc_raw)
{
    /* LOW已经是最小电流档，不存在继续降档。 */
    s_auto_down_count = 0U;

    if (abs_delta_adc_raw >= AUTO_RANGE_LOW_UP_CODE)
    {
        if (s_auto_up_count < AUTO_RANGE_UP_CONFIRM_TIMES)
        {
            s_auto_up_count++;
        }

        if (s_auto_up_count >= AUTO_RANGE_UP_CONFIRM_TIMES)
        {
            return MID_CUR;
        }
    }
    else
    {
        /* 中途一个采样点不满足条件，连续确认必须重新开始。 */
        s_auto_up_count = 0U;
    }

    return LOW_CUR;
}

/**
 * @brief 判断MID档需要升档、降档还是保持
 * @param current_adc_raw MID电流通道的原始ADC值
 * @param abs_delta_adc_raw 电流通道与参考通道的ADC绝对差值
 * @return LOW、MID或HIGH目标档位
 */
static uint8_t ADC_AutoRange_CheckMid(uint16_t current_adc_raw,
                                      uint16_t abs_delta_adc_raw)
{
    bool mid_near_rail;

    /*
     * MID通道接近0或4095时，即使差值判断异常，
     * 也优先按过载处理，避免ADC饱和后继续停在MID档。
     */
    mid_near_rail =
        (current_adc_raw <= AUTO_RANGE_MID_RAIL_MARGIN)
        || (current_adc_raw >=
            (4095U - AUTO_RANGE_MID_RAIL_MARGIN));

    /* 升档优先，升档和降档两个计数器不能同时累加。 */
    if ((abs_delta_adc_raw >= AUTO_RANGE_MID_UP_CODE)
        || mid_near_rail)
    {
        s_auto_down_count = 0U;

        if (s_auto_up_count < AUTO_RANGE_UP_CONFIRM_TIMES)
        {
            s_auto_up_count++;
        }

        if (s_auto_up_count >= AUTO_RANGE_UP_CONFIRM_TIMES)
        {
            return HIGH_CUR;
        }
    }
    else if (abs_delta_adc_raw <= AUTO_RANGE_MID_DOWN_CODE)
    {
        s_auto_up_count = 0U;

        if (s_auto_down_count < AUTO_RANGE_DOWN_CONFIRM_TIMES)
        {
            s_auto_down_count++;
        }

        if (s_auto_down_count >= AUTO_RANGE_DOWN_CONFIRM_TIMES)
        {
            return LOW_CUR;
        }
    }
    else
    {
        /* 当前幅度位于上下阈值之间，处于保持区。 */
        ADC_AutoRange_ClearConfirmCount();
    }

    return MID_CUR;
}

/**
 * @brief 判断HIGH档是否需要降到MID档
 * @param abs_delta_adc_raw 电流通道与参考通道的ADC绝对差值
 * @return 继续保持HIGH档，或者请求切换到MID档
 */
static uint8_t ADC_AutoRange_CheckHigh(uint16_t abs_delta_adc_raw)
{
    /* HIGH已经是最大电流档，不存在继续升档。 */
    s_auto_up_count = 0U;

    if (abs_delta_adc_raw <= AUTO_RANGE_HIGH_DOWN_CODE)
    {
        if (s_auto_down_count < AUTO_RANGE_DOWN_CONFIRM_TIMES)
        {
            s_auto_down_count++;
        }

        if (s_auto_down_count >= AUTO_RANGE_DOWN_CONFIRM_TIMES)
        {
            return MID_CUR;
        }
    }
    else
    {
        s_auto_down_count = 0U;
    }

    return HIGH_CUR;
}

/**
 * @brief 根据当前真实档位执行一次自动量程判断
 * @param current_range 当前真实硬件档位
 * @param current_adc_raw 当前档位对应的电流ADC值
 * @param abs_delta_adc_raw 电流通道与参考通道的ADC绝对差值
 * @return 本采样点判断出的目标档位
 */
static uint8_t ADC_AutoRange_DecideRange(uint8_t current_range,
                                         uint16_t current_adc_raw,
                                         uint16_t abs_delta_adc_raw)
{
    switch (current_range)
    {
        case LOW_CUR:
            return ADC_AutoRange_CheckLow(abs_delta_adc_raw);

        case MID_CUR:
            return ADC_AutoRange_CheckMid(current_adc_raw,
                                          abs_delta_adc_raw);

        case HIGH_CUR:
            return ADC_AutoRange_CheckHigh(abs_delta_adc_raw);

        default:
            /* 未知负载优先回到最大量程，避免大电流经过50Ω采样电阻。 */
            return HIGH_CUR;
    }
}

/**
 * @brief 处理一个DMA半区，并执行自动量程判断
 * @param chunk_ptr DMA半区首地址，包含10组五通道ADC数据
 * @return true 本块已经处理
 * @return false 本块为空或处于切档后的丢弃阶段
 */
bool Process_ADC_Chunk(const uint16_t* chunk_ptr)
{
    uint8_t gate_mode;           //当前开关模式
    uint8_t current_range;       //当前档位
    uint8_t request_range;       //请求档位
    uint8_t i;                   //循环变量
    bool decision_enabled;       //是否允许判断换挡

    uint16_t current_adc_raw;   //当前通道原始ADC值
    uint16_t ref_adc_raw;       //参考通道原始ADC值
    uint16_t abs_delta_adc_raw; //绝对差值

    if (chunk_ptr == NULL)
    {
        /*空指针，直接退出函数*/
        return false;
    }

    /*先保持当前档位*/
    gate_mode = Gate_Get_Mode();
    current_range = Gate_get_status();
    request_range = current_range;

    /*
     * 刚切档后的下一整块数据不可信。
     * 原因：
     * MOS开关、采样电阻流路和INA190输出都需要一点稳定时间。
     * 所以这里整块丢弃，不参与换挡判断，也不参与正式数据显示。
     */
    if (s_auto_skip_chunks > 0)
    {
        s_auto_skip_chunks--;
        return false;                       //返回false，表示本块数据不可信，退出函数
    }
    

    /*
     * 手动模式下不执行自动换挡。
     * 防止切回自动模式后，沿用之前残留的确认次数。
     */
    if (gate_mode!=GATE_MODE_AUTO)
    {
        ADC_AutoRange_ClearConfirmCount();//清除所有自动量程的计数值
        s_auto_skip_chunks = 0U;
        s_auto_cooldown_chunks = 0U;

    }



    /*3.处理是否自动量程状态
     * 只有处于自动模式并且冷却结束后，才允许判断换挡。
     * 冷却期间仍然读取ADC和更新显示，只是不允许再次切档。
     * 
     */
    decision_enabled  = (gate_mode == GATE_MODE_AUTO) && (s_auto_cooldown_chunks==0);//要满足门状态和冷却计算=0

    /*
     * 一个DMA半区有10组采样。
     * 连续确认使用的是“连续采样点”，不是LCD刷新次数。
     */
    for ( i = 0; i < ADC_TIMES; i++)
    {
        const uint16_t *sample_row;

        sample_row = chunk_ptr + ((uint32_t)i*ADC_CHANNELS);                //这里是移动下一组5通道ADC数据的意思
        /*比如：i = 0，chunk_ptr + 0 × 5= chunk_ptr + 0
        当i=1 ，= chunk_ptr + 5
        V0 L0 M0 H0 R0 V1 L1 M1 H1 R1 V2 L2 M2 H2 R2 ...
        ↑              ↑              ↑
        0              5              10*/


        /*获取当前档位对应的原始ADC值*/
        current_adc_raw = ADC_SelectCurrentRaw(sample_row,current_range);
        /*获取参考通道原始ADC值*/
        ref_adc_raw = sample_row[IDX_REF];

        abs_delta_adc_raw =
            ADC_AbsDiffU16(current_adc_raw, ref_adc_raw);

        /*
        * 只有通过切档稳定期检查的有效数据才参与LCD平均。
        * USB波形包仍然使用原来的原始数据链路。
        */
        Data_Monitor_Update(sample_row[IDX_VOL],
                            current_adc_raw,
                            ref_adc_raw,
                            current_range);


        if (!decision_enabled)
        {
            /*
             * 手动模式或冷却期间只更新数据，
             * 不执行下面的自动换挡判断。
             */
            continue;
        }




        request_range =
            ADC_AutoRange_DecideRange(current_range,
                                      current_adc_raw,
                                      abs_delta_adc_raw);

        /*
         * 一旦确认需要换挡，就停止继续处理本块后面的采样点。
         * 真正的GPIO切换统一放到for循环结束后执行。
         */
        if (request_range != current_range)
        {
            break;
        }




        
    }
    
     
    /*
     * 在数据块处理结束后再真正写GPIO。
     * 避免一个数据块的前半部分属于旧档、后半部分属于新档。
     */
    if ((gate_mode==GATE_MODE_AUTO) && (request_range != current_range))
    {
        /*如果门状态为自动 并且 期望档位≠当前档位*/
        flow_route_selection(request_range);    //真正切换电路测量档位，改GPIO设置

        /*
         * 下一块直接丢弃。因为下快数据是旧档位测量得到的
         * 丢弃结束后，再经过5个有效数据块的冷却期。
         */
        s_auto_skip_chunks = AUTO_RANGE_SKIP_CHUNKS;//标记丢弃下一个数据
        s_auto_cooldown_chunks = AUTO_RANGE_COOLDOWN_CHUNKS;//标记冷却5个数据期
        ADC_AutoRange_ClearConfirmCount();  //清空两个计数器
    }
        else if (s_auto_cooldown_chunks > 0U)
    {
        /*
         * 冷却期的数据仍然可以用于显示。
         * 每正常处理完一块，冷却计数减1。
         */
        s_auto_cooldown_chunks--;
    }

    return true;






}








/************************************************************************/

/**
 * @brief 启动ADC定时触发和DMA循环采样
 * @return HAL_OK 启动成功
 * @return 其他值 ADC或TIM2启动失败
 */
HAL_StatusTypeDef ADC_Sampling_Start(void)
{
    HAL_StatusTypeDef status;

    /*1.避免重复启动*/
    if (s_adc_sampling_started)
    {
        return HAL_OK;
    }
    
    /*2.清空缓冲区*/
    memset(adc_raw_buffer,0,sizeof(adc_raw_buffer));
    s_adc_error_count = 0U;
    /* 新一轮ADC采样不能沿用上一次LCD平均值。 */
    Data_Monitor_Clear();

    memset(s_adc_block_queue, 0, sizeof(s_adc_block_queue));
    s_adc_block_write_index = 0U;
    s_adc_block_read_index = 0U;
    s_adc_block_sequence = 0U;
    s_adc_error_seen = 0U;
    s_adc_last_processed_sequence = 0U;


    /*
     * 启动采样时清空波形内容，但包序号保持单调递增。
     * 这样ADC停止后重新启动，不会让发送任务把序号回退误判成大量新包。
     */
    memset(&s_usb_packet, 0, sizeof(s_usb_packet));
    s_usb_packet_valid = false;


    /*
     * 3.必须先启动ADC DMA，再启动TIM2。
     * 否则TIM2可能已经发出触发，但DMA还没准备好接收数据。
     */
    /*注意， 10次采样才为1轮，即1个半区填充，所以一轮是100字节（因为存储空间是16位，为单位）*/
    status=HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_raw_buffer[0][0],ADC_DMA_LENGTH);
    if (status!=HAL_OK)
    {
        return status;
    }
    
    status = HAL_TIM_Base_Start(&htim2);
    if (status != HAL_OK)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        return status;
    }
    
    s_adc_sampling_started=true;

    return HAL_OK;

    
}




/**
 * @brief 停止TIM2触发和ADC DMA采样
 */
void ADC_Sampling_Stop(void)
{
    HAL_TIM_Base_Stop(&htim2);//先停时钟，切断触发源，后再切断ADC转换
    HAL_ADC_Stop_DMA(&hadc1);
    s_adc_sampling_started=false;
}

/**
 * @brief 把ADC差值换算成带校准的电流
 * @param current_adc_raw 当前量程电流通道ADC值
 * @param ref_adc_raw 1.5V参考通道ADC值
 * @param range 当前采样档位
 * @return 校准后的电流，单位uA
 *
 * @note 完整公式：
 * 电流 = 理论电流 × 当前档倍率 + 当前档零点补偿
 */
float ADC_ConvertCurrent_uA(
    uint16_t current_adc_raw,
    uint16_t ref_adc_raw,
    uint8_t range)
{
    /*
     * 当前ADC校准参数。
     *
     * 里面保存LOW / MID / HIGH三档各自的：
     *   1. scale_multiplier 比例修正系数
     *   2. offset_ua        零点补偿
     */
    ADC_Calibration_T calibration;

    int32_t delta_adc_raw;

    float shunt_resistance_ohm;     // 当前档位采样电阻，单位Ω
    float scale_multiplier;         // 当前档位比例校准系数
    float offset_ua;                // 当前档位零点补偿，单位uA

    /*
     * 中间计算变量。
     */
    float ina_output_delta_v;     // INA190输出端电压差，单位V
    float shunt_voltage_v;        // 采样电阻两端真实电压，单位V
    float theoretical_current_ua; // 理论电流，单位uA
    float calibrated_current_ua;  // 最终校准电流，单位uA

    /*
     * 一定先转成有符号整数再相减。
     *
     * 例如：
     * current=2043，ref=2048
     * 正确结果应该是-5。
     */
    delta_adc_raw =
        (int32_t)current_adc_raw
        - (int32_t)ref_adc_raw;

    /*
     * 从RAM设置副本读取当前校准值。
     * 这里不会访问EEPROM，所以不会阻塞ADC处理。
     */
    Sys_Get_AdcCalibration(&calibration);


    /* ========================================================
     * ③ 根据当前量程选择对应参数
     * 不同档位具有不同的：
     *
     *     采样电阻 R_shunt
     *     比例校准 K
     *     零点补偿 Offset
     *
     * ======================================================== */
    switch (range)
    {
        case LOW_CUR:
            shunt_resistance_ohm = LOW_CUR_RES;
            scale_multiplier =
                calibration.low_scale_multiplier;
            offset_ua =
                calibration.low_offset_ua;
            break;

        case MID_CUR:
            shunt_resistance_ohm = MID_CUR_RES;
            scale_multiplier =
                calibration.mid_scale_multiplier;
            offset_ua =
                calibration.mid_offset_ua;
            break;

        case HIGH_CUR:
            shunt_resistance_ohm = HIGH_CUR_RES;
            scale_multiplier =
                calibration.high_scale_multiplier;
            offset_ua =
                calibration.high_offset_ua;
            break;

        default:
            /*
             * 档位参数异常时无法确定采样电阻，
             * 返回0，避免使用错误档位进行换算。
             */
            return 0.0f;
    }

    /* ========================================================
     * ④ ADC码差 → INA190输出端电压差
     *
     * ADC每1 LSB对应的电压：
     *
     *                  Vref
     *     V_LSB = -------------
     *                 N_ADC
     *
     * 所以：
     *
     *                         Vref
     *     ΔV_INA = ΔADC × -----------
     *                        N_ADC
     *
     * 单位：
     *
     *     LSB × V/LSB = V
     *
     * 结果 ina_output_delta_v 的单位为 V。
     * ======================================================== */
    ina_output_delta_v =
        (float)delta_adc_raw
        * (ADC_VREF_V / ADC_CODE_COUNT);

    /*
     * 第二步：
     * 去掉INA190A2的50倍放大，
     * 恢复采样电阻两端的真实电压。
     */
    shunt_voltage_v =
        ina_output_delta_v / INA190_GAIN;

    /*
     * 第三步：
     * I = U / R，得到理论电流。
     * 最后乘1000000，把A转换成uA。
     */
    theoretical_current_ua =
        shunt_voltage_v
        / shunt_resistance_ohm
        * 1000000.0f;

        
    /* ========================================================
     * ⑦ 应用软件校准参数
     * 最终公式：
     *     I_cal = I_theory × K + Offset
     * 其中：
     *     K      = scale_multiplier
     *     Offset = offset_ua
     * multiplier用于修正比例误差：
     *     1.00 → 不修正
     *     1.05 → 理论电流增加5%
     *     0.95 → 理论电流减少5%
     * offset用于修正零点误差：
     *
     *     0uA   → 不修正
     *     -30uA → 最终结果减30uA
     *     +30uA → 最终结果加30uA
     * 默认校准参数：
     *     K      = 1.0
     *     Offset = 0
     * 此时：
     *     I_cal = I_theory
     * ======================================================== */
    calibrated_current_ua =
        theoretical_current_ua
        * scale_multiplier
        + offset_ua;

    return calibrated_current_ua;
}


/**
 * @brief 计算环形队列的下一个槽位
 */
static uint8_t ADC_BlockQueue_NextIndex(uint8_t index)
{
    index++;

    if (index >= ADC_BLOCK_QUEUE_DEPTH)
    {
        index = 0U;
    }

    return index;
}

/**
 * @brief 在ADC中断中发布一块完整数据
 * @param source 已完成DMA半区的首地址
 * @note 队列满时丢弃最新块，序号断点会让任务识别丢块
 */
static void ADC_BlockQueue_PublishFromISR(const uint16_t *source)
{
    uint8_t write_index;        //当前写指针
    uint8_t next_index;         //下一个写指针
    ADC_BlockSlot_T *slot;      //这个是指向当前写指针的槽位指针，用于填充数据

    write_index = s_adc_block_write_index;              //获取当前写指针
    next_index = ADC_BlockQueue_NextIndex(write_index);   //获取下一个写指针

    /* 每次DMA半区完成都占用一个序号，丢块后任务能识别序号断点。 */
    s_adc_block_sequence++;

    if (next_index == s_adc_block_read_index)       //如果下一个写指针等于读指针，说明队列已满
    {
        return;
    }

    slot = &s_adc_block_queue[write_index];              //获取当前写指针对应的槽位指针
    slot->sequence = s_adc_block_sequence;                 //设置数据序号
    slot->sample_range = Gate_get_status();              //设置当前真实档位
    memcpy(slot->samples, source, sizeof(slot->samples)); //复制数据到槽位中

    /* 数据全部写完后才能发布新的写指针。 */
    __DMB();
    s_adc_block_write_index = next_index;              //更新写指针
}

/**
 * @brief 处理ADC队列中已经完成的全部数据块
 * @return 本次处理的数据块数量
 */
uint32_t ADC_ProcessPendingBlocks(void)
{
    uint8_t read_index;
    uint32_t processed_count;
    uint32_t error_count;
    uint32_t block_sequence;
    uint8_t published_write_index;
    ADC_BlockSlot_T *slot;
    bool block_valid;

    processed_count = 0U;
    error_count = s_adc_error_count;

    /* ADC硬件报错后取消连续确认，避免把错误前后的数据拼成一次换挡。 */
    if (error_count != s_adc_error_seen)
    {
        ADC_AutoRange_ClearConfirmCount();

        /*
         * 错误发生前后数据的连续性已经无法保证。
         * 丢掉此刻已经排队的块，防止把错误两侧的采样拼成一次连续确认。
         */
        published_write_index = s_adc_block_write_index;    //获取当前写指针

        while (s_adc_block_read_index != published_write_index)
        {   /* 当读指针不等于写指针时，说明队列中还有数据 */
            s_adc_block_read_index =
                ADC_BlockQueue_NextIndex(s_adc_block_read_index);
        }

        s_adc_last_processed_sequence = 0U;

        s_adc_error_seen = error_count;
    }

    for (;;)
    {
        read_index = s_adc_block_read_index;     //获取当前读指针

        if (read_index == s_adc_block_write_index)
        {
            /* 当读指针等于写指针时，说明队列已空 */
            break;
        }
        /* 执行到这里说明队列中还有数据 */
        /* 先确认写指针已经发布，再读取对应槽位。 */
        __DMB();
        slot = &s_adc_block_queue[read_index];
        block_sequence = slot->sequence;

        /* 序号不连续说明中间确实丢过块，连续确认必须重新开始。 */
        /*举例：当前处理到序号为100，读取到序号为102，说明中间丢了一块数据。*/
        if ((s_adc_last_processed_sequence != 0U) &&
            (block_sequence != (s_adc_last_processed_sequence + 1U)))
        {
            ADC_AutoRange_ClearConfirmCount();
        }

        /*
         * 队列积压期间如果前一块触发了切档，后续旧档数据不能按新档解释。
         * 这里直接丢掉旧档块；真正属于新档的第一块仍由SKIP逻辑丢弃。
         */
        if (slot->sample_range != Gate_get_status())
        {
            s_adc_last_processed_sequence = block_sequence;
            __DMB();
            s_adc_block_read_index = ADC_BlockQueue_NextIndex(read_index);
            continue;
        }

        block_valid =
            Process_ADC_Chunk(&slot->samples[0][0]);

        /*
        * 切档后的稳定等待块不上传。
        * 正常块和触发切档的旧档数据块都可以上传。
        */
        if (block_valid)
        {
            ADC_BuildUsbPacket(slot);
        }

        s_adc_last_processed_sequence = block_sequence;

        /* 槽位使用完成后再移动读指针，允许中断后续复用这个槽位。 */
        __DMB();
        s_adc_block_read_index = ADC_BlockQueue_NextIndex(read_index);
        processed_count++;
    }

    return processed_count;
}














/**
 * @brief ADC DMA前半区填满回调(套用的是ADC半传输回调函数)
 * @param hadc ADC句柄
 * @note  中断中只复制完整数据块并通知任务，不执行换挡、显示或打印。
 */
 void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }
    ADC_BlockQueue_PublishFromISR(&adc_raw_buffer[0][0]);


    /*
     * 中断中只通知任务，不在这里执行换挡和LCD刷新。
     *
     * 本工程DMA2 Stream0中断优先级为5，
     * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY也是5，
     * 因此允许调用CMSIS-RTOS的ISR通知接口。
     */
    if (MessageSendTaskHandle != NULL)
    {
        (void)osThreadFlagsSet(MessageSendTaskHandle,
                               ADC_FLAG_BLOCK_READY);
    }
     
}


/**
 * @brief ADC DMA后半区填满回调(套用的是ADC的全传输回调函数)
 * @param hadc ADC句柄
 * @note  中断中只复制完整数据块并通知任务，不执行换挡、显示或打印。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }

    /* 10～19行已经写完，此时DMA已经回到前半区。 */
    ADC_BlockQueue_PublishFromISR(&adc_raw_buffer[ADC_TIMES][0]);

    /*和半完成回调一样*/
    if (MessageSendTaskHandle != NULL)
    {
        (void)osThreadFlagsSet(MessageSendTaskHandle,
                               ADC_FLAG_BLOCK_READY);
    }
}


/**
 * @brief ADC DMA错误回调
 * @param hadc ADC句柄
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }

    s_adc_error_count++;

    if (MessageSendTaskHandle != NULL)
    {
        (void)osThreadFlagsSet(MessageSendTaskHandle,
                               ADC_FLAG_ERROR);
    }
}
