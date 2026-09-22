


#include "user_AdcDataStrategy.h"
#include "user_TasksInit.h"
#include "BL24C02.h"
#include "gate.h"





/* DMA循环采样缓冲区，共100个uint16_t，占用200字节。 */
uint16_t adc_raw_buffer[ADC_DMA_ROWS][ADC_CHANNELS];


/*
 * 保存ADC任务最近处理完成的数据块。
 * LCD或其他低优先级代码只读取这个稳定快照。
 */
static uint16_t s_adc_snapshot[ADC_TIMES][ADC_CHANNELS];
/*
 * 偶数表示快照稳定，奇数表示ADC任务正在复制。
 * 读取方前后检查序号，避免读到一半新、一半旧的数据。
 */
static volatile uint32_t s_adc_snapshot_seq;
static volatile bool s_adc_snapshot_valid;

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
    uint32_t sequence;
    uint8_t sample_range;
    uint8_t reserved;
    uint16_t samples[ADC_TIMES][ADC_CHANNELS];
} ADC_BlockSlot_T;

static ADC_BlockSlot_T s_adc_block_queue[ADC_BLOCK_QUEUE_DEPTH];
static volatile uint8_t s_adc_block_write_index;
static volatile uint8_t s_adc_block_read_index;
static volatile uint32_t s_adc_block_sequence;
static volatile uint32_t s_adc_block_drop_count;
static uint32_t s_adc_block_drop_seen;
static uint32_t s_adc_error_seen;
static uint32_t s_adc_last_processed_sequence;

/*ADC采集启动标志*/
static bool s_adc_sampling_started;

/* 连续过载和连续欠载确认次数。 */
static uint8_t s_auto_up_count;
static uint8_t s_auto_down_count;

/* 切档后的数据丢弃和冷却计数。 */
static uint8_t s_auto_skip_chunks;
static uint8_t s_auto_cooldown_chunks;

/* 提供给STEP5 LCD页面观察的数据。 */
static ADC_AutoRangeDebug_T s_auto_debug;

/* 奇数表示ADC任务正在更新调试数据，偶数表示数据稳定。 */
static volatile uint32_t s_auto_debug_seq;


/* 只用于STEP3调试观察。 */
volatile uint32_t adc_half_count;
volatile uint32_t adc_full_count;
volatile uint32_t adc_error_count;


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
 * @brief 进入调试结构体写入阶段
 * @note 序号变成奇数后，LCD任务会等待本次更新完成
 */
static void ADC_AutoRange_DebugWriteBegin(void)
{
    s_auto_debug_seq++;
    __DMB();
}

/**
 * @brief 结束调试结构体写入阶段
 * @note 先保证数据写完，再把序号恢复成偶数
 */
static void ADC_AutoRange_DebugWriteEnd(void)
{
    __DMB();
    s_auto_debug_seq++;
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
 * @brief 同步内部计数到调试结构体
 */
static void ADC_AutoRange_UpdateDebugCount(void)
{
    s_auto_debug.up_count = s_auto_up_count;
    s_auto_debug.down_count = s_auto_down_count;
    s_auto_debug.skip_chunks = s_auto_skip_chunks;
    s_auto_debug.cooldown_chunks = s_auto_cooldown_chunks;
}


/**
 * @brief 复位自动量程算法
 */
void ADC_AutoRange_Reset(void)
{
    ADC_AutoRange_DebugWriteBegin();

    memset(&s_auto_debug,0,sizeof(s_auto_debug));

    ADC_AutoRange_ClearConfirmCount();
    /*初始化计数值*/
    s_auto_skip_chunks = 0;
    s_auto_cooldown_chunks = 0;
    /*初始化状态*/
    s_auto_debug.state = ADC_RANGE_STATE_HOLD;
    s_auto_debug.current_range = Gate_get_status();
    s_auto_debug.sample_range = Gate_get_status();
    s_auto_debug.request_range = Gate_get_status();
    /*由于将自动测量复位，所以全部状态回归初始*/
    s_auto_debug.last_from = Gate_get_status();
    s_auto_debug.last_to = Gate_get_status();
    s_auto_debug.last_reason = ADC_SWITCH_REASON_NONE;

    ADC_AutoRange_UpdateDebugCount();
    ADC_AutoRange_DebugWriteEnd();
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


/********************************自动切换量程核心代码****************************************/
/**
 * @brief 处理一个DMA半区，并执行自动量程判断
 * @param chunk_ptr DMA半区首地址，包含10组五通道ADC数据
 * @return true 本块已经处理
 * @return false 本块为空或处于切档后的丢弃阶段
 */
bool Process_ADC_Chunk(const uint16_t* chunk_ptr)
{
    uint8_t gate_mode;
    uint8_t current_range;
    uint8_t request_range;
    uint8_t i;
    bool decision_enabled;

    uint16_t current_adc_raw;
    uint16_t ref_adc_raw;
    uint16_t abs_delta_adc_raw;
    bool mid_near_rail;
    int32_t delta_adc_raw;

    /*1.先保持默认档位，保持默认事件和判空*/
    ADC_SwitchReason_T  switch_reason = ADC_SWITCH_REASON_NONE;

    if (chunk_ptr == NULL)
    {
        return false;
    }

    ADC_AutoRange_DebugWriteBegin();

    /*保持当前档位*/
    gate_mode = Gate_Get_Mode();
    current_range = Gate_get_status();
    request_range = current_range;

    s_auto_debug.current_range = current_range;
    s_auto_debug.request_range = request_range;

    /*
     * 刚切档后的下一整块数据不可信。
     *
     * 原因：
     * MOS开关、采样电阻流路和INA190输出都需要一点稳定时间。
     * 所以这里整块丢弃，不参与换挡判断，也不参与正式数据显示。
     */
    if (s_auto_skip_chunks > 0)
    {
        s_auto_skip_chunks--;

        s_auto_debug.state = ADC_RANGE_STATE_SKIP;
        s_auto_debug.skipped_block_count++;

        ADC_AutoRange_UpdateDebugCount();
        ADC_AutoRange_DebugWriteEnd();
        return false;
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

    if (decision_enabled)
    {
        s_auto_debug.state = ADC_RANGE_STATE_HOLD;//保持
    }
    else if (s_auto_cooldown_chunks > 0)
    {
        s_auto_debug.state = ADC_RANGE_STATE_COOL;//冷却
    }
    else
    {
        s_auto_debug.state = ADC_RANGE_STATE_HOLD;//保持
    }

    /*
     * 一个DMA半区有10组采样。
     * 连续确认使用的是“连续采样点”，不是LCD刷新次数。
     */
    for ( i = 0; i < ADC_TIMES; i++)
    {
        const uint16_t *sample_row;

        sample_row = chunk_ptr + ((uint32_t)i*ADC_CHANNELS);    //这里是移动下一组5通道ADC数据的意思
        /*比如：i = 0，chunk_ptr + 0 × 5= chunk_ptr + 0
        当i=1 ，= chunk_ptr + 5
        V0 L0 M0 H0 R0 V1 L1 M1 H1 R1 V2 L2 M2 H2 R2 ...
        ↑              ↑              ↑
        0              5              10*/


        current_adc_raw = ADC_SelectCurrentRaw(sample_row,current_range);
        ref_adc_raw = sample_row[IDX_REF];

        /*
         * 先转成有符号32位整数再相减。
         * 这样2043-2048会得到-5，不会发生无符号下溢。
         */
        delta_adc_raw =
            (int32_t)current_adc_raw -
            (int32_t)ref_adc_raw;

        abs_delta_adc_raw =
            ADC_AbsDiffU16(current_adc_raw, ref_adc_raw);
        /* 保存最近处理的一个采样点，供LCD观察。 */
        s_auto_debug.valid = true;
        s_auto_debug.sample_range = current_range;
        s_auto_debug.current_adc_raw = current_adc_raw;
        s_auto_debug.delta_adc_raw = delta_adc_raw;
        s_auto_debug.ref_adc_raw = ref_adc_raw;
        s_auto_debug.abs_delta_adc_raw = abs_delta_adc_raw;

        if (!decision_enabled)
        {
            /*
             * 手动模式或冷却期间只更新数据，
             * 不执行下面的自动换挡判断。
             */
            continue;
        }




    switch (current_range)
    {
             case LOW_CUR:
            {
                /*
                 * LOW已经是最小电流档，只能升到MID。
                 * 所以这里清除无意义的降档计数。
                 */
                s_auto_down_count = 0U;

                if (abs_delta_adc_raw >= AUTO_RANGE_LOW_UP_CODE)
                {   /*如果大于阈值，则判断已经是否满足4次升档条件计数*/
                    if (s_auto_up_count < AUTO_RANGE_UP_CONFIRM_TIMES)
                    {   
                        s_auto_up_count++;
                    }

                    s_auto_debug.state = ADC_RANGE_STATE_UP_WAIT;//同时标记为待升档状态

                    if (s_auto_up_count >= AUTO_RANGE_UP_CONFIRM_TIMES)
                    {   /*如果计数条件已经满足，则准备升档*/
                        request_range = MID_CUR;                //标记期望升档的档位  
                        switch_reason = ADC_SWITCH_REASON_OVERLOAD;//标记升档原因
                    }
                }
                else
                {
                    /*
                     * 只要中途有一个采样点不满足条件，
                     * 连续确认链条就要清零，重新开始计数。
                     */
                    s_auto_up_count = 0U;
                    s_auto_debug.state = ADC_RANGE_STATE_HOLD;//标记为保持状态
                }

                break;
            }

            case MID_CUR:
            {
                /*
                 * MID档既能升到HIGH，也能降到LOW。
                 *
                 * 如果MID通道已经接近0或4095，即使DELTA判断异常，
                 * 也优先按过载处理，避免ADC饱和后继续停在MID档。
                 */
                mid_near_rail =
                    (current_adc_raw <= AUTO_RANGE_MID_RAIL_MARGIN)
                    || (current_adc_raw >=
                        (4095U - AUTO_RANGE_MID_RAIL_MARGIN));

                /*
                 * 升档和降档必须互斥。
                 * 优先处理过载，防止极端情况下同时增加两个计数器。(也就是先处理升档)
                 */
                if ((abs_delta_adc_raw >= AUTO_RANGE_MID_UP_CODE)
                    || mid_near_rail)
                {
                    /*如果变化量大于阈值*/
                    s_auto_down_count=0;    //避免两个计算器同时增加，先停止降档计

                    if (s_auto_up_count < AUTO_RANGE_UP_CONFIRM_TIMES)
                    {
                         s_auto_up_count++; //小于条件则继续++
                    }
                    s_auto_debug.state = ADC_RANGE_STATE_UP_WAIT;//并同时标记状态为待升档
                    
                    if (s_auto_up_count>=AUTO_RANGE_UP_CONFIRM_TIMES)
                    {
                        /*如果满足升档条件*/
                        request_range = HIGH_CUR;//标记期望升档档位

                        if (mid_near_rail)
                        {
                            switch_reason = ADC_SWITCH_REASON_NEAR_RAIL;//标记切档原因为逼近ADC最大值
                        }
                        else
                        {
                            switch_reason = ADC_SWITCH_REASON_OVERLOAD;//标记原因为过负载
                        }
                        
                    }
                    
                }
                /*后处理降档逻辑*/
                else if (abs_delta_adc_raw <= AUTO_RANGE_MID_DOWN_CODE)
                {
                    /*如果变化量小于阈值*/
                    /*为避免两个计数器同时计数，先停止升档计数*/
                    s_auto_up_count = 0;

                    if (s_auto_down_count < AUTO_RANGE_DOWN_CONFIRM_TIMES)
                    {   /*如果降档条件＜10*/
                        s_auto_down_count++;//继续++
                    }
                    
                    s_auto_debug.state = ADC_RANGE_STATE_DOWN_WAIT;//标记为待降档状态

                    if (s_auto_down_count >= AUTO_RANGE_DOWN_CONFIRM_TIMES)
                    {
                        /* 如果降档计数达到，那么标记期望切档模式 */
                        request_range = LOW_CUR;
                        switch_reason = ADC_SWITCH_REASON_UNDERLOAD;//标记切档原因为负载电流减小，切换高精度单位测量
                    }
                    



                }
                else
                {
                    /*
                     * 当前幅度位于上下阈值之间。
                     * 那么就停在保持区，不升档也不降档。
                     */
                    ADC_AutoRange_ClearConfirmCount();
                    s_auto_debug.state = ADC_RANGE_STATE_HOLD;
                }






                break;
            }


            case HIGH_CUR:
            {
                /*
                 * HIGH已经是最大电流档，只能降到MID。
                 */
                s_auto_up_count = 0U;//所以升档的计数必须停

                if (abs_delta_adc_raw <= AUTO_RANGE_HIGH_DOWN_CODE)
                {
                    /*如果降档计数器＜10，则继续++*/
                    if (s_auto_down_count <
                        AUTO_RANGE_DOWN_CONFIRM_TIMES)
                    {
                        s_auto_down_count++;
                    }

                    s_auto_debug.state =ADC_RANGE_STATE_DOWN_WAIT;//并标记为待降档状态

                    /*如果降档次数达标，则标记期望切档档位，并标记切档原因是负载电流减小，切换高精度单位测量*/
                    if (s_auto_down_count >=AUTO_RANGE_DOWN_CONFIRM_TIMES)
                    {
                        request_range = MID_CUR;
                        switch_reason = ADC_SWITCH_REASON_UNDERLOAD;
                    }
                }
                else
                {
                    /*如果中途出现电流ADC值小于降档阈值，则取消连续计数并归0，状态标记为保持*/
                    s_auto_down_count = 0U;
                    s_auto_debug.state = ADC_RANGE_STATE_HOLD;
                }
                

                break;
            }

            default:
            {
                /*
                 * 软件档位记录异常时，回到HIGH档。
                 * 未知负载优先使用最大量程，避免大电流经过50Ω采样电阻。
                 */
                request_range = HIGH_CUR;
                switch_reason = ADC_SWITCH_REASON_RANGE_ERROR;
                break;
            }
    }   /* switch (current_range) 换挡判断结束 */

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
        uint8_t confirm_count;

        if ((switch_reason == ADC_SWITCH_REASON_OVERLOAD) || (switch_reason == ADC_SWITCH_REASON_NEAR_RAIL))
        {
            /* 如果切档原因为负载过大 或者 采集ADC值逼近ADC最大值 */
            confirm_count = s_auto_up_count;    //则提取升档计数值
        }
        else
        {
            /* 否则就提取降档计数值 */
            confirm_count = s_auto_down_count;
        }
        
        flow_route_selection(request_range);    //真正切换电路测量档位，改GPIO设置

        /*以下则记录今次换挡原因和换挡前的各项设置，用于调试对比*/
        s_auto_debug.last_from = current_range;
        s_auto_debug.last_to = request_range;
        s_auto_debug.last_reason = switch_reason;
        s_auto_debug.last_confirm_count = confirm_count;    //这里应该是换挡次数
        s_auto_debug.switch_count++;                        //累计换挡次数？

        /*
         * 下一块直接丢弃。因为下快数据是旧档位测量得到的
         * 丢弃结束后，再经过5个有效数据块的冷却期。
         */
        s_auto_skip_chunks = AUTO_RANGE_SKIP_CHUNKS;//标记丢弃下一个数据
        s_auto_cooldown_chunks = AUTO_RANGE_COOLDOWN_CHUNKS;//标记冷却5个数据期
        ADC_AutoRange_ClearConfirmCount();  //清空两个计数器
        /*提取给LCD*/
        s_auto_debug.state = ADC_RANGE_STATE_SWITCH;
        s_auto_debug.current_range = request_range;
        s_auto_debug.request_range = request_range;
        
    }
        else if (s_auto_cooldown_chunks > 0U)
    {
        /*
         * 冷却期的数据仍然可以用于显示。
         * 每正常处理完一块，冷却计数减1。
         */
        s_auto_cooldown_chunks--;
        s_auto_debug.cooldown_block_count++;
        s_auto_debug.state = ADC_RANGE_STATE_COOL;
    }

    ADC_AutoRange_UpdateDebugCount();
    ADC_AutoRange_DebugWriteEnd();

    return true;






}

/**
 * @brief 获取自动量程调试数据
 */
bool ADC_AutoRange_GetDebug(ADC_AutoRangeDebug_T *debug)
{
    uint32_t seq_begin;
    uint32_t seq_end;

    if (debug == NULL)
    {
        return false;
    }

    /*
     * ADC任务可能在LCD读取过程中更新结构体。
     * 前后序号一致且为偶数，才说明这份副本来自同一次完整更新。
     */
    for (;;)
    {
        seq_begin = s_auto_debug_seq;

        if ((seq_begin & 1U) != 0U)
        {
            continue;
        }

        __DMB();
        *debug = s_auto_debug;
        __DMB();

        seq_end = s_auto_debug_seq;

        if ((seq_begin == seq_end) && ((seq_end & 1U) == 0U))
        {
            break;
        }
    }

    return debug->valid;
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
    adc_half_count=0;
    adc_full_count=0;
    adc_error_count=0;

    memset(s_adc_snapshot, 0, sizeof(s_adc_snapshot));
    s_adc_snapshot_seq = 0U;
    s_adc_snapshot_valid = false;

    memset(s_adc_block_queue, 0, sizeof(s_adc_block_queue));
    s_adc_block_write_index = 0U;
    s_adc_block_read_index = 0U;
    s_adc_block_sequence = 0U;
    s_adc_block_drop_count = 0U;
    s_adc_block_drop_seen = 0U;
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
 * @brief 获取最近一个完整DMA半区的五通道平均值
 * @param average 用于保存平均结果
 * @return true 获取成功
 * @return false 参数错误或还没有有效快照
 */
bool ADC_GetRawAverage(ADC_RawAverage_T *average)
{
    uint16_t local_snapshot[ADC_TIMES][ADC_CHANNELS];       //局部副本，避免ADC任务在读取中途更新全局快照
    uint32_t sum[ADC_CHANNELS] = {0U};                      //分别将5通道的ADC值进行累加，用一个数组存储起来
    uint32_t seq_begin;                                     //seq_begin，seq_end用来判断复制期间，中断有没有修改全局快照
    uint32_t seq_end;   
    uint32_t row;                                           //两个循环下标row，channel。
    uint32_t channel;

    /*1.先判断是否合法，避免空指针；保证在ISR后完成了更新*/
    if (average == NULL || !s_adc_snapshot_valid)
    {
        return false;
    }    
    /*
     * 这个do while循环的目的是，将备份提取出来，复制到这个函数的局部备份（也就是快照）
     * 每次先判断ADC任务是否正在更新，如果不是，才提取数据
     * 如果正在更新则直接跳过本轮（即continue）
     * 
     * 当提取完数据后，用seq_end再获取一次是否是处于ISR更新数据状态，
     * 再进入一次while的判断，目的是判断当前数据是否是被ISR中途修改过的
     * 也就是避免后半段是新数据、前半段还是旧数据
     * 如果是这样的数据非法，那就是不可用的，直接再进入一轮while循环重新提取快照
     * 如果不是非法数据，那就停止do while循环，那就进入下一步分别累加各个通道ADC值
     */
    for (;;)
    {
        seq_begin = s_adc_snapshot_seq;

        /* 奇数表示ADC任务正在更新快照，稍后重新读取。 */
        if ((seq_begin & 1U) != 0U)
        {
            continue;
        }

        /*
        * DMB只保证内存访问顺序，不会禁止任务切换。
        * 是否发生并发修改，由前后两次序号负责判断。
        */
        __DMB();

        memcpy(local_snapshot,
            s_adc_snapshot,
            sizeof(local_snapshot));

        __DMB();

        seq_end = s_adc_snapshot_seq;

        /* 序号前后一致且为偶数，说明本地副本完整。 */
        if ((seq_begin == seq_end) &&
            ((seq_end & 1U) == 0U))
        {
            break;
        }
    }


    for ( row = 0; row < ADC_TIMES; row++)
    {
        for ( channel = 0; channel < ADC_CHANNELS; channel++)
        {
            sum[channel] += local_snapshot[row][channel];
        }
        
    }
    
    /*4.这里是求平均值：
    按正常来说平均值=SUM/10，但是由于C语言整除会丢小数(这里是整数，所以最后一位是小数)，所以ADC_TIMES/2=5
    即sum+5，用来保证四舍五入,列如：
    10次采样总和 = 20486
    不加5：
    20486 ÷ 10 = 2048

    加5：
    (20486 + 5) ÷ 10
    = 20491 ÷ 10
    = 2049
    精度不一样
    */
    average->vol_raw = (uint16_t)((sum[IDX_VOL] + ADC_TIMES/2) / ADC_TIMES);
    average->low_raw = (uint16_t)((sum[IDX_LOW] + ADC_TIMES/2) / ADC_TIMES);
    average->mid_raw = (uint16_t)((sum[IDX_MID] + ADC_TIMES/2) / ADC_TIMES);
    average->high_raw = (uint16_t)((sum[IDX_HIGH] + ADC_TIMES/2) / ADC_TIMES);
    average->ref_raw = (uint16_t)((sum[IDX_REF] + ADC_TIMES/2) / ADC_TIMES);

    return true;

}

/**
 * @brief 把电压通道ADC值换算成VBUS电压
 * @param vol_raw 电压通道平均原始值
 * @return VBUS电压，单位V
 */
float ADC_ConvertVoltage_V(uint16_t vol_raw)
{
    float adc_voltage_v;
    float vbus_voltage_v;

    /* ADC原始值换算成VADC引脚电压。 */
    adc_voltage_v = (float)vol_raw
                    * ADC_VREF_V
                    / ADC_MAX_CODE;

    /* 恢复被电阻网络缩小11倍之前的VBUS电压。 */
    vbus_voltage_v = adc_voltage_v
                     * VBUS_DIVIDER_GAIN;

    return vbus_voltage_v;
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
 * @brief 发布ADC任务已经处理到的最新数据块快照
 * @param source 队列中当前数据块的首地址
 * @note 这里只在ADC处理任务中调用，不直接读取DMA活动半区
 */
static void ADC_PublishSnapshot(const uint16_t* source)
{
    /*1.变成奇数，表示在更新数据*/
    s_adc_snapshot_seq++;
    __DMB();

    /*2.复制数据*/
    memcpy(s_adc_snapshot,source,sizeof(s_adc_snapshot));
    __DMB();

    /*3.奇数+1变成偶数，表示更新完成*/
    s_adc_snapshot_seq++;
    s_adc_snapshot_valid = true;
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
 * @note 队列满时丢弃最新块并累计计数，绝不覆盖任务尚未处理的数据
 */
static void ADC_BlockQueue_PublishFromISR(const uint16_t *source)
{
    uint8_t write_index;
    uint8_t next_index;
    ADC_BlockSlot_T *slot;

    write_index = s_adc_block_write_index;
    next_index = ADC_BlockQueue_NextIndex(write_index);

    /* 每次DMA半区完成都占用一个序号，丢块后任务能识别序号断点。 */
    s_adc_block_sequence++;

    if (next_index == s_adc_block_read_index)
    {
        s_adc_block_drop_count++;
        return;
    }

    slot = &s_adc_block_queue[write_index];
    slot->sequence = s_adc_block_sequence;
    slot->sample_range = Gate_get_status();
    memcpy(slot->samples, source, sizeof(slot->samples));

    /* 数据全部写完后才能发布新的写指针。 */
    __DMB();
    s_adc_block_write_index = next_index;
}

/**
 * @brief 处理ADC队列中已经完成的全部数据块
 * @return 本次处理的数据块数量
 */
uint32_t ADC_ProcessPendingBlocks(void)
{
    uint8_t read_index;
    uint32_t processed_count;
    uint32_t drop_count;
    uint32_t error_count;
    uint32_t block_sequence;
    uint32_t error_discard_count;
    uint8_t published_write_index;
    ADC_BlockSlot_T *slot;
    bool block_valid;

    processed_count = 0U;
    drop_count = s_adc_block_drop_count;
    error_count = adc_error_count;

    /* 队列满导致的丢块单独累计，方便LCD直接观察任务是否跟得上。 */
    if (drop_count != s_adc_block_drop_seen)
    {
        ADC_AutoRange_DebugWriteBegin();
        s_auto_debug.lost_block_count +=
            (uint32_t)(drop_count - s_adc_block_drop_seen);
        ADC_AutoRange_DebugWriteEnd();
        s_adc_block_drop_seen = drop_count;
    }

    /* ADC硬件报错后取消连续确认，避免把错误前后的数据拼成一次换挡。 */
    if (error_count != s_adc_error_seen)
    {
        ADC_AutoRange_ClearConfirmCount();

        /*
         * 错误发生前后数据的连续性已经无法保证。
         * 丢掉此刻已经排队的块，防止把错误两侧的采样拼成一次连续确认。
         */
        error_discard_count = 0U;
        published_write_index = s_adc_block_write_index;

        while (s_adc_block_read_index != published_write_index)
        {
            s_adc_block_read_index =
                ADC_BlockQueue_NextIndex(s_adc_block_read_index);
            error_discard_count++;
        }

        s_adc_last_processed_sequence = 0U;

        ADC_AutoRange_DebugWriteBegin();
        s_auto_debug.state = ADC_RANGE_STATE_HOLD;
        s_auto_debug.lost_block_count += error_discard_count;
        ADC_AutoRange_UpdateDebugCount();
        ADC_AutoRange_DebugWriteEnd();

        s_adc_error_seen = error_count;
    }

    for (;;)
    {
        read_index = s_adc_block_read_index;

        if (read_index == s_adc_block_write_index)
        {
            break;
        }

        /* 先确认写指针已经发布，再读取对应槽位。 */
        __DMB();
        slot = &s_adc_block_queue[read_index];
        block_sequence = slot->sequence;

        /* 序号不连续说明中间确实丢过块，连续确认必须重新开始。 */
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
            ADC_AutoRange_DebugWriteBegin();
            s_auto_debug.lost_block_count++;
            ADC_AutoRange_DebugWriteEnd();

            s_adc_last_processed_sequence = block_sequence;
            __DMB();
            s_adc_block_read_index = ADC_BlockQueue_NextIndex(read_index);
            continue;
        }

        /* 快照和自动换挡都只在ADC任务中更新，不再读取DMA正在改写的原缓冲区。 */
        ADC_PublishSnapshot(&slot->samples[0][0]);

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
 * @note  当前只做计数，不能在中断中显示LCD、打印或执行复杂计算。
 */
 void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }
    adc_half_count++;
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
 * @note  当前只做计数，不能在中断中显示LCD、打印或执行复杂计算。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }

    adc_full_count++;
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

    adc_error_count++;

    if (MessageSendTaskHandle != NULL)
    {
        (void)osThreadFlagsSet(MessageSendTaskHandle,
                               ADC_FLAG_ERROR);
    }
}
