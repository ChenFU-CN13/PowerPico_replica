/**
  ******************************************************************************
  * @file    user_MessageTask.c
  * @brief   数据收发调度任务（系统数据流中枢）
  *
  * 【文件作用】
  *   定义两个 FreeRTOS 任务，负责将采样数据上报 USB / UI、
  *   以及对上位机命令进行接收与执行：
  *     1. MessageReceiveTask —— 上位机 -> 板子 的命令接收/执行任务
  *     2. MessageSendTask   —— 板子 -> 上位机 的采样数据上报任务
  *
  * 【数据流框架】
  *
  *   采集侧（MessageSendTask 的数据来源）：
  *     TIM2 触发 ADC1  ->  DMA 双缓冲 adc_raw_buffer（前/后两组）
  *        |  [半满/全满] DMA中断登记数据块并置位事件标志
  *        v
  *     ADC_FLAG_BLOCK_READY
  *        |  ADC_ProcessPendingBlocks()处理待处理数据块
  *        |---> 自动量程算法更新电流结果
  *        |---> 周期性写入PowerDataQueue -> LvHandlerTask刷新LCD
  *
  *   命令侧（MessageReceiveTask）：
  *     USB CDC 中断收到字节 -> CmdRxQueue
  *        -> CmdStrategy_ProcessRx() 解析执行：
  *           切量程(CMD_RANGE_SET)/校准(CMD_CAL_SET)/查询/升级(CMD_FW_UPDATE) 等
  *        回应帧放入 CmdTxQueue -> MessageSendTask 负责代发回上位机
  *
  *   发送仲裁（MsgSendTask 内，单USB通道排队发送）：
  *     命令帧优先于 ADC 数据帧，靠 pending_command / pending_adc /
  *     active_tx 三缓冲 + tx_active 状态避免两路数据互相覆盖。
  *
  * 【依赖关系】
  *   - user_AdcDataStrategy.h : ADC_ProcessPendingBlocks / ADC数据快照
  *   - user_CmdStrategy.h     : CmdStrategy_Init / CmdStrategy_ProcessRx
  *   - usbd_cdc_if.h          : CDC_Transmit_FS(USB发送)
  *   - user_TasksInit.h       : 消息队列与事件标志定义
  *
  * 【相关事件标志】(定义见 user_TasksInit.h)
  *   ADC_FLAG_BLOCK_READY ：有新的DMA半区或全区数据块待处理
  *   ADC_FLAG_ERROR       ：ADC发生错误
  
  注意以上是源码的注释，现在是用来临时测试用的！！，后续工程完成后，继续沿用以上相关注释
  ******************************************************************************
  */
#include "user_MessageTask.h"
#include "user_TasksInit.h"
#include "user_AdcDataStrategy.h"
#include "user_CmdStrategy.h"
#include "usbd_cdc_if.h"

#include <string.h>
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/*
 * 一个ADC数据块约对应1ms采样时间。
 * 每处理100块，约每100ms向显示任务发布一次数据。
 *
 * LCD每200ms刷新一次，PowerDataQueue深度为8，
 * 这个速度既能及时响应负载变化，也不会频繁刷新屏幕。
 */
#define STEP6D_UI_PUBLISH_BLOCKS    100U

/*
 * 10：先进行低速测试，每10个ADC块上传一包，约100包/秒。
 * 1 ：最终全速测试，每个ADC块上传一包，约1000包/秒。
 */
#define STEP7_WAVE_PACKET_DIV       1U

#if (STEP7_WAVE_PACKET_DIV == 0U)
#error "STEP7_WAVE_PACKET_DIV must be greater than zero"
#endif


/* Private variables ---------------------------------------------------------*/
/*
 * STEP6E USB接收链路计数。
 *
 * received/dropped在USB中断上下文更新，processed在接收任务更新。
 * Cortex-M4对对齐的32位变量可原子读写，LCD这里只做观察，不要求三个值
 * 必须来自完全相同的时刻，因此不额外关中断。
 */
static volatile uint32_t s_rx_received_chunks;
static volatile uint32_t s_rx_processed_chunks;
static volatile uint32_t s_rx_dropped_chunks;

/*
 * STEP7 USB发送链路计数，可直接加入Keil Watch窗口观察。
 * sent：已经交给USB驱动的波形包数量；
 * replace：来不及发送而被更新包替换的数量；
 * busy/fail：CDC_Transmit_FS返回忙或失败的次数。
 */
volatile uint32_t g_step7_wave_sent_count;
volatile uint32_t g_step7_wave_replace_count;
volatile uint32_t g_step7_usb_busy_count;
volatile uint32_t g_step7_usb_fail_count;



/* Tasks ---------------------------------------------------------------------*/

/**
 * @brief 通知MessageSendTask释放当前USB发送缓冲区
 * @note 正常发送完成或USB会话重置时都可以调用
 */
void MessageSend_NotifyTxCompleteFromISR(void)
{
    /*
     * USB回调运行在中断上下文。
     *这里只置位，不在中断中继续调用CDC_Transmit_FS()。
     */
    if (MessageSendTaskHandle != NULL)
    {
        (void)osThreadFlagsSet(MessageSendTaskHandle,FLAG_USB_TX_COMPLETE);
    }
}


/**
 * @brief 在USB接收回调中投递一块原始数据
 */
bool MessageReceive_EnqueueFromISR(const uint8_t *data,
                                   uint32_t length)
{
    CmdRxChunk_T chunk;

    /*
     * USB FS CDC正常一包最多64字节。
     * 先检查长度再转换成uint16_t，避免超长数据发生截断。
     */
    /* USB零长度包没有业务数据，不计为丢包。 */
    if (length == 0U)
    {
        return true;
    }

    if (data == NULL
        || length > CMD_RX_CHUNK_SIZE
        || CmdRxQueue == NULL)
    {
        s_rx_dropped_chunks++;
        return false;
    }

    chunk.length = (uint16_t)length;

    /*
     * 必须复制数据，不能只把USB缓冲区指针放进队列。
     * 回调退出并重新使能接收后，该缓冲区会被下一包数据覆盖。
     */
    memcpy(chunk.data, data, chunk.length);

    /*
     * timeout必须为0：USB回调运行在中断上下文，绝对不能阻塞等待。
     * 当前CMSIS-RTOS2适配层会自动调用xQueueSendToBackFromISR()。
     */
    //osMessageQueuePut第三个参数是优先级，如果priority=0，说明不设置优先级，正常FIFO排着队
    if (osMessageQueuePut(CmdRxQueue,
                          &chunk,
                          0U,
                          0U) != osOK)
    {
        s_rx_dropped_chunks++;
        return false;
    }

    s_rx_received_chunks++;
    return true;
}

/**
 * @brief 读取USB接收链路调试计数
 */
void MessageReceive_GetDebug(MessageRxDebug_T *debug)
{
    if (debug == NULL)
    {
        return;
    }

    debug->received_chunks = s_rx_received_chunks;
    debug->processed_chunks = s_rx_processed_chunks;
    debug->dropped_chunks = s_rx_dropped_chunks;
}

/**
 * @brief 接收并解析上位机通过USB CDC发送的命令
 * @param argument 未使用
 *
 * 数据流：
 * USB回调 -> CmdRxQueue -> MessageReceiveTask -> CmdStrategy_ProcessRx()
 */
void MessageReceiveTask(void *argument)
{
    CmdRxChunk_T chunk;

    (void)argument;

    /* 命令解析器只由本任务初始化和喂入数据。 */
    CmdStrategy_Init();

    for (;;)
    {
        /*
         * 正常情况下队列在调度器启动前已经创建。
         * 这里保留防御性检查，避免创建失败后任务空转占满CPU。
         */
        if (CmdRxQueue == NULL)
        {
            osDelay(100U);
            continue;
        }

        /* 如果没有USB数据时阻塞，不占用CPU时间
            如果有USB数据则往下进入CmdStrategy_ProcessRx解析数据。
            解析完数据之后，累计解析了多少次 */
        if (osMessageQueueGet(CmdRxQueue,
                              &chunk,
                              NULL,
                              osWaitForever) != osOK)
        {
            osDelay(10U);
            continue;
        }

        /*
         * 命令拼包和解析已经离开USB中断上下文。
         * 当前升级命令这里只置位请求，EEPROM写入、USB卸载和复位
         * 仍由defaultTask中的CmdStrategy_Process()执行。
         */
        CmdStrategy_ProcessRx(chunk.data, chunk.length);
        s_rx_processed_chunks++;
    }
}

/**
 * @brief 把ADC统计后的平均测量结果发送给显示任务
 *
 * @note
 * 电压和电流都来自约250ms的统计窗口。
 * USB波形上传仍然保持原来的原始采样数据，不受本函数影响。
 */
static void Step6D_PublishPowerData(void)
{

    PowerData_T new_data = {0};
    PowerData_T discarded;

    /*
     * 队列创建失败时不继续访问。
     * LCD会通过BERR错误位显示队列创建失败。
     */
    if (PowerDataQueue == NULL)
    {
        return;
    }

    /*当刚刚上电时，未累计够第一组250ms数据时先不发布*/
    if (!Data_Monitor_GetValues(&new_data.voltage_v,
                                &new_data.current_ua,
                                &new_data.range))
    {
        return;
    }


    /*
     * 队列满时丢弃最旧值，只保留最新显示结果。
     */
    if (osMessageQueuePut(PowerDataQueue,
                          &new_data,
                          0U,
                          0U) == osErrorResource)
    {
        (void)osMessageQueueGet(PowerDataQueue,
                                &discarded,
                                NULL,
                                0U);

        (void)osMessageQueuePut(PowerDataQueue,
                                &new_data,
                                0U,
                                0U);
    }
}

/**
 * @brief 处理ADC数据、发布LCD数据并统一发送波形包和命令回复
 * @param argument FreeRTOS任务入口参数，本任务没有使用
 *
 * @note
 * 本任务是USB发送侧的唯一管理者：
 * 1. 处理ADC模块产生的数据块；
 * 2. 定期把最新测量结果发给LCD任务；
 * 3. 获取最新ADC波形包；
 * 4. 从CmdTxQueue取出命令回复；
 * 5. 在命令和波形之间进行发送仲裁；
 * 6. 等待USB发送完成后才能复用active_tx。
 */
void MessageSendTask(void *argument)
{
    uint32_t flags;                 //等待的事件标志位
    uint32_t wait_time;             //等待时间，单位ms
    uint32_t processed_count;       //本次处理的数据块数量
    uint32_t ui_block_count = 0U;   //累计处理的数据块数量

    uint32_t wave_div_count = 0U;      //波形包发送次数
    uint32_t last_wave_sequence = 0U;  //上一个波形包序号
    uint32_t latest_wave_sequence;     //最新波形包序号
    uint32_t new_wave_count;           //新波形包数量

    CmdTxFrame_T pending_command;     //待发送的命令回复
    USB_ADC_Packet_T pending_adc;     //待发送的ADC波形包

    /*
     * active_tx必须能够容纳最大的波形包。
     * 命令回复最大58字节，小于81字节。
     */
    uint8_t active_tx[sizeof(USB_ADC_Packet_T)];

    const uint8_t *next_data;       //待发送的数据指针
    uint16_t next_length;          //待发送的数据长度

    bool command_pending = false;  //是否有待发送的命令回复
    bool adc_pending = false;       //是否有待发送的ADC波形包
    bool usb_tx_active = false;     //USB发送是否正在处理
    bool next_is_command;          //下一个待发送数据是否为命令回复

    uint8_t usb_tx_result;          //USB发送结果

    (void)argument;

    for (;;)
    {
        /*1.
         * USB空闲且有待发数据时，每1ms重试一次。
         * USB发送中时，等待发送完成、ADC或命令事件。
         */
        wait_time =
            (!usb_tx_active                     //USB发送是否正在处理
             && (command_pending || adc_pending))//是否有待发送的命令回复或ADC波形包
            ? 1U
            : osWaitForever;


        /*2.
        这里等待的事件标志位是：
         * 1. ADC_FLAG_ALL_EVENTS：ADC模块产生的数据块
         * 2. FLAG_CMD_TX_READY：CmdTxQueue取出的命令回复
         * 3. FLAG_USB_TX_COMPLETE：USB发送完成
        */
        flags = osThreadFlagsWait(
            ADC_FLAG_ALL_EVENTS
                | FLAG_CMD_TX_READY
                | FLAG_USB_TX_COMPLETE,
            osFlagsWaitAny,
            wait_time);

        /*3.
         * 1ms等待超时不是故障。
         * 清成0后继续尝试发送pending数据。
         */
        if ((flags & osFlagsError) != 0U)
        {
            /*osFlagsError：等待超时不是故障*/
            flags = 0U;
        }

        if ((flags & FLAG_USB_TX_COMPLETE) != 0U)
        {
            /*FLAG_USB_TX_COMPLETE：USB发送完成*/
            usb_tx_active = false;
        }

        /*执行到这里说明程序等待超时不是故障，或者USB发送完成，那么准备处理数据块
        processed_count = 0 是为了清空上次处理的数据块数量，避免重复处理*/
        processed_count = 0U;//本次处理的数据块数量

        if ((flags & ADC_FLAG_ALL_EVENTS) != 0U)
        {
            /*ADC_FLAG_ALL_EVENTS：ADC模块产生的数据块*/
            processed_count =
                ADC_ProcessPendingBlocks();//处理ADC模块产生的数据块
        }

        /* 保留原来的LCD数据发布节拍。 */
        ui_block_count += processed_count;

        /*这里用大白话讲一下：
        1. ui_block_count 是累计处理的数据块数量，每次处理ADC模块产生的数据块后，会更新这个值。
        2. STEP6D_UI_PUBLISH_BLOCKS 是LCD数据发布节拍，每处理STEP6D_UI_PUBLISH_BLOCKS个数据块，就会发布一次LCD数据。
        3. 这里用取余运算 % 来实现循环发布，避免数据丢失。*/
        if (ui_block_count
            >= STEP6D_UI_PUBLISH_BLOCKS)
        {
            ui_block_count %=
                STEP6D_UI_PUBLISH_BLOCKS;

            Step6D_PublishPowerData();//发布最新测量结果给LCD任务
        }

        /*
         * 获取ADC模块最近生成的波形包序号。
         * 序号发生变化，说明至少产生了一包新数据。
         */
        if (processed_count != 0U
            && ADC_GetLatestUsbPacket(
                   NULL,
                   &latest_wave_sequence))
        {
            new_wave_count =
                latest_wave_sequence
                - last_wave_sequence;

            if (new_wave_count != 0U)
            {
                last_wave_sequence =
                    latest_wave_sequence;

                wave_div_count +=
                    new_wave_count;

                if (wave_div_count
                    >= STEP7_WAVE_PACKET_DIV)
                {
                    wave_div_count %=
                        STEP7_WAVE_PACKET_DIV;

                    /*
                     * 全速模式下，如果一次处理出了多包，
                     * 只能保留最新包，记录中间被覆盖的数量。
                     */
                    if ((STEP7_WAVE_PACKET_DIV == 1U)
                        && (new_wave_count > 1U))
                    {
                        g_step7_wave_replace_count +=
                            new_wave_count - 1U;
                    }

                    /*
                     * pending_adc还没有发出时又有新包，
                     * 直接使用最新包覆盖旧包。
                     * 实时波形优先保证“新”，旧数据不继续积压。
                     */
                    if (adc_pending)
                    {
                        g_step7_wave_replace_count++;
                    }

                    if (ADC_GetLatestUsbPacket(
                            &pending_adc,
                            NULL))
                    {
                        adc_pending = true;
                    }
                }
            }
        }

        /*
         * 命令缓存空闲时，从命令回复队列取出一帧。
         */
        if (!command_pending
            && CmdTxQueue != NULL
            && osMessageQueueGet(
                   CmdTxQueue,
                   &pending_command,
                   NULL,
                   0U) == osOK)
        {
            command_pending = true;
        }

        /*
         * 清理非法命令帧，避免一直卡住发送任务。
         */
        if (command_pending
            && (pending_command.length == 0U
                || pending_command.length
                       > sizeof(pending_command.data)))
        {
            command_pending = false;
        }

        /*
         * 上一帧还没有完成时，
         * active_tx绝对不能覆盖。
         */
        if (usb_tx_active)
        {
            continue;
        }

        next_data = NULL;
        next_length = 0U;
        next_is_command = false;

        /*
         * 命令回复优先。
         * 波形是连续数据，晚一包影响不大；
         * 命令需要尽快返回，避免上位机超时。
         */
        if (command_pending)
        {
            next_data = pending_command.data;
            next_length = pending_command.length;
            next_is_command = true;
        }
        else if (adc_pending)
        {
            next_data = (const uint8_t *)&pending_adc;
            next_length = sizeof(pending_adc);
        }

        if (next_data == NULL)
        {
            continue;
        }

        if (next_length > sizeof(active_tx))
        {
            if (next_is_command)
            {
                command_pending = false;
            }
            else
            {
                adc_pending = false;
            }

            continue;
        }

        /*
         * USB驱动发送期间会继续使用这个地址，
         * 所以必须复制到独立的active_tx。
         */
        memcpy(active_tx,
               next_data,
               next_length);

        /*
         * 必须在CDC_Transmit_FS之前置true，
         * 防止发送完成回调来得很快造成状态覆盖。
         */
        usb_tx_active = true;

        usb_tx_result =
            CDC_Transmit_FS(active_tx,
                            next_length);

        if (usb_tx_result == USBD_OK)
        {
            if (next_is_command)
            {
                command_pending = false;
            }
            else
            {
                adc_pending = false;
                g_step7_wave_sent_count++;
            }
        }
        else
        {
            /*
             * 本轮没有启动发送。
             * 保留pending数据，1ms后继续重试。
             */
            usb_tx_active = false;

            if (usb_tx_result == USBD_BUSY)
            {
                g_step7_usb_busy_count++;
            }
            else
            {
                g_step7_usb_fail_count++;
            }
        }
    }
}


