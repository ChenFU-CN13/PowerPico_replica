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
  *     active_tx 三缓冲 + usb_tx_active 状态避免两路数据互相覆盖。
  *
  * 【依赖关系】
  *   - user_AdcDataStrategy.h : ADC_ProcessPendingBlocks / 最新ADC波形包
  *   - user_CmdStrategy.h     : CmdStrategy_Init / CmdStrategy_ProcessRx
  *   - usbd_cdc_if.h          : CDC_Transmit_FS(USB发送)
  *   - user_TasksInit.h       : 消息队列与事件标志定义
  *
  * 【相关事件标志】(定义见 user_TasksInit.h)
  *   ADC_FLAG_BLOCK_READY ：有新的DMA半区或全区数据块待处理
  *   ADC_FLAG_ERROR       ：ADC发生错误
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
#define MESSAGE_UI_PUBLISH_BLOCKS    100U

/*
 * 10：先进行低速测试，每10个ADC块上传一包，约100包/秒。
 * 1 ：最终全速测试，每个ADC块上传一包，约1000包/秒。
 */
#define MESSAGE_WAVE_PACKET_DIV       1U

#if (MESSAGE_WAVE_PACKET_DIV == 0U)
#error "MESSAGE_WAVE_PACKET_DIV must be greater than zero"
#endif

/**
 * @brief MessageSendTask循环中需要长期保存的状态
 *
 * pending_command和pending_adc保存两类待发送数据。
 * active_tx是USB驱动真正使用的独立缓冲区，发送完成前不能覆盖。
 */
typedef struct
{
    uint32_t ui_block_count;
    uint32_t wave_div_count;
    uint32_t last_wave_sequence;

    CmdTxFrame_T pending_command;
    USB_ADC_Packet_T pending_adc;
    uint8_t active_tx[sizeof(USB_ADC_Packet_T)];

    bool command_pending;
    bool adc_pending;
    bool usb_tx_active;
} MessageSendContext_T;


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
        return false;
    }

    return true;
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
    }
}

/**
 * @brief 把ADC统计后的平均测量结果发送给显示任务
 *
 * @note
 * 电压和电流都来自约250ms的统计窗口。
 * USB波形上传仍然保持原来的原始采样数据，不受本函数影响。
 */
static void MessageSend_PublishPowerData(void)
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
 * @brief 处理ADC事件，并更新LCD发布节拍和待发送波形
 * @param context MessageSendTask私有状态
 * @param flags 本轮收到的任务事件标志
 */
static void MessageSend_HandleAdcEvents(MessageSendContext_T *context,
                                        uint32_t flags)
{
    uint32_t processed_count = 0U;
    uint32_t latest_wave_sequence;
    uint32_t new_wave_count;

    if ((flags & ADC_FLAG_ALL_EVENTS) != 0U)
    {
        processed_count = ADC_ProcessPendingBlocks();
    }

    /* 保留原来的LCD数据发布节拍。 */
    context->ui_block_count += processed_count;

    if (context->ui_block_count >= MESSAGE_UI_PUBLISH_BLOCKS)
    {
        context->ui_block_count %= MESSAGE_UI_PUBLISH_BLOCKS;
        MessageSend_PublishPowerData();
    }

    if (processed_count == 0U
        || !ADC_GetLatestUsbPacket(NULL,
                                   &latest_wave_sequence))
    {
        return;
    }

    /* 序号差表示本轮至少产生了一包新数据。 */
    new_wave_count =
        latest_wave_sequence - context->last_wave_sequence;

    if (new_wave_count == 0U)
    {
        return;
    }

    context->last_wave_sequence = latest_wave_sequence;
    context->wave_div_count += new_wave_count;

    if (context->wave_div_count < MESSAGE_WAVE_PACKET_DIV)
    {
        return;
    }

    context->wave_div_count %= MESSAGE_WAVE_PACKET_DIV;

    /*
     * pending_adc还没有发出时又有新包，直接用最新包覆盖旧包。
     * 波形属于实时数据，优先保证“新”，不积压历史波形。
     */
    if (ADC_GetLatestUsbPacket(&context->pending_adc, NULL))
    {
        context->adc_pending = true;
    }
}

/**
 * @brief 在命令缓存空闲时，从回复队列取出一帧命令
 * @param context MessageSendTask私有状态
 */
static void MessageSend_LoadPendingCommand(MessageSendContext_T *context)
{
    if (context->command_pending || CmdTxQueue == NULL)
    {
        return;
    }

    if (osMessageQueueGet(CmdTxQueue,
                          &context->pending_command,
                          NULL,
                          0U) != osOK)
    {
        return;
    }

    /* 非法帧直接丢弃，避免发送任务被一帧坏数据长期卡住。 */
    if (context->pending_command.length == 0U
        || context->pending_command.length
               > sizeof(context->pending_command.data))
    {
        return;
    }

    context->command_pending = true;
}

/**
 * @brief USB空闲时尝试发送一帧命令或波形
 * @param context MessageSendTask私有状态
 *
 * 命令回复优先于波形。CDC_Transmit_FS()返回非USBD_OK时，
 * 本轮没有取得缓冲区所有权，pending数据保留到下一轮重试。
 */
static void MessageSend_TryTransmit(MessageSendContext_T *context)
{
    const uint8_t *next_data = NULL;
    uint16_t next_length = 0U;
    bool next_is_command = false;
    uint8_t usb_tx_result;

    if (context->usb_tx_active)
    {
        return;
    }

    if (context->command_pending)
    {
        next_data = context->pending_command.data;
        next_length = context->pending_command.length;
        next_is_command = true;
    }
    else if (context->adc_pending)
    {
        next_data = (const uint8_t *)&context->pending_adc;
        next_length = sizeof(context->pending_adc);
    }

    if (next_data == NULL)
    {
        return;
    }

    if (next_length > sizeof(context->active_tx))
    {
        if (next_is_command)
        {
            context->command_pending = false;
        }
        else
        {
            context->adc_pending = false;
        }

        return;
    }

    /* USB驱动发送期间会继续使用该地址，必须复制到独立缓冲区。 */
    memcpy(context->active_tx, next_data, next_length);

    /* 先置忙，再调用驱动，避免极快的完成回调覆盖状态。 */
    context->usb_tx_active = true;
    usb_tx_result =
        CDC_Transmit_FS(context->active_tx, next_length);

    if (usb_tx_result == USBD_OK)
    {
        if (next_is_command)
        {
            context->command_pending = false;
        }
        else
        {
            context->adc_pending = false;
        }
    }
    else
    {
        /* USBD_BUSY或发送失败时保留pending数据，1ms后重试。 */
        context->usb_tx_active = false;
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
    uint32_t flags;
    uint32_t wait_time;
    MessageSendContext_T context = {0};

    (void)argument;

    for (;;)
    {
        /*
         * USB空闲且有待发数据时，每1ms重试一次。
         * USB发送中时，等待发送完成、ADC或命令事件。
         */
        wait_time =
            (!context.usb_tx_active
             && (context.command_pending
                 || context.adc_pending))
            ? 1U
            : osWaitForever;

        /* 同时等待ADC数据、命令回复和USB发送完成事件。 */
        flags = osThreadFlagsWait(
            ADC_FLAG_ALL_EVENTS
                | FLAG_CMD_TX_READY
                | FLAG_USB_TX_COMPLETE,
            osFlagsWaitAny,
            wait_time);

        /* 1ms等待超时不是故障，继续重试pending数据。 */
        if ((flags & osFlagsError) != 0U)
        {
            flags = 0U;
        }

        if ((flags & FLAG_USB_TX_COMPLETE) != 0U)
        {
            context.usb_tx_active = false;
        }

        MessageSend_HandleAdcEvents(&context, flags);
        MessageSend_LoadPendingCommand(&context);
        MessageSend_TryTransmit(&context);
    }
}
