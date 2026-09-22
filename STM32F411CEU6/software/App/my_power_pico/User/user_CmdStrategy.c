/**
 * @file user_CmdStrategy.c
 * @brief 解析上位机字节流，执行命令并生成带 CRC 的回复帧。
 *
 * 请求帧格式：
 * [0..1]  0xA5 0x5A 帧头
 * [2]     协议版本
 * [3]     命令类型
 * [4..5]  序列号，小端
 * [6..7]  payload 长度，小端
 * [8..]   payload
 * [末2字节] CRC16-CCITT，小端；计算范围从 version 到 payload 末尾
 *
 * 调用链：USB 回调 -> CmdRxQueue -> MessageReceiveTask -> CmdStrategy_ProcessRx()
 *       -> CmdTxQueue -> MessageSendTask -> USB CDC。
 *
 * 这里按“字节流”处理，不能假设一次 USB 接收正好是一整帧；半帧、粘包和帧前噪声
 * 都由 cmd_state 缓冲区和 CmdStrategy_TryGetFrame() 处理。
 */
#include "user_CmdStrategy.h"

#include "BL24C02.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "gate.h"
#include "user_TasksInit.h"


#include <stdbool.h>
#include <stddef.h>
#include <string.h>


#define CMD_PROTOCOL_MAX_PAYLOAD    48U     // 单帧业务数据最大 48 字节。
#define CMD_PROTOCOL_FRAME_OVERHEAD 10U     // 帧头2 + 版本1 + 类型1 + 序号2 + 长度2 + CRC2
#define CMD_PROTOCOL_RX_BUFFER_SIZE 64U     // 缓冲区大小为64

#define CMD_RESPONSE_MASK    0x80U   // 回复命令号最高位置1
/**
 * @brief 命令执行结果
 */
typedef enum
{
    CMD_STATUS_OK = 0,                    // 命令执行成功
    CMD_STATUS_UNSUPPORTED_VERSION = 1,  // 协议版本不支持
    CMD_STATUS_UNSUPPORTED_COMMAND = 2,  // 命令号未实现
    CMD_STATUS_INVALID_LENGTH = 3,       // payload长度错误
    CMD_STATUS_INVALID_PARAMETER = 4     // 参数内容错误
} CmdStatus_T;

/**
 * @brief 从接收字节流中解析出的完整命令
 */
typedef struct
{
    uint8_t version;                              // 协议版本
    uint8_t type;                                 // 命令类型
    uint16_t sequence;                            // 命令序号
    uint16_t payload_length;                      // 业务数据长度
    uint8_t payload[CMD_PROTOCOL_MAX_PAYLOAD];    // 业务数据
} CmdFrame_T;



typedef struct 
{
    uint8_t buffer[CMD_PROTOCOL_RX_BUFFER_SIZE];
    uint16_t length;
} CmdRxState_t;



static CmdRxState_t s_cmd_rx;
static volatile uint8_t s_update_pending = 0;


/**
 * @brief 从小端字节流读取 16 位无符号数
 */
static uint16_t CmdStrategy_ReadU16(const uint8_t*source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1]<<8);
}

/**
 * @brief 按小端顺序写入16位数据
 */
static void CmdStrategy_WriteU16(uint8_t *destination,
                                 uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)(value >> 8);
}

/**
 * @brief CRC16-CCITT，初值 0xFFFF，多项式 0x1021
 */
static uint16_t CmdStrategy_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    for ( index = 0; index < length; index++)
    {
        uint8_t bit;
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}


/**
 * @brief 从接收缓存前端删除已处理字节
 */
static void CmdStrategy_RemovePrefix(uint16_t count)
{
    if (count >= s_cmd_rx.length) {
        s_cmd_rx.length = 0U;
        return;
    }

    memmove(s_cmd_rx.buffer,
            &s_cmd_rx.buffer[count],
            s_cmd_rx.length - count);

    s_cmd_rx.length -= count;
}

/**
 * @brief 在缓存中寻找 0xA5 0x5A 帧头
 */
static uint16_t CmdStrategy_FindMagic(void)
{
    uint16_t index;

    for (index = 0U; index + 1U < s_cmd_rx.length; index++) {
        if (s_cmd_rx.buffer[index] == 0xA5U
            && s_cmd_rx.buffer[index + 1U] == 0x5AU) {
            return index;
        }
    }

    return s_cmd_rx.length;
}


/**
 * @brief 从USB接收缓存中提取一帧CRC正确的命令
 *
 * 本函数只负责协议拆帧，不执行具体命令。
 *
 * @param frame 保存解析完成的命令内容
 *
 * @return true
 *         已经成功提取一帧CRC正确的完整命令，
 *         frame中保存了解析结果。
 *
 * @return false
 *         当前没有完整有效帧。
 *         可能是数据不足、没有找到帧头，
 *         或清理错误数据后需要等待下一包USB数据。
 *
 * 函数执行顺序：
 * 调用 CmdStrategy_TryGetFrame(frame)
               │
               ▼
      frame 是否为 NULL？
       │               │
      是               否
       │               │
返回 false             ▼
                 进入 while(true)
                       │
                       ▼
              寻找帧头 A5 5A
                       │
          ┌────────────┴────────────┐
          │                         │
       没找到                     找到了
          │                         │
          ▼                         ▼
 最后一个字节是否为 A5？     帧头前是否有垃圾数据？
      │            │             │          │
     是            否            有          没有
      │            │             │          │
保留一个 A5      清空缓存       删除垃圾数据   保持不变
      │            │             └────┬─────┘
      └──────┬─────┘                  │
             ▼                        ▼
        返回 false              是否收到至少8字节？
                                │              │
                               否              是
                                │              │
                         返回 false            ▼
                                          读取payload长度
                                                 │
                                                 ▼
                                  payload长度是否超过上限？
                                      │                │
                                     是                否
                                      │                │
                                删除第一个字节          ▼
                                      │          计算完整帧长度
                                      │                │
                                      └─重新循环        ▼
                                            完整帧是否已经收齐？
                                                │           │
                                               否           是
                                                │           │
                                         返回 false         ▼
                                                        读取帧尾CRC
                                                             │
                                                             ▼
                                                        重新计算CRC
                                                             │
                                                             ▼
                                                      两个CRC相等吗？
                                                  │                  │
                                                 否                  是
                                                  │                  │
                                           删除第一个字节             ▼
                                                  │           提取帧内字段
                                                  │        ┌──────────────┐
                                                  │        │ version      │
                                                  │        │ type         │
                                                  │        │ sequence     │
                                                  │        │ payload_len  │
                                                  │        │ payload      │
                                                  │        └──────────────┘
                                                  │                  │
                                                  └─重新循环          ▼
                                                       删除这一整帧
                                                                  │
                                                                  ▼
                                                            返回 true
 */
static bool CmdStrategy_TryGetFrame(CmdFrame_T *frame)
{
    uint16_t magic_index;       // 0xA5 0x5A帧头在接收缓存中的位置
    uint16_t payload_length;    // 当前命令携带的业务数据长度
    uint16_t frame_length;      // 当前完整命令帧的总长度
    uint16_t expected_crc;      // 从接收帧末尾读取的CRC
    uint16_t actual_crc;        // 根据接收内容重新计算得到的CRC

    /*1.判断数据帧是否有问题*/
    if (frame == NULL)
    {
        return false;
    }

    while (true)
    {
        /*
         * 在当前接收缓存中寻找连续的0xA5、0x5A。
         *
         * 找到时：
         * 返回0xA5所在的数组下标。
         *
         * 找不到时：
         * 返回s_cmd_rx.length。
         */
        magic_index = CmdStrategy_FindMagic();
        if (magic_index == s_cmd_rx.length)
        {
            /*
             * 特殊情况：
             *
             * 当前缓存最后一个字节是0xA5，
             * 它可能是一个被USB拆开的帧头前半部分。
             *
             * 例如：
             *
             * 第一包：11 22 A5
             * 第二包：5A 01 10 ...
             *
             * 此时必须保留最后的0xA5，
             * 等待下一包开头的0x5A。
             */
            /* 若最后一个字节是 0xA5，保留它等待下一包的 0x5A */
            if (s_cmd_rx.length != 0U && s_cmd_rx.buffer[s_cmd_rx.length-1] == 0XA5U)
            {
                /*
                 * 把最后一个0xA5移动到buffer[0]，
                 * 清掉它前面的所有无效数据。
                 */
                s_cmd_rx.buffer[0] = 0xA5U;
                s_cmd_rx.length = 1;//当前缓存只保留1个字节
            }
            else
            {
                s_cmd_rx.length=0;  //如果没有找到完整帧头，那就清楚所有数据，当前的数据最后一个字节不是0xA5
            }
            

            return false;//当前无法组成完整帧头，等待下包数据
        }
        
        /*
         * 找到了0xA5 0x5A，但帧头前面还有其他数据。
         * 例如：
         * 11 22 33 A5 5A 01 ...
         * ↑无效数据    ↑真正帧头
         * 删除前面的无效字节，
         * 让0xA5移动到buffer[0]。
         */
        if (magic_index != 0)
        {
            /* code */
            CmdStrategy_RemovePrefix(magic_index);
        }

        /*
         * 当前帧的payload长度字段位于buffer[6]和buffer[7]。
         * 因此至少收到前8字节以后，
         * 才能够读取payload_length。
         *
         * 前8字节包括：
         * 帧头2字节
         * + 版本1字节
         * + 类型1字节
         * + 序列号2字节
         * + payload长度2字节
         */
        if (s_cmd_rx.length<8)
        {
            return false;       //如果当前只是半个帧头，或者半个固定字段，保留现有数据，等待下一包数据追加
        }

        /*
         * 从buffer[6]和buffer[7]读取payload长度。
         * 协议使用小端格式：
         * buffer[6]：低8位
         * buffer[7]：高8位
         */
        payload_length  = CmdStrategy_ReadU16(&s_cmd_rx.buffer[6]);


        /*
         * payload不能超过协议规定的最大长度。
         *
         * 如果超过上限，说明：
         *
         * 1. 当前0xA5 0x5A可能是假帧头；
         * 2. 接收数据可能发生损坏；
         * 3. 发送方使用了不兼容的协议。
         */
        if (payload_length > CMD_PROTOCOL_MAX_PAYLOAD)
        {
            /*
             * 这里只删除当前buffer[0]的0xA5，
             * 不直接清空整个缓存。
             *
             * 因为后续字节中可能还存在真正的0xA5 0x5A帧头。
             */
            CmdStrategy_RemovePrefix(1);
            continue;   //回到while循环开头，在剩余数据中继续寻找帧头
        }
        

        /*
         * 计算当前完整帧应该具有的总长度。
         * CMD_PROTOCOL_FRAME_OVERHEAD包含：
         * 帧头2
         * + 版本1
         * + 类型1
         * + 序列号2
         * + payload长度2
         * + CRC2
         * 合计10字节。
         * 因此：
         * 完整帧长度 = 10 + payload长度
         */
        frame_length = (uint16_t)(CMD_PROTOCOL_FRAME_OVERHEAD + payload_length);

        /*判断是否收到完整长度*/
        if (s_cmd_rx.length < frame_length)
        {
            /*如果当前缓存数据还没有达到完整长度，可能是一帧数据被USB拆成多包，假设完整长度是20，当前是15*/
            return false;//不删除当前数据，继续等待下包
        }
        

        /*
         * 完整帧已经接收完毕。
         * 最后两个字节是发送方附带的CRC16，
         * 按小端格式读取：
         * 倒数第2字节：CRC低8位
         * 倒数第1字节：CRC高8位
         */
        expected_crc = CmdStrategy_ReadU16(
            &s_cmd_rx.buffer[frame_length - 2U]);
        /*
         * 重新计算当前帧的CRC16。
         * CRC从buffer[2]开始计算，
         * 因为buffer[0]和buffer[1]是帧头，不参与CRC。
         *
         * 参与CRC的数据长度为：
         * version          1字节
         * type             1字节
         * sequence         2字节
         * payload_length   2字节
         * payload          payload_length字节
         * 固定部分合计6字节，
         * 所以总长度为6 + payload_length。
         */
        actual_crc = CmdStrategy_Crc16(&s_cmd_rx.buffer[2],
                                       (uint16_t)(6U + payload_length));

        /*
         * 接收帧携带的CRC与本地计算结果不同，
         * 说明当前帧内容发生损坏。
         */
        if (expected_crc != actual_crc)
        {
            /*
             * CRC错误帧：
             * 1. 不执行命令；
             * 2. 不生成回复；
             * 3. 不按frame_length直接删除整帧。
             * 这里只删除当前第一个0xA5，
             * 然后重新寻找后面的帧头。
             * 这样即使当前错误帧中间混入了下一帧，
             * 解析器仍有机会重新同步。
             */
            CmdStrategy_RemovePrefix(1);
            continue;
        }
        //执行到这里说明：帧头正确，* 1. 帧头正确；* 2. payload长度没有超过上限；* 3. 完整帧已经全部接收；* 4. CRC校验正确。
        //* 现在把原始字节转换成CmdFrame_T结构体。


        /*
         * buffer[2]保存协议版本。
         *
         *这里只提取版本，不判断当前固件是否支持。
         * 版本检查由后续CmdStrategy_Execute()负责。
         */
        frame->version = s_cmd_rx.buffer[2];

        /*
         * buffer[3]保存命令类型。
         *
         * 例如：
         *
         * CMD_CAL_GET
         * CMD_RANGE_GET
         * CMD_RANGE_SET
         * CMD_CAL_SET
         * CMD_CAL_RESET
         * CMD_FW_UPDATE
         */
        frame->type = s_cmd_rx.buffer[3];

        /*
         * buffer[4]和buffer[5]保存16位序列号。
         * 序列号用于让上位机确认：
         * 收到的回复对应哪一条请求。
         * 回复帧应该原样带回该序列号。
         */
        frame->sequence = CmdStrategy_ReadU16(&s_cmd_rx.buffer[4]);

        /* 保存本帧真实的payload长度。*/
        frame->payload_length = payload_length;

                /*
         * payload长度不为0时，
         * 把buffer[8]开始的业务数据复制到输出结构体。
         *
         * buffer[8]正好是payload的第一个字节。
         */
        if (frame->payload_length!=0)
        {
            memcpy(frame->payload,&s_cmd_rx.buffer[8],payload_length);
        }

       /*
         * 当前完整帧已经成功提取。
         *
         * 从接收缓存开头删除frame_length个字节。
         *
         * 如果后面还粘着第二帧，
         * 第二帧会被保留在缓存中。
         */
        CmdStrategy_RemovePrefix(frame_length);


        /*
         * 通知调用者：
         *
         * 已经成功取得一帧完整且CRC正确的命令，
         * frame中保存了解析结果。
         */
        return true;



    }




}


/**
 * @brief 编码一帧完整命令数据
 * @return 编码后的总长度，参数错误返回0
 */
static uint16_t CmdStrategy_EncodeFrame(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output)
{
    uint16_t crc;

    if (output == NULL
        || payload_length > CMD_PROTOCOL_MAX_PAYLOAD
        || (payload == NULL && payload_length != 0U))
    {
        return 0U;
    }

    output[0] = 0xA5U;
    output[1] = 0x5AU;
    output[2] = CMD_PROTOCOL_VERSION;
    output[3] = type;

    CmdStrategy_WriteU16(&output[4], sequence);
    CmdStrategy_WriteU16(&output[6], payload_length);

    if (payload_length != 0U)
    {
        memcpy(&output[8], payload, payload_length);
    }

    /*
     * CRC从version开始计算，不包括A5 5A帧头。
     */
    crc = CmdStrategy_Crc16(
        &output[2],
        (uint16_t)(6U + payload_length));

    CmdStrategy_WriteU16(
        &output[8U + payload_length],
        crc);

    return (uint16_t)(
        CMD_PROTOCOL_FRAME_OVERHEAD + payload_length);
}

/**
 * @brief 编码命令回复
 *
 * 回复payload第一个字节固定为状态码，
 * 后面才是命令自己的返回数据。
 */
static uint16_t CmdStrategy_EncodeResponse(
    const CmdFrame_T *request,
    CmdStatus_T status,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output)
{
    uint8_t response_payload[CMD_PROTOCOL_MAX_PAYLOAD];

    if (request == NULL
        || output == NULL
        || payload_length >= CMD_PROTOCOL_MAX_PAYLOAD
        || (payload == NULL && payload_length != 0U))
    {
        return 0U;
    }

    response_payload[0] = (uint8_t)status;

    if (payload_length != 0U)
    {
        memcpy(&response_payload[1],
               payload,
               payload_length);
    }

    return CmdStrategy_EncodeFrame(
        (uint8_t)(request->type | CMD_RESPONSE_MASK),
        request->sequence,
        response_payload,
        (uint16_t)(payload_length + 1U),
        output);
}

/**
 * @brief 执行命令并生成回复
 */
static void CmdStrategy_Execute(
    const CmdFrame_T *request,
    CmdTxFrame_T *response)
{
    CmdStatus_T status = CMD_STATUS_OK;
    uint8_t payload[2];
    uint16_t payload_length = 0U;

    response->length = 0U;

    if (request->version != CMD_PROTOCOL_VERSION)
    {
        status = CMD_STATUS_UNSUPPORTED_VERSION;
    }
    else
    {
        switch (request->type)
        {
            case CMD_RANGE_GET:
                if (request->payload_length != 0U)
                {
                    status = CMD_STATUS_INVALID_LENGTH;
                }
                else
                {
                    /*
                     * payload[0]：用户选择的AUTO/LOW/MID/HIGH模式
                     * payload[1]：当前硬件实际接通的量程
                     */
                    payload[0] = Gate_Get_Mode();
                    payload[1] = Gate_get_status();
                    payload_length = 2U;
                }
                break;

            case CMD_FW_UPDATE:
                if (request->payload_length != 2U)
                {
                    status = CMD_STATUS_INVALID_LENGTH;
                }
                else if (request->payload[0] != 0xC3U
                         || request->payload[1] != 0x3CU)
                {
                    status = CMD_STATUS_INVALID_PARAMETER;
                }
                else
                {
                    /*
                     * 保持现有升级架构：
                     * 这里只置位，由defaultTask中的
                     * CmdStrategy_Process()延后执行复位。
                     */
                    s_update_pending = 1U;
                    return;
                }
                break;

            default:
                status = CMD_STATUS_UNSUPPORTED_COMMAND;
                break;
        }
    }

    response->length =
        CmdStrategy_EncodeResponse(
            request,
            status,
            payload,
            payload_length,
            response->data);
}

/**
 * @brief 把回复送入统一USB发送队列
 */
static bool CmdStrategy_QueueResponse(
    const CmdTxFrame_T *response)
{
    if (response == NULL
        || response->length == 0U
        || response->length > CMD_TX_FRAME_SIZE
        || CmdTxQueue == NULL
        || MessageSendTaskHandle == NULL)
    {
        return false;
    }

    /*
     * 最多等待10个系统节拍。
     * 避免USB异常时接收任务永久堵死。
     */
    if (osMessageQueuePut(CmdTxQueue,
                          response,
                          0U,
                          10U) != osOK)
    {
        return false;
    }

    (void)osThreadFlagsSet(
        MessageSendTaskHandle,
        FLAG_CMD_TX_READY);

    return true;
}


/**
 * @brief 识别开发阶段的 BOOT\r\n 文本命令
 * @note 用串口助手即可触发，方便先验证完整升级闭环。
 */
static void CmdStrategy_CheckTextBootCommand(const uint8_t *data,
                                             uint16_t length)
{
    static const uint8_t boot_command[] = "BOOT\r\n";
    static uint8_t match_index = 0U;
    uint16_t index;
    for (index = 0U; index < length; index++)
    {
        if(data[index] == boot_command[match_index])
        {
            match_index++;
            if (match_index == (sizeof(boot_command) - 1U))
            {
                s_update_pending = 1U;
                match_index = 0U;
            }
        }
        else
        {
            match_index = (data[index] == boot_command[0]?1:0);
        }
    }
}

void CmdStrategy_Init(void)
{
    memset(&s_cmd_rx, 0, sizeof(s_cmd_rx));
    s_update_pending = 0U;
}


void CmdStrategy_ProcessRx(const uint8_t *data,
                           uint16_t length)
{
    uint16_t index;
    CmdFrame_T request;
    CmdTxFrame_T response;

    if (data == NULL || length == 0U)
    {
        return;
    }

    /*
     * BOOT文本命令目前作为开发阶段备用入口。
     * STEP7全部通过后再统一清理。
     */
    CmdStrategy_CheckTextBootCommand(data, length);

    /*
     * 逐字节追加的好处：
     * 即使64字节USB数据中粘了多帧，也能在缓存满前及时解析。
     */
    for (index = 0U; index < length; index++)
    {
        if (s_cmd_rx.length >= CMD_PROTOCOL_RX_BUFFER_SIZE)
        {
            s_cmd_rx.length = 0U;
        }

        s_cmd_rx.buffer[s_cmd_rx.length++] = data[index];

        while (CmdStrategy_TryGetFrame(&request))
        {
            CmdStrategy_Execute(&request, &response);

            if (response.length != 0U)
            {
                (void)CmdStrategy_QueueResponse(&response);
            }
        }
    }
}




/**
 * @brief 在任务上下文执行升级请求
 */
void CmdStrategy_Process(void)
{
    /*解析：（1）如果s_update_pending=1，就继续往下执行，先把标志位复位后
    ，再写入外部EEPROM升级标志位
    并且断开USB外设和电脑的连接，最后进行复位，回到bootloader程序中

    在bootloader程序中，重新配置USB外设并和电脑通讯，main函数里面先判断EEPROM的标志位
    如果读取到 update\r\n 后，会清回 -nope-\r\n，避免以后每次上电都进入升级模式。
    如果没有读取到，则不会发生下一步动作，

    接着会判断根据物理 Boot 键或 EEPROM 标志进入升级并且CD 显示 Bootload。
    因为我们在EEPROM中写入标志位
    所以不用每次按按键了，并且在Bootloader_Main函数中

    （2）如果s_update_pending=0，则直接返回

    */
    if (s_update_pending == 0)
    {
        return;
    }

    s_update_pending=0;

    /*
     * 必须先写成功 EEPROM，再复位。
     * 写失败则留在 当前的App，不会误复位。
     */
    if (!EEPROM_UpdateCommand_Write(true))
    {
        return;
    }

    osDelay(100U);


    USER_USB_DEVICE_DeInit();//卸载USB外设，断开和电脑的通讯

    osDelay(500U);

    /* 下次启动由官方 Bootloader 读取 EEPROM 0x20 的 update\r\n */
    NVIC_SystemReset();

    

}


