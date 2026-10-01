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


#define CMD_PROTOCOL_MAX_PAYLOAD     48U
#define CMD_PROTOCOL_FRAME_OVERHEAD  10U
#define CMD_PROTOCOL_MAX_FRAME_SIZE  \
    (CMD_PROTOCOL_FRAME_OVERHEAD + CMD_PROTOCOL_MAX_PAYLOAD)

/* 最多暂存两帧，处理USB粘包 */
#define CMD_PROTOCOL_RX_BUFFER_SIZE  \
    (CMD_PROTOCOL_MAX_FRAME_SIZE * 2U)

#define CMD_RESPONSE_MASK            0x80U

/**
 * @brief 命令执行结果
 */
typedef enum
{
    CMD_STATUS_OK = 0,                   // 命令执行成功
    CMD_STATUS_UNSUPPORTED_VERSION = 1, // 协议版本不支持
    CMD_STATUS_UNSUPPORTED_COMMAND = 2, // 命令号未实现
    CMD_STATUS_INVALID_LENGTH = 3,      // payload长度错误
    CMD_STATUS_INVALID_PARAMETER = 4,   // 参数内容错误
    CMD_STATUS_EEPROM_ERROR = 5,        // EEPROM保存失败
    CMD_STATUS_BUSY = 6,                // 预留忙状态
    CMD_STATUS_CRC_ERROR = 7            // 预留CRC错误状态
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
 * @brief 按小端顺序写入32位无符号数
 * @param destination 输出字节流
 * @param value 要写入的32位数据
 */
static void CmdStrategy_WriteU32(uint8_t *destination,uint32_t value)
{
    destination[0] = (uint8_t)(value&0XFF);
    destination[1] = (uint8_t)((value>>8)&0XFF);
    destination[2] = (uint8_t)((value>>16)&0XFF);
    destination[3] = (uint8_t)((value>>24));
}

/**
 * @brief 从小端字节流读取32位无符号数
 * @param source 输入字节流
 * @return 读取到的32位数据
 */
static uint32_t CmdStrategy_ReadU32(const uint8_t *source)
{
    return (uint32_t)source[0]
     |((uint32_t)source[1] << 8)
     |((uint32_t)source[2] << 16)
     |((uint32_t)source[3] << 24);
}


/**
 * @brief 把float原始位模式按小端顺序写入协议
 * @param destination 输出字节流
 * @param value 要写入的浮点数
 */
static void CmdStrategy_WriteFloat(uint8_t *destination, float value)
{

    uint32_t bit;
    /*
     * 使用memcpy保留float原始位模式。
     * 不使用uint32_t指针强转，避免未对齐访问和严格别名问题。
     */
    memcpy(&bit,&value,sizeof(bit));
    CmdStrategy_WriteU32(destination,bit);
}

/**
 * @brief 从协议的小端4字节恢复float
 * @param source 输入字节流
 * @return 恢复出的浮点数
 */
static float CmdStrategy_ReadFloat(const uint8_t *source)
{
    uint32_t bits;
    float value;
    bits = CmdStrategy_ReadU32(source);
    memcpy(&value,&bits,sizeof(value));
    return value;
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
             * 当前缓存最后一个字节是0xA5，
             * 它可能是一个被USB拆开的帧头前半部分。
             * 例如：
             * 第一包：11 22 A5
             * 第二包：5A 01 10 ...
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
 * @brief 编码一帧完整命令数据（请求帧和回复帧共用的底层组帧函数）
 *
 * 本函数把传入的类型/序列号/payload 按协议字节布局逐个写入输出缓冲区，
 * 并在末尾补上 CRC16，形成可直接通过 USB 发送的完整字节帧。
 *
 * 生成的帧字节布局（小端序，多字节字段低字节在前）：
 *
 *   偏移   长度   字段
 *   ─────────────────────────────────────────────
 *   [0]    2     帧头 0xA5 0x5A
 *   [2]    1     协议版本（固定 CMD_PROTOCOL_VERSION = 1）
 *   [3]    1     命令类型（回复帧已在此前 |0x80）
 *   [4]    2     序列号 sequence
 *   [6]    2     payload 长度 payload_length
 *   [8]    N     payload 业务数据（N = payload_length，可为 0）
 *   [8+N]  2     CRC16
 *   ─────────────────────────────────────────────
 *   总长度 = 10 + payload_length = CMD_PROTOCOL_FRAME_OVERHEAD + payload_length
 *
 * @param type           命令类型字节；回复帧调用时传入 (原命令号 | 0x80)
 * @param sequence       16 位命令序列号，用于上位机匹配请求与回复
 * @param payload        业务数据首地址；payload_length=0 时可为 NULL
 * @param payload_length 业务数据字节数，最大不超过 CMD_PROTOCOL_MAX_PAYLOAD(48)
 * @param output         输出缓冲区首地址，需能容纳整帧（最大 58 字节）
 * @return 编码后的整帧总长度；参数非法时返回 0
 */
static uint16_t CmdStrategy_EncodeFrame(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output)
{
    uint16_t crc;   // 本帧计算出的 CRC16，写入帧尾

    /* 入参合法性检查，三项任一成立都无法组帧：
     * 1. output 为空：没有地方写结果；
     * 2. payload 超长：超出协议上限，防止写爆 output；
     * 3. payload 为空却声称有数据：memcpy 空指针会进硬件异常。
     */
    if (output == NULL
        || payload_length > CMD_PROTOCOL_MAX_PAYLOAD
        || (payload == NULL && payload_length != 0U))
    {
        return 0U;
    }

    /* ---- 按偏移逐字段写入固定头部 ---- */

    output[0] = 0xA5U;                    // 帧头第1字节
    output[1] = 0x5AU;                    // 帧头第2字节
    output[2] = CMD_PROTOCOL_VERSION;     // 协议版本，固定为 1
    output[3] = type;                     // 命令类型（回复时已带 0x80 标志）

    /* 2 字节字段统一用 WriteU16 按小端写入（先低字节后高字节）。 */
    CmdStrategy_WriteU16(&output[4], sequence);         // 序列号
    CmdStrategy_WriteU16(&output[6], payload_length);   // payload 长度

    /* 只有携带业务数据时才复制；payload_length=0 时跳过，
     * output[8] 直接作为后续 CRC 的位置。
     */
    if (payload_length != 0U)
    {
        memcpy(&output[8], payload, payload_length);
    }

    /*
     * CRC 从 version（output[2]）开始计算，不包括 A5 5A 帧头。
     *
     * 参与 CRC 的数据 = 固定部分 6 字节
     *   (version 1 + type 1 + sequence 2 + payload_length 2)
     *   + payload_length 字节业务数据
     * 即总长度 6 + payload_length。
     */
    crc = CmdStrategy_Crc16(
        &output[2],
        (uint16_t)(6U + payload_length));

    /* 把 CRC 按小端写到 payload 之后（帧的最后 2 字节）。 */
    CmdStrategy_WriteU16(
        &output[8U + payload_length],
        crc);

    /* 返回整帧长度：固定开销 10 + payload 长度。 */
    return (uint16_t)(
        CMD_PROTOCOL_FRAME_OVERHEAD + payload_length);
}

/**
 * @brief 编码命令回复帧
 *
 * 本函数在调用底层 EncodeFrame 之前，先把"状态码 + 命令返回数据"拼成
 * 回复帧专用的 payload。这是回复帧与请求帧在结构上的唯一区别。
 *
 * 回复帧 payload 布局：
 *
 *   偏移(相对payload)   长度   字段
 *   ────────────────────────────────────────────
 *   [0]                1     状态码 status（必有）
 *   [1]                M     命令返回数据（M = payload_length，可为 0）
 *   ────────────────────────────────────────────
 *   回复 payload 总长度 = 1 + payload_length
 *
 * 注意两个关键点：
 * 1. 状态码固定占据 payload[0]，所以即使命令执行失败、没有任何返回数据，
 *    回复帧的 payload 也至少有 1 字节（状态码），这与请求帧允许 payload=0 不同；
 * 2. 调用 EncodeFrame 时命令类型要 | CMD_RESPONSE_MASK(0x80)，
 *    例如请求 0x11 的回复类型为 0x91，供上位机区分收发方向。
 *
 * @param request        对应的请求帧（从中取命令号和序列号回填到回复）
 * @param status         命令执行状态码（OK / 各类错误码）
 * @param payload        命令返回的业务数据首地址；payload_length=0 时可为 NULL
 * @param payload_length 命令返回数据的字节数（不含状态码）
 * @param output         输出缓冲区首地址，存放最终回复帧
 * @return 回复帧整帧长度；参数非法时返回 0
 */
static uint16_t CmdStrategy_EncodeResponse(
    const CmdFrame_T *request,
    CmdStatus_T status,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *output)
{
    /* 回复专用 payload：第0字节状态码 + 后续返回数据。
     * 用局部数组拼装，避免改动调用方传入的 payload 原数据。
     */
    uint8_t response_payload[CMD_PROTOCOL_MAX_PAYLOAD];

    /* 入参合法性检查：
     * 1/2. request、output 为空无法工作；
     * 3. 注意这里用 >= 而非 >：因为还要再塞 1 字节状态码，
     *    返回数据若已达 48，加上状态码会变成 49 超出上限；
     * 4. payload 为空却声称有数据，memcpy 会触发硬件异常。
     */
    if (request == NULL
        || output == NULL
        || payload_length >= CMD_PROTOCOL_MAX_PAYLOAD
        || (payload == NULL && payload_length != 0U))
    {
        return 0U;
    }

    /* 状态码固定放在回复 payload 的第 0 字节。 */
    response_payload[0] = (uint8_t)status;

    /* 命令返回数据紧跟状态码之后；没有返回数据时跳过。 */
    if (payload_length != 0U)
    {
        memcpy(&response_payload[1],
               payload,
               payload_length);
    }

    /* 交给通用组帧函数完成最终打包：
     * - 类型 |= 0x80 标记这是回复；
     * - 序列号原样回填，保证上位机能对应到原请求；
     * - payload 长度传 (payload_length + 1)，多出的 1 是状态码。
     */
    return CmdStrategy_EncodeFrame(
        (uint8_t)(request->type | CMD_RESPONSE_MASK),
        request->sequence,
        response_payload,
        (uint16_t)(payload_length + 1U),
        output);
}


/**
 * @brief 把三档六个校准参数编码成24字节
 * @param payload 输出载荷
 * @param calibration 校准参数
 *
 * 字节顺序：
 * [0..3]   LOW增益
 * [4..7]   LOW零点偏移，单位uA
 * [8..11]  MID增益
 * [12..15] MID零点偏移，单位uA
 * [16..19] HIGH增益
 * [20..23] HIGH零点偏移，单位uA
 */
static void CmdStrategy_WriteCalibrationPayload(uint8_t *payload,const ADC_Calibration_T *calibration)
{
    CmdStrategy_WriteFloat(
        &payload[0],
        calibration->low_scale_multiplier);

    CmdStrategy_WriteFloat(
        &payload[4],
        calibration->low_offset_ua);

    CmdStrategy_WriteFloat(
        &payload[8],
        calibration->mid_scale_multiplier);

    CmdStrategy_WriteFloat(
        &payload[12],
        calibration->mid_offset_ua);

    CmdStrategy_WriteFloat(
        &payload[16],
        calibration->high_scale_multiplier);

    CmdStrategy_WriteFloat(
        &payload[20],
        calibration->high_offset_ua);
}



/**
 * @brief 从24字节载荷恢复三档校准参数
 * @param calibration 校准参数输出地址
 * @param payload 输入载荷
 */
static void CmdStrategy_ReadCalibrationPayload(ADC_Calibration_T *calibration,const uint8_t *payload)
{
    calibration->low_scale_multiplier =
        CmdStrategy_ReadFloat(&payload[0]);

    calibration->low_offset_ua =
        CmdStrategy_ReadFloat(&payload[4]);

    calibration->mid_scale_multiplier =
        CmdStrategy_ReadFloat(&payload[8]);

    calibration->mid_offset_ua =
        CmdStrategy_ReadFloat(&payload[12]);

    calibration->high_scale_multiplier =
        CmdStrategy_ReadFloat(&payload[16]);

    calibration->high_offset_ua =
        CmdStrategy_ReadFloat(&payload[20]);
}

/**
 * @brief 校验并保存校准参数
 * @param calibration 新校准参数
 * @return 命令执行状态
 *
 * EEPROM写入失败时恢复旧RAM参数，
 * 防止RAM已经生效、EEPROM却没有保存。
 */
static CmdStatus_T CmdStrategy_SaveCalibration(const ADC_Calibration_T *calibration)
{
    ADC_Calibration_T previous;

    /* 先备份当前正在使用的参数 */
    Sys_Get_AdcCalibration(&previous);

    /* 参数非法时不能覆盖旧参数 */
    if (!Sys_Set_AdcCalibration(calibration))
    {
        return CMD_STATUS_INVALID_PARAMETER;
    }
    
    /* 将整份系统设置写入EEPROM */
    if (!EEPROM_SysSetting_Save())
    {
        /* EEPROM失败，恢复原来的RAM参数 */
        (void)Sys_Set_AdcCalibration(&previous);
        return CMD_STATUS_EEPROM_ERROR;

    }
    
    return CMD_STATUS_OK;
}




/**
 * @brief 执行命令并生成回复
 * 本函数是命令分发中心：根据请求帧的 cmd 字段路由到对应处理分支，
 * 校验长度/参数合法性后填充回复 payload，再调用 EncodeResponse 打包。
 * 调用上下文：MessageReceiveTask 中被 CmdStrategy_ProcessRx 调用，
 *           属于任务上下文（非 ISR），可安全访问 EEPROM/GPIO。
 * 回复命令号规则：响应帧 type = 请求 type | 0x80（在 EncodeResponse 内完成）。
 * @param request   已解析完成的请求帧（帧头/CRC 已校验通过）
 * @param response  输出参数，函数填充 length 和 data 后由调用者发送
 */
static void CmdStrategy_Execute(const CmdFrame_T *request, CmdTxFrame_T *response)
{
    CmdStatus_T status = CMD_STATUS_OK;
    ADC_Calibration_T calibration;  //校准参数临时存储

    /*
     * 最大回复业务数据为48字节校准参数。
     * 使用协议允许的最大空间，留1字节给状态码，所以最大值-1，留出47字节给业务回复
     * 因为业务回复最大是6个float校准参数6*4=24字节。
     */
    uint8_t payload[CMD_PROTOCOL_MAX_PAYLOAD - 1] = {0};
    uint16_t payload_length = 0U;

    response->length = 0U;

    /*1.检查协议版本是否支持*/
    if (request -> version != CMD_PROTOCOL_VERSION)
    {   
        /*说明协议版本不对*/
        status = CMD_STATUS_UNSUPPORTED_VERSION;
    }
    else
    {
        /*说明协议版本正确*/
        /*开始根据命令类型分别执行业务逻辑*/
        switch (request -> type)
        {
            /*1.读取三档校准参数，下发 payload_length = 0，如果不为0说明数据帧出错，不可用*/
            case CMD_CAL_GET:
                if (request->payload_length != 0)
                {
                    status = CMD_STATUS_INVALID_LENGTH;
                }
                else
                {
                    /*(1)先把当前正在使用的有效参数备份到RAM变量*/
                    Sys_Get_AdcCalibration(&calibration);

                    /*
                     * 把六个float编码成固定顺序的24字节：
                     *
                     * [0..3]   LOW增益
                     * [4..7]   LOW零点偏移
                     * [8..11]  MID增益
                     * [12..15] MID零点偏移
                     * [16..19] HIGH增益
                     * [20..23] HIGH零点偏移
                     */
                    CmdStrategy_WriteCalibrationPayload(payload,&calibration);
                    /*6个float ，每个4字节*/
                    payload_length = 24U;

                }
                break;
        
            case CMD_RANGE_GET:
                /*2.读取当前量程模式以及实际硬件的档位*/
                /*和上面一样，查询命令的payload长度为0*/
                if (request->payload_length != 0)
                {
                    status = CMD_STATUS_INVALID_LENGTH;
                }
                else
                {
                                        /*
                     * payload[0]：用户选择的控制模式
                     *
                     * 0 = AUTO
                     * 1 = LOW
                     * 2 = MID
                     * 3 = HIGH
                     *
                     * payload[1]：硬件当前真正接通的量程
                     *
                     * 1 = LOW
                     * 2 = MID
                     * 3 = HIGH
                     */
                    payload[0] = Gate_Get_Mode();
                    payload[1] = Gate_get_status();

                    payload_length = 2U;
                }
                

                break;

            case CMD_RANGE_SET:
                /*3.设置自动或者手动量程，这个是设置类命令，payload长度！=0才对
                 * payload[0] = AUTO、LOW、MID或HIGH*/
                if (request->payload_length != 1U)
                {
                    status = CMD_STATUS_INVALID_LENGTH;
                }
                else if (request->payload[0] > GATE_MODE_HIGH)
                {
                    /*
                     * 当前合法枚举值为0～3。
                     * request->payload[0]是uint8_t，
                     * 不可能小于0，所以只检查是否超过HIGH。
                     */
                    status = CMD_STATUS_INVALID_PARAMETER;
                }
                else
                {
                    /*
                     * 修改当前量程控制模式。
                     * 如果设置LOW、MID或HIGH：
                     *     Gate_Set_Mode()会进一步切换GATE GPIO，
                     *     让硬件立即进入指定量程。
                     * 如果设置AUTO：
                     *     只恢复自动控制模式，
                     *     后续由Process_ADC_Chunk()判断实际档位。
                     * 这里不调用EEPROM_SysSetting_Save()，
                     * 所以属于临时量程切换。
                     * 官方Client做自动零点校准时，可以：
                     * 切LOW → 采集LOW零点
                     * 切MID → 采集MID零点
                     * 切HIGH → 采集HIGH零点
                     * 最后切回AUTO
                     * 不会永久修改用户原有的量程设置。
                     */
                    Gate_Set_Mode(request->payload[0]);

                    /*
                     * 切换完成后，把软件模式和硬件实际档位回传。
                     * 上位机不能只根据自己发送的值判断切换成功。
                     */
                    payload[0] = Gate_Get_Mode();
                    payload[1] = Gate_get_status();

                    payload_length = 2U;
                }
                

                break;


            case CMD_CAL_SET:
                /*4.写入三档校准参数*/
                /*6个float必须正好24字节，长度不是24，不能解析*/
                if (request->payload_length != 24U)
                {
                    status = CMD_STATUS_INVALID_LENGTH;
                }
                else
                {
                    /*先读取当前正在使用的有效参数*/
                     Sys_Get_AdcCalibration(&calibration);

                    /*将下发数据帧中的payload的24字节 恢复 为6字节
                    这一步只负载字节转换*/
                    CmdStrategy_ReadCalibrationPayload(
                        &calibration,
                        request->payload);

                    /*统一执行校验，RAM更新，EEPROM保存*/
                    /*
                     * 内部执行顺序：
                     *
                     * 1. 备份原校准参数；
                     * 2. 检查新参数是否合法；
                     * 3. 合法后更新RAM；
                     * 4. 保存整份系统设置到EEPROM；
                     * 5. EEPROM失败时恢复旧RAM参数。
                     * 可能返回：
                     * CMD_STATUS_OK
                     * CMD_STATUS_INVALID_PARAMETER
                     * CMD_STATUS_EEPROM_ERROR
                     * */
                    status = CmdStrategy_SaveCalibration(&calibration);

                    if (status == CMD_STATUS_OK)
                    {
                        /* 注意：这里不是直接将 下发的数据 原样返回，而是保存成功后，再从系统中读取实际生效参数
                        再传回给上位机，这样确保上位机收到的是设备实际真正保存的结果 */
                        Sys_Get_AdcCalibration(&calibration);

                        CmdStrategy_WriteCalibrationPayload(
                            payload,
                            &calibration);
                        payload_length = 24;    
                        
                    }
                    /*如果保存失败，有status记录原因，负载长度依旧为0*/

                }
                
                break;

            case CMD_CAL_RESET:
                /*5.恢复默认出厂校准参数*/

                /*
                 * 恢复出厂校准属于会修改EEPROM的操作，
                 * 因此要求上位机携带两个确认字节。
                 */
                if (request->payload_length != 2U)
                {
                    status = CMD_STATUS_INVALID_LENGTH;
                }
                else if (request->payload[0] != 0xC3U
                         || request->payload[1] != 0x3CU)
                {
                    /* 确认码不正确时拒绝执行。*/
                    status = CMD_STATUS_INVALID_PARAMETER;
                }
                else
                {
                    /*
                     * 生成默认校准参数：
                     * LOW增益  = 1.0
                     * MID增益  = 1.0
                     * HIGH增益 = 1.0
                     * LOW偏移  = 0
                     * MID偏移  = 0
                     * HIGH偏移 = 0
                     */
                    ADC_Calibration_SetDefault(&calibration);

                    /*
                     * 默认参数同样需要：
                     * 校验
                     * → 更新RAM
                     * → 写EEPROM
                     * → 失败时恢复旧RAM
                     */
                    status =
                        CmdStrategy_SaveCalibration(&calibration);

                    if (status == CMD_STATUS_OK)
                    {
                        /*
                         * 保存成功后重新读取实际结果，
                         * 再把六个参数回传给官方Client。
                         */
                        Sys_Get_AdcCalibration(&calibration);

                        CmdStrategy_WriteCalibrationPayload(
                            payload,
                            &calibration);

                        payload_length = 24U;
                    }
                }

                break;

            case CMD_FW_UPDATE:
                /*6.请求进入bootloader升级模式*/

                /*同上，先判断是否合法*/
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
                     * 保留当前复刻工程已经验证过的升级结构。
                     * 此处不直接访问EEPROM、不直接卸载USB，
                     * 也不直接执行NVIC_SystemReset()。
                     * 这里只设置升级请求标志。
                     * defaultTask后续调用CmdStrategy_Process()时，再统一完成整体升级步骤
                     */       
                    s_update_pending = 1U;
                     
                    return;
                }
                break;

        default:
                /*
                 * 命令号不属于当前协议支持范围。
                 *
                 * 不执行任何业务操作，
                 * 最终只回复“不支持该命令”状态码。
                 */
                status = CMD_STATUS_UNSUPPORTED_COMMAND;

            break;
        }

    }

    /*
     * 除合法的固件升级请求以外，
     * 所有命令最终都从这里统一生成回复。
     *
     * CmdStrategy_EncodeResponse()会完成：
     *
     * 1. 在业务payload前加入1字节status；
     * 2. 将回复命令号设置为request->type | 0x80；
     * 3. 原样带回请求序号request->sequence；
     * 4. 写入payload长度；
     * 5. 计算并追加CRC16；
     * 6. 返回整帧有效长度。
     *
     * 成功读取校准参数时：
     *
     * status              1字节
     * calibration payload 24字节
     * 回复payload总长度   25字节
     * 整帧总长度          10 + 25 = 35字节
     *
     * 命令执行失败时：
     *
     * payload_length = 0
     * 回复payload只有1字节status
     * 上位机可以根据status显示具体错误原因。
     */
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

