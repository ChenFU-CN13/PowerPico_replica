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

#include <stdbool.h>
#include <stddef.h>
#include <string.h>


#define CMD_PROTOCOL_MAX_PAYLOAD    48U     // 单帧业务数据最大 48 字节。
#define CMD_PROTOCOL_FRAME_OVERHEAD 10U     // 帧头2 + 版本1 + 类型1 + 序号2 + 长度2 + CRC2
#define CMD_PROTOCOL_RX_BUFFER_SIZE 64U     // 缓冲区大小为64


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
 * @brief 尝试从缓冲区取出一帧，并识别源工程的 CMD_FW_UPDATE
 * @return true：已经消费一帧或一个错误帧；false：等待更多 USB 数据
 */
static bool CmdStrategy_TryParseFrame(void)
{
    uint16_t magic_index;
    uint16_t payload_length;
    uint16_t frame_length;
    uint16_t expected_crc;
    uint16_t actual_crc;


    while (true)
    {
        magic_index = CmdStrategy_FindMagic();
        if (magic_index == s_cmd_rx.length)
        {
            /* 若最后一个字节是 0xA5，保留它等待下一包的 0x5A */
            if (s_cmd_rx.length != 0U && s_cmd_rx.buffer[s_cmd_rx.length-1] == 0XA5U)
            {
                /* code */
                s_cmd_rx.buffer[0] = 0xA5U;
                s_cmd_rx.length = 1;
            }
            else
            {
                s_cmd_rx.length=0;
            }
            

            return false;
        }
        

        if (magic_index != 0)
        {
            /* code */
            CmdStrategy_RemovePrefix(magic_index);

        }
        
        if (s_cmd_rx.length<CMD_PROTOCOL_FRAME_OVERHEAD)
        {
            return false;
        }
        
        payload_length  = CmdStrategy_ReadU16(&s_cmd_rx.buffer[6]);

        if (payload_length > CMD_PROTOCOL_MAX_PAYLOAD)
        {
            /* code */
            CmdStrategy_RemovePrefix(1);
            continue;
        }
        

        frame_length = (uint16_t)(CMD_PROTOCOL_FRAME_OVERHEAD + payload_length);

        if (s_cmd_rx.length < frame_length)
        {
            return false;
        }
        
        expected_crc = CmdStrategy_ReadU16(
            &s_cmd_rx.buffer[frame_length - 2U]);

        actual_crc = CmdStrategy_Crc16(&s_cmd_rx.buffer[2],
                                       (uint16_t)(6U + payload_length));


        if (expected_crc != actual_crc)
        {
            /* code */
            CmdStrategy_RemovePrefix(1);
            continue;
        }
        
        /*
         * 源工程升级命令：
         * version=1，type=0x15，payload 长度=2，内容=C3 3C。
         */
        if (s_cmd_rx.buffer[2] == CMD_PROTOCOL_VERSION
            && s_cmd_rx.buffer[3] == CMD_FW_UPDATE
            && payload_length == 2U
            && s_cmd_rx.buffer[8] == 0xC3U
            && s_cmd_rx.buffer[9] == 0x3CU) {
            s_update_pending = 1U;
        }


        CmdStrategy_RemovePrefix(frame_length);
        return true;

    }
    



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


void CmdStrategy_ProcessRx(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == NULL || length == 0U) {
        return;
    }

    /* 文本命令仅用于当前过渡阶段调试 */
    CmdStrategy_CheckTextBootCommand(data, length);

    /*
     * 同时支持源工程的二进制升级帧。
     * 本函数只操作 RAM 缓冲和标志位，不做阻塞操作。
     */
    for (index = 0U; index < length; index++) {
        if (s_cmd_rx.length >= CMD_PROTOCOL_RX_BUFFER_SIZE) {
            s_cmd_rx.length = 0U;
        }

        s_cmd_rx.buffer[s_cmd_rx.length++] = data[index];

        while (CmdStrategy_TryParseFrame()) {
            /* 一包 USB 数据中可能连着多帧，持续解析 */
        }
    }
}




/**
 * @brief 在任务上下文执行升级请求
 */
void CmdStrategy_Process(void)
{
    if (s_update_pending == 0)
    {
        return;
    }
    
    s_update_pending=0;

    /*
     * 必须先写成功 EEPROM，再复位。
     * 写失败时留在 App，避免复位后仍进入原 App 却误以为进升级失败。
     */
    if (!EEPROM_UpdateCommand_Write(true))
    {
        return;
    }

    osDelay(100U);

    /* 让 Windows 先看到 App CDC 设备断开 */
    USER_USB_DEVICE_DeInit();

    osDelay(500U);

    /* 下次启动由官方 Bootloader 读取 EEPROM 0x20 的 update\r\n */
    NVIC_SystemReset();

    

}





