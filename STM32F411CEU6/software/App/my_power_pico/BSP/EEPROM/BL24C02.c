#include "BL24C02.h"
#include "i2c.h"

// hardware settings

#define EEPROM_PAGE_SIZE            16U         // BL24C02 单页最大写入 16 字节，跨页必须拆开写。
#define EEPROM_SYS_SETTINGS_ADDRESS 0x40U        // 整个系统设置结构体在 EEPROM 中的固定起始地址。


/******************************************
EEPROM Data description:
[0x00]: 0x55 for check
[0x01]: 0xAA for check

[0x20-]: update command storage area, "update\r\n"
[0x40-]: SysSettings_T
*******************************************/


/**
 * @brief 从 EEPROM 连续读取数据
 * @param address EEPROM 内部起始地址，范围 0x00~0xFF
 * @param data 接收缓冲区
 * @param length 读取长度
 * @return true：读取成功；false：参数或 I2C 通信失败
 */
bool EEPROM_ReadBytes(uint8_t address, uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || ((uint16_t)address + length) > 256U)
    {
        return false;
    }
    return HAL_I2C_Mem_Read(&hi2c1,
                            BL24C02_ADDRESS<<1, //芯片I2C地址，STM32要求设备地址放在高7位，所以左移1位
                            address,            //写入数据在芯片哪个位置
                            I2C_MEMADD_SIZE_8BIT,//EEPROM 的内部地址宽度是 8 bit。
                            data,
                            length,                           
                            50U                        
                            ) == HAL_OK;
}


/**
 * @brief 在 EEPROM 单页内写入数据
 * @param address EEPROM 内部起始地址
 * @param data 待写数据
 * @param length 写入长度，最大 16 字节，不能跨页
 * @return true：写入成功；false：参数或 I2C 通信失败
 */
bool EEPROM_WritePage(uint8_t address, const uint8_t *data, uint8_t length)
{
    if (data == NULL 
        || length == 0 
        || length > EEPROM_PAGE_SIZE                                        //总长度是不是超过了一页？
        || (address & (EEPROM_PAGE_SIZE - 1U)) + length > EEPROM_PAGE_SIZE  //虽然总长度没超过一页，但从当前地址开始会不会跨页？
        || ((uint16_t)address + length) > 256)                              //防止超过整颗 EEPROM,
        /*因为24C02是2Kbit=256Byte，有效地址为0x00-0xff*/
        /*这里强转为uint16_t是因为数学计算上，可能会＞255，比如256，所以用无符号16位刚刚好满足未知情况*/

    {
        return false;
    }

if (HAL_I2C_Mem_Write(&hi2c1,
                      BL24C02_ADDRESS << 1,
                      address,
                      I2C_MEMADD_SIZE_8BIT,
                      (uint8_t *)data,
                      length,
                      50U) != HAL_OK) {
    return false;
}
    
     /* 等待 EEPROM 内部写周期结束，重新 ACK 后才算写成功 */
    return HAL_I2C_IsDeviceReady(&hi2c1,
                                BL24C02_ADDRESS << 1,
                                20U,
                                2U) == HAL_OK;
}


/**
 * @brief 写入或清除下次进入 Bootloader 的请求
 * @param is_update true：请求升级；false：清除请求
 * @return true：写入成功；false：I2C 通信失败
 */
bool EEPROM_UpdateCommand_Write(bool is_update)
{
    static const uint8_t update_command[8] = 
    {
        'u', 'p', 'd', 'a', 't', 'e', '\r', '\n'
    };

    static const uint8_t normal_command[8] = 
    {
        '-', 'n', 'o', 'p', 'e', '-', '\r', '\n'
    };

    return EEPROM_WritePage(0X20,is_update?update_command:normal_command,8);
}

/**
 * @brief 判断 EEPROM 中是否存在升级请求
 * @return true：内容是 update\r\n；false：不是或读取失败
 */
bool EEPROM_UpdateCommand_Check(void)
{
     uint8_t command[8];

    if (!EEPROM_ReadBytes(0x20U, command, sizeof(command))) {
        return false;
    }
    return memcmp(command, "update\r\n", 8U) == 0;
}






