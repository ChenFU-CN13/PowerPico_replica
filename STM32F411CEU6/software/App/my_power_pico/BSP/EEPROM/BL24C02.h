#ifndef __BL24C02_H
#define __BL24C02_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "user_AdcDataStrategy.h"
#include "gate.h"
/* 源工程 BL24C02 的 7 位 I2C 地址 */
#define BL24C02_ADDRESS      0x50U

/* BL24C02 单页最大写入长度 */
#define EEPROM_PAGE_SIZE      16U

/**
 * @brief 跨页连续写入EEPROM
 * @param address EEPROM内部起始地址
 * @param data 待写数据
 * @param length 写入长度
 * @return true：写入成功；false：参数错误或I2C通信失败
 */
bool EEPROM_WriteBytes(uint8_t address,const uint8_t *data,uint16_t length);

bool EEPROM_ReadBytes(uint8_t address, uint8_t *data, uint16_t length);
bool EEPROM_WritePage(uint8_t address, const uint8_t *data, uint8_t length);

/* 给 Bootloader 写入/读取“下次进入升级模式”标志 */
bool EEPROM_UpdateCommand_Write(bool is_update);
bool EEPROM_UpdateCommand_Check(void);


/**
 * @brief 从EEPROM加载系统设置
 * @return true EEPROM数据有效；false 已恢复默认设置
 */
bool EEPROM_SysSetting_Load(void);

/**
 * @brief 把当前RAM设置保存到EEPROM
 * @return true 保存成功；false EEPROM写入失败
 */
bool EEPROM_SysSetting_Save(void);

/**
 * @brief 更新RAM中的ADC校准参数
 * @param calibration 新校准参数
 * @return true 参数有效并已更新；false 参数无效
 */
bool Sys_Set_AdcCalibration(
    const ADC_Calibration_T *calibration);

/**
 * @brief 获取RAM中当前生效的ADC校准参数
 * @param calibration 校准参数输出地址
 */
void Sys_Get_AdcCalibration(
    ADC_Calibration_T *calibration);


/* 系统显示设置 */
bool Sys_Set_BacklightLevel(uint8_t level);         // 设置背光亮度
bool Sys_Set_KeySoundEnable(bool enable);           // 设置按键音效
bool Sys_Set_LanguageSelect(uint8_t language);      // 设置语言
bool Sys_Set_Rotation(uint16_t rotation);           // 设置屏幕旋转角度
bool Sys_Set_CurrentRangeMode(uint8_t mode);        // 设置当前工作模式

uint8_t Sys_Get_BacklightLevel(void);               // 获取背光亮度
bool Sys_Get_KeySoundEnable(void);                 // 获取按键音效
uint8_t Sys_Get_LanguageSelect(void);               // 获取语言
uint16_t Sys_Get_Rotation(void);                     // 获取屏幕旋转角度
uint8_t Sys_Get_CurrentRangeMode(void);             // 获取当前工作模式


#endif /* __BL24C02_H */
