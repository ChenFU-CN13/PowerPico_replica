#ifndef __BL24C02_H
#define __BL24C02_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* 源工程 BL24C02 的 7 位 I2C 地址 */
#define BL24C02_ADDRESS      0x50U

/* BL24C02 单页最大写入长度 */
#define EEPROM_PAGE_SIZE      16U

bool EEPROM_ReadBytes(uint8_t address, uint8_t *data, uint16_t length);
bool EEPROM_WritePage(uint8_t address, const uint8_t *data, uint8_t length);

/* 给 Bootloader 写入/读取“下次进入升级模式”标志 */
bool EEPROM_UpdateCommand_Write(bool is_update);
bool EEPROM_UpdateCommand_Check(void);

#endif /* __BL24C02_H */
