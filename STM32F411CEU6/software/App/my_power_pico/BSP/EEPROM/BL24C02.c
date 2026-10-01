#include "BL24C02.h"
#include "i2c.h"

// hardware settings

#define EEPROM_PAGE_SIZE            16U         // BL24C02 单页最大写入 16 字节，跨页必须拆开写。
#define EEPROM_SYS_SETTINGS_ADDRESS 0x40U        // 整个系统设置结构体在 EEPROM 中的固定起始地址。

/**
 * @brief 保存在EEPROM 0x40区域的系统设置
 *
 * 当前STEP6,本轮虽然只测试ADC校准，但保留官方设置结构，
 * 后面背光、按键音、屏幕方向和量程模式可以继续复用。
 */
typedef struct
{
    uint8_t backlight_level;       // 背光亮度，合法范围0~100
    uint8_t key_sound_enable;      // 按键音：0关闭，1开启
    uint8_t language_select;       // 语言：0英文，1中文
    uint16_t rotation;             // 屏幕方向：0、90、180或270
    uint8_t current_range_mode;    // AUTO、LOW、MID或HIGH
    ADC_Calibration_T adc_calibration; // 三档电流校准参数
} SysSettings_T;


/*
 * 系统运行期间只使用这份RAM副本。
 *
 * ADC换算时不会每次访问EEPROM，
 * 而是直接读取这里保存的校准参数。
 */
static SysSettings_T s_sys_settings = 
{
    .backlight_level = 50,
    .key_sound_enable = 1,
    .language_select = 0,
    .current_range_mode = GATE_MODE_AUTO,
    .rotation = 0,

    .adc_calibration = 
    {
        .high_scale_multiplier = 1.0f,
        .mid_scale_multiplier = 1.0f,
        .low_scale_multiplier = 1.0f,

        .low_offset_ua = 0.0f,
        .mid_offset_ua = 0.0f,
        .high_offset_ua = 0.0f
    }

};

/**
 * @brief 把整份系统设置恢复成默认值
 * @param settings 要恢复的设置结构体
 */
static void SysSettings_SetDefault(SysSettings_T *settings)
{
    if (settings == NULL)
    {
        return;
    }
    
    settings->backlight_level = 50;
    settings->current_range_mode = GATE_MODE_AUTO;
    settings->key_sound_enable = 1;
    settings->language_select = 0;
    settings->rotation = 0;

    ADC_Calibration_SetDefault(&settings->adc_calibration);

}

/**
 * @brief 检查EEPROM读出的系统设置
 * @param settings 待检查的设置
 * @return true 所有字段有效；false 存在非法字段
 */








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
 * @brief 跨页连续写入EEPROM.24字节先分为16字节写入一页，后8字节写入后一页
 * @param address EEPROM内部起始地址
 * @param data 待写数据
 * @param length 写入长度
 * @return true：写入成功；false：参数错误或I2C通信失败
 */
bool EEPROM_WriteBytes(uint8_t address,
                       const uint8_t *data,
                       uint16_t length)
{
    uint16_t current_address = address;
    uint16_t remaining = length;

    
    if (data == NULL || length == 0 || ((uint16_t)address+length) > 256U) {
        return false;
    }

    while (remaining != 0)
    {
        uint8_t page_space;
        uint8_t chunk_length;

        /*计算当前页还剩多少空间*/
        page_space = (uint8_t)(EEPROM_PAGE_SIZE-(current_address & (EEPROM_PAGE_SIZE - 1)));

        /* 本次只写当前页能够容纳的数据 */
        chunk_length = remaining < page_space
            ? (uint8_t)remaining
            : page_space;

        if (!EEPROM_WritePage((uint8_t)current_address,
                              data,
                              chunk_length))
        {
            return false;
        }
            
        current_address += chunk_length;
        data += chunk_length;
        remaining -= chunk_length;

    }
    return true;
    
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


/**
 * @brief 检查EEPROM读出的系统设置
 * @param settings 待检查的设置
 * @return true 所有字段有效；false 存在非法字段
 */
static bool SysSettings_IsValid(
    const SysSettings_T *settings)
{
    bool rotation_valid;
    bool range_valid;

    if (settings == NULL)
    {
        return false;
    }


    rotation_valid =
        settings->rotation == 0U
        || settings->rotation == 90U
        || settings->rotation == 180U
        || settings->rotation == 270U;

    range_valid =
        settings->current_range_mode == GATE_MODE_AUTO
        || settings->current_range_mode == GATE_MODE_LOW
        || settings->current_range_mode == GATE_MODE_MID
        || settings->current_range_mode == GATE_MODE_HIGH;


    return settings->backlight_level <= 100U
        && settings->key_sound_enable <= 1U
        && settings->language_select <= 1U
        && rotation_valid
        && range_valid
        && ADC_Calibration_IsValid(
               &settings->adc_calibration);





}


/**
 * @brief 从EEPROM恢复系统设置
 * @return true EEPROM设置有效；false 已使用默认值
 */
bool EEPROM_SysSetting_Load(void)
{
    SysSettings_T loaded_settings;

    /*
        先将EEPROM数据读取出来存入局部变量
        接着验证读取出来的数据是否合法
     */
    if (!EEPROM_ReadBytes(EEPROM_SYS_SETTINGS_ADDRESS,
                          (uint8_t *)&loaded_settings,
                          (uint16_t)sizeof(loaded_settings))
        || !SysSettings_IsValid(&loaded_settings))
    {
        /*如果
         * EEPROM读取失败、旧数据或非法数据，
         * 都统一恢复为安全默认值。
         */
        SysSettings_SetDefault(&s_sys_settings);
        return false;

    }
    /*如果数据合法，则赋值给全局变量，供外部使用*/
     s_sys_settings = loaded_settings;

    return true;




}




/**
 * @brief 把当前RAM中的整份系统设置保存到EEPROM
 * @return true 参数有效且写入成功；false 参数非法或I2C写入失败
 */
bool EEPROM_SysSetting_Save(void)
{
    SysSettings_T snapshot;
    /*
     * 先复制一份稳定快照。
     * EEPROM写入时间较长，不能在写入过程中继续直接读取全局结构。
     */
    __disable_irq();
    snapshot = s_sys_settings;
    __enable_irq();

    /*
     * 保存前再检查一次RAM副本。
     * 防止异常数据被写入EEPROM，导致下次启动继续加载错误参数。
     */
    if (!SysSettings_IsValid(&snapshot))
    {
        return false;
    }

    /*
     * SysSettings_T超过EEPROM单页长度时，
     * EEPROM_WriteBytes()会自动拆成多个不跨页的数据块写入。
     */
    return EEPROM_WriteBytes(
        EEPROM_SYS_SETTINGS_ADDRESS,
        (const uint8_t *)&snapshot,
        (uint16_t)sizeof(snapshot));
}




/**
 * @brief 更新当前RAM中的校准参数
 * @param calibration 新校准参数
 * @return true 参数有效；false 参数非法
 */
bool Sys_Set_AdcCalibration(
    const ADC_Calibration_T *calibration)
{
    /*
     * 先校验再覆盖。
     * 非法参数不能破坏当前仍然有效的参数。
     */
    if (!ADC_Calibration_IsValid(calibration))
    {
        return false;
    }

    /*
     * 六个float作为一组整体更新，
     * 防止其他任务在复制中途读取到混合参数。
     */
    __disable_irq();
    s_sys_settings.adc_calibration = *calibration;
    __enable_irq();
    return true;
}


/**
 * @brief 获取当前生效的校准参数
 * @param calibration 校准参数输出地址
 */
void Sys_Get_AdcCalibration(
    ADC_Calibration_T *calibration)
{
    if (calibration == NULL)
    {
        return;
    }

    /*
     * 保证调用者取得的是同一时刻的一整组参数。
     */
    __disable_irq();
    *calibration = s_sys_settings.adc_calibration;
    __enable_irq();
}

bool Sys_Set_BacklightLevel(uint8_t level)
{
    if (level > 100U)
    {
        return false;
    }

    s_sys_settings.backlight_level = level;
    return true;
}

bool Sys_Set_KeySoundEnable(bool enable)
{
    s_sys_settings.key_sound_enable = enable ? 1U : 0U;
    return true;
}

bool Sys_Set_LanguageSelect(uint8_t language)
{
    if (language > 1U)
    {
        return false;
    }

    s_sys_settings.language_select = language;
    return true;
}

bool Sys_Set_Rotation(uint16_t rotation)
{
    if (rotation != 0U
        && rotation != 90U
        && rotation != 180U
        && rotation != 270U)
    {
        return false;
    }

    s_sys_settings.rotation = rotation;
    return true;
}

bool Sys_Set_CurrentRangeMode(uint8_t mode)
{
    if (mode != GATE_MODE_AUTO
        && mode != GATE_MODE_LOW
        && mode != GATE_MODE_MID
        && mode != GATE_MODE_HIGH)
    {
        return false;
    }

    s_sys_settings.current_range_mode = mode;
    return true;
}

uint8_t Sys_Get_BacklightLevel(void)
{
    return s_sys_settings.backlight_level;
}

bool Sys_Get_KeySoundEnable(void)
{
    return s_sys_settings.key_sound_enable != 0U;
}

uint8_t Sys_Get_LanguageSelect(void)
{
    return s_sys_settings.language_select;
}

uint16_t Sys_Get_Rotation(void)
{
    return s_sys_settings.rotation;
}

uint8_t Sys_Get_CurrentRangeMode(void)
{
    return s_sys_settings.current_range_mode;
}
