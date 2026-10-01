#include "ui_helpers.h"
#include "tim.h"
#include "lcd_init.h"
#include "BL24C02.h"
#include "gate.h"
#include "user_TasksInit.h"

/*
 * UI测量数据缓存。
 *
 * 只有LvHandlerTask会写入；
 * 主页的LVGL定时器也在LvHandlerTask中读取。
 * 因此STEP8A不需要互斥锁。
 */
static float s_ui_voltage_v;
static float s_ui_current_ua;


/**
 * @brief 强制LVGL刷新整个屏幕
 */
void ui_full_screen_refresh(lv_obj_t *screen)
{
    if (screen == NULL)
    {
        return;
    }

    lv_obj_invalidate(screen);  //标记需要重新刷新的区域
    lv_refr_now(NULL);          //刷新UI
}


/**
 * @brief 把TIM5微秒计数换算为时分秒
 */
void ui_GetElapsedTime_HMS(uint8_t *hours,
                           uint8_t *minutes,
                           uint8_t *seconds)
{
    uint64_t total_us;
    uint32_t total_seconds;
    total_us = GetMicrosecondCounter();
    total_seconds = (uint32_t)(total_us / 1000000ULL);  //ms转us

    if (hours != NULL)
    {
        /*
         * 主页现有接口使用uint8_t保存小时。
         * 连续运行超过255小时后会回绕，后续再单独优化。
         */
        *hours = (uint8_t)(total_seconds / 3600U);
    }
    if (minutes != NULL)
    {
        *minutes =
            (uint8_t)((total_seconds % 3600U) / 60U);
    }

    if (seconds != NULL)
    {
        *seconds =
            (uint8_t)(total_seconds % 60U);
    }

}

/**
 * @brief 返回STEP8A固定背光
 *
 * STEP8B再改成从EEPROM系统设置读取。
 */
uint8_t ui_get_back_light_level(void)
{
    return Sys_Get_BacklightLevel();
}

bool ui_get_key_sound_enable(void)
{
    return Sys_Get_KeySoundEnable();
}

uint8_t ui_get_language_select(void)
{
    return Sys_Get_LanguageSelect();
}

uint16_t ui_get_display_rotation(void)
{
    return Sys_Get_Rotation();
}

uint8_t ui_get_current_range_mode(void)
{
    return Sys_Get_CurrentRangeMode();
}

void ui_set_back_light_level(uint8_t level)
{
    if (!Sys_Set_BacklightLevel(level))
    {
        return;
    }

    if (level == 0U)
    {
        LCD_Close_Light();
    }
    else
    {
        LCD_Set_Light(level);
    }
}

void ui_set_key_sound_enable(bool enable)
{
    (void)Sys_Set_KeySoundEnable(enable);
}

void ui_set_language_select(uint8_t language)
{
    (void)Sys_Set_LanguageSelect(language);
}

void ui_set_display_rotation(uint16_t rotation)
{
    if (Sys_Set_Rotation(rotation))
    {
        LCD_SetRotation(rotation);
    }
}

void ui_set_current_range_mode(uint8_t mode)
{
    if (Sys_Set_CurrentRangeMode(mode))
    {
        Gate_Set_Mode(mode);
    }
}

bool ui_system_settings_save(void)
{
    return EEPROM_SysSetting_Save();
}

/**
 * @brief 获取主页需要的最新测量数据
 */
void ui_get_vol_cur(float *voltage, float *current)
{
    if (voltage != NULL)
    {
        *voltage = s_ui_voltage_v;
    }

    if (current != NULL)
    {
        *current = s_ui_current_ua;
    }
}

/**
 * @brief 更新主页测量数据缓存
 */
void ui_update_vol_cur_varables(float voltage,
                                float current)
{
    s_ui_voltage_v = voltage;
    s_ui_current_ua = current;
}

/**
 * @brief 向PD任务发送一条完整命令
 */
static bool ui_pd_send_command(PD_command_t event,
                               float voltage,
                               float current,
                               PD_FIXED_VOL_LEVEL level)
{
    PD_command_msg_t msg;

    if (PD_cmd_MessageQueue == NULL)
    {
        return false;
    }

    /*
     * 每个字段都初始化，避免把栈中的随机值发送给PD任务。
     */
    msg.event = event;
    msg.pps_set_voltage = voltage;
    msg.pps_set_current = current;
    msg.pd_fixed_level = level;

    return osMessageQueuePut(PD_cmd_MessageQueue,
                             &msg,
                             0U,
                             0U) == osOK;
}

bool ui_send_pdsink_start_msg(void)
{
    return ui_pd_send_command(PD_CMD_START,
                              0.0f,
                              0.0f,
                              PD_FIXED_VOL_LEVEL_5V);
}

bool ui_send_pdsink_stop_msg(void)
{
    return ui_pd_send_command(PD_CMD_STOP,
                              0.0f,
                              0.0f,
                              PD_FIXED_VOL_LEVEL_5V);
}

bool ui_send_pps_set_msg(float voltage, float current)
{
    return ui_pd_send_command(PD_CMD_SET_PPS,
                              voltage,
                              current,
                              PD_FIXED_VOL_LEVEL_5V);
}

bool ui_send_pd_fixed_set_msg(uint8_t level)
{
    if (level > PD_FIXED_VOL_LEVEL_20V)
    {
        return false;
    }

    return ui_pd_send_command(
        PD_CMD_SET_PD_FIXED,
        0.0f,
        0.0f,
        (PD_FIXED_VOL_LEVEL)level);
}

int8_t MsgQueueGet_PD_ready(void)
{
    PD_handle_event_t result;

    if (PD_handle_event_MsgQueue == NULL)
    {
        return 0;
    }

    if (osMessageQueueGet(PD_handle_event_MsgQueue,
                          &result,
                          NULL,
                          0U) != osOK)
    {
        return 0;
    }

    switch (result)
    {
        case PD_EVT_PPS_READY:
            return 1;

        case PD_EVT_FIXED_READY:
            return 2;

        case PD_EVT_PPS_FAILED:
            return -1;

        default:
            return 0;
    }
}

