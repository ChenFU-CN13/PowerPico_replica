#ifndef __UI_HELPERS_H__
#define __UI_HELPERS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 强制刷新整个屏幕
 * @param screen 需要刷新的屏幕对象
 */
void ui_full_screen_refresh(lv_obj_t *screen);

/**
 * @brief 获取设备运行时间
 * @param hours 小时输出地址
 * @param minutes 分钟输出地址
 * @param seconds 秒输出地址
 */
void ui_GetElapsedTime_HMS(uint8_t *hours,
                           uint8_t *minutes,
                           uint8_t *seconds);

/**
 * @brief 获取STEP8A固定背光亮度
 * @return 背光百分比
 */
uint8_t ui_get_back_light_level(void);

/**
 * @brief 设置LCD背光亮度
 * @param level 亮度百分比，0表示关闭
 */
void ui_set_back_light_level(uint8_t level);

/**
 * @brief 读取UI当前使用的电压和电流
 * @param voltage 电压输出地址，单位V
 * @param current 电流输出地址，单位uA
 */
void ui_get_vol_cur(float *voltage, float *current);

/**
 * @brief 更新UI测量数据缓存
 * @param voltage 电压，单位V
 * @param current 电流，单位uA
 */
void ui_update_vol_cur_varables(float voltage,
                                float current);


bool ui_get_key_sound_enable(void);                     //获取按键音效是否开启
uint8_t ui_get_language_select(void);                   //获取语言选择
uint16_t ui_get_display_rotation(void);                 //获取显示旋转角度
uint8_t ui_get_current_range_mode(void);                //获取当前范围模式

void ui_set_key_sound_enable(bool enable);              //设置按键音效是否开启
void ui_set_language_select(uint8_t language);          //设置语言选择
void ui_set_display_rotation(uint16_t rotation);        //设置显示旋转角度
void ui_set_current_range_mode(uint8_t mode);           //设置当前范围模式

bool ui_system_settings_save(void);                     //保存系统设置

/**
 * @brief 通知PD任务开始协商
 * @return true发送成功，false队列不可用或已满
 */
bool ui_send_pdsink_start_msg(void);

/**
 * @brief 通知PD任务停止协商并断开CC
 * @return true发送成功，false发送失败
 */
bool ui_send_pdsink_stop_msg(void);

/**
 * @brief 设置PPS目标值
 * @param voltage 电压，单位V
 * @param current 电流，单位A
 * @return true发送成功，false发送失败
 */
bool ui_send_pps_set_msg(float voltage, float current);

/**
 * @brief 切换固定PDO档位
 * @param level 0/1/2/3/4分别对应5/9/12/15/20V
 * @return true发送成功，false发送失败
 */
bool ui_send_pd_fixed_set_msg(uint8_t level);

/**
 * @brief 非阻塞读取PD初次协商结果
 * @return 0无结果，1为PPS成功，2为固定PDO成功，-1为失败
 */
int8_t MsgQueueGet_PD_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* __UI_HELPERS_H__ */
