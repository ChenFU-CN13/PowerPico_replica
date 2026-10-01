#ifndef __UI_H__
#define __UI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "lv_lib_pm.h"
#include "lv_lib_animation.h"
#include "ui_helpers.h"
#include "common/lv_i18n.h"

LV_FONT_DECLARE(ui_font_zhongyuan18);       //这个是18号字体
LV_FONT_DECLARE(ui_font_zhongyuan20);       //这个是20号字体
/**
 * @brief 初始化PowerPico界面
 */
void ui_init(void);

/* STEP8A主页使用的两个自定义字体 */
LV_FONT_DECLARE(ui_font_HeiTi32);       //这个是32号字体
LV_FONT_DECLARE(ui_font_HeiTi48);       //这个是48号字体

/* 启动页使用的图片 */
LV_IMAGE_DECLARE(ui_img_chicken96_png); //启动页使用的图片

#ifdef __cplusplus
}
#endif

#endif /* __UI_H__ */
