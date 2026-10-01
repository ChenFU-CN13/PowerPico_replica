

#include "ui.h"
#include "screens/ui_StartPage.h"
#include "screens/ui_mainPage.h"
#include "screens/ui_SetPage.h"
#include "screens/ui_PPSPage.h"
#include "screens/ui_PDFixedPage.h"


/*
 * STEP8A只注册主页。
 * 启动页不属于PageManager循环页面，
 * 它只在上电动画期间显示一次。
 */
static Page_t s_pages[] = 
{
    {
        .init = ui_main_screen_init,
        .deinit = ui_main_screen_destroy,
        .page_obj = &ui_HomeScreen,
        .key_event_handler = ui_main_page_key_handler,
        .name = "Main Page"
    },
    {
        .init = ui_SetPage_screen_init,
        .deinit = ui_SetPage_screen_destroy,
        .page_obj = &ui_SetPage,
        .key_event_handler = ui_set_page_key_handler,
        .name = "Set Page"
    },
    {
        .init = ui_PPSPage_screen_init,
        .deinit = ui_PPSPage_screen_destroy,
        .page_obj = &ui_PPSPage,
        .key_event_handler = ui_pps_page_key_handler,
        .name = "PPS Page"
    },
    {
        .init = ui_PDFixedPage_screen_init,
        .deinit = ui_PDFixedPage_screen_destroy,
        .page_obj = &ui_PDFixedPage,
        .key_event_handler = ui_pdfixed_page_key_handler,
        .name = "PDFixed Page"
    }

};


/**
 * @brief 官方工程的整屏刷新保护
 * 当前先保留源工程行为，保证显示效果一致。
 * 如果后续发现它明显影响USB吞吐，再单独测试是否可以取消。
 */
static void ui_main_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    ui_full_screen_refresh(lv_screen_active());
}

/**
 * @brief 启动动画完成后进入主页
 */
static void ui_start_animation_done(lv_anim_t *animation)
{
    uint8_t page_index;

    (void)animation;

    lv_lib_pm_Init();

    for (page_index = 0U;
         page_index < (sizeof(s_pages) / sizeof(s_pages[0]));
         page_index++)
    {
        lv_lib_pm_register(&s_pages[page_index]); 
    }     
    
    lv_lib_pm_load_init_screen();
}

/**
 * @brief 背光渐亮动画的实际执行函数
 */
static void ui_animation_set_light(void *object,
                                   int32_t light_level)
{

    (void)object;
    ui_set_back_light_level((uint8_t)light_level);

}


/**
 * @brief 启动背光渐亮动画
 */
static void ui_start_animation(void)
{
    uint8_t target_level;

    target_level = ui_get_back_light_level();

    lv_lib_anim_user_animation(
        NULL,
        0U,                         /* 不延迟 */
        2000U,                      /* 动画持续2秒 */
        0,                          /* 从关闭背光开始 */
        target_level,               /* 渐亮到目标亮度 */
        0U,
        0U,
        0U,
        0U,
        lv_anim_path_ease_in,
        ui_animation_set_light,
        ui_start_animation_done);
}

/**
 * @brief 初始化PowerPico启动页和主页
 */
void ui_init(void)
{
    lv_display_t *display;
    lv_theme_t *theme;

    display = lv_display_get_default();

    theme = lv_theme_default_init(
        display,
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_RED),
        true,
        LV_FONT_DEFAULT);



    lv_display_set_theme(display, theme);       //这个是指设置默认主题

    /*
     * 保留官方工程每1秒整屏刷新一次的临时保护。
     * 定时器不需要保存，因为整个运行期间都有效。
     */
    (void)lv_timer_create(ui_main_timer_cb,     // 创建一个定时器，每1秒调用一次ui_main_timer_cb
                          1000U,
                          NULL);

    ui_set_display_rotation(ui_get_display_rotation());

    lv_i18n_init(lv_i18n_language_pack);

    if (ui_get_language_select() == 0U)
    {
        lv_i18n_set_locale("en");
    }
    else
    {
        lv_i18n_set_locale("zh-cn");
    }
    /* 先显示启动页，再由2秒动画切换到主页 */
    ui_StartPage_screen_init();
    lv_screen_load(ui_StartPage);

    ui_start_animation();       // 启动背光渐亮动画
}
