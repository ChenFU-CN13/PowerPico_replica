/**
 * @file fusb302_dev.c
 * @brief PowerPico 的 PD 应用状态机，衔接 RTOS 任务、FUSB302 驱动和 PD 协议层。
 *
 * 三层关系：
 * - FUSB302_UFP.c：直接读写 FUSB302 寄存器/FIFO，负责 CC、收发包和中断。
 * - PD_UFP_Protocol.c：解析 PDO/APDO，生成 Request，维护 PD 消息 ID。
 * - 本文件：处理连接/超时/重试，把协议成功结果写进 app_pd。
 *
 * 主调用链：PDUFPTask -> fusb302_timer()/FUSB302_alert()
 *                     -> handle_FUSB302_event() -> PD_protocol_handle_msg()。
 */
#include "fusb302_dev.h"
#include "i2c.h"
#include "cmsis_os2.h"

#define FUSB302_USE_OS 1

#if FUSB302_USE_OS == 1
    #define clock_ms() osKernelGetTickCount()
    #define delay_ms(ms) osDelay(ms)
#else
    #define clock_ms() HAL_GetTick()
    #define delay_ms(ms) HAL_Delay(ms)
#endif



#define PD_POLLING              100  // 即使 INT_N 没拉低，也每 100 ms 主动读一次事件。
#define TYPEC_SINK_WAIT_CAP     350  // 等待 Source_Capabilities 的超时时间，单位 ms。
#define REQUEST_TO_PS_READY     580  // 发送 Request 后等待 PS_RDY 的超时时间，单位 ms。
#define PPS_REQUEST             5000 // PPS 模式每 5 s 重发一次 Request，维持可编程供电。

FUSB302_dev_t fusb302_dev;
App_PD_t app_pd;

FUSB302_ret_t fusb302_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, count, 10); // 通过 I2C1 读 8 位寄存器地址。
    return FUSB302_SUCCESS; // 当前源码没有把 HAL 返回值上传，I2C 故障也会被当作成功；复刻时应重点验证。
}

FUSB302_ret_t fusb302_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, count, 10); // 通过 I2C1 写寄存器。
    return FUSB302_SUCCESS; // 与读函数一样，当前实现忽略 HAL 超时/NACK 状态。
}

FUSB302_ret_t fusb302_delay_ms_wrapper(uint32_t ms)
{
#if FUSB302_USE_OS == 1
    osDelay(ms);
#else
    HAL_Delay(ms);
#endif
    return FUSB302_SUCCESS;
}

void status_power_ready(status_power_t status, uint16_t voltage, uint16_t current)
{
    app_pd.ready_voltage = voltage; // 注意：当前源码固定 PDO 传 50 mV 单位，PPS 传 20 mV 单位，读取方必须结合 status 解释。
    app_pd.ready_current = current; // 注意：固定 PDO 传 10 mA 单位，PPS 传 50 mA 单位。
    app_pd.status_power = status;   // 最后更新状态，供查询函数判断供电类型。
}

void set_default_power(void)
{
    status_power_ready(STATUS_POWER_TYP, PD_V(5.0), PD_A(1.0)); // 超时或普通 Type-C 电流时回到 5 V/1 A 软件默认值。
}

/**
 * @brief 消费协议层产生的 Source_Cap、Reject、PS_RDY 事件。
 * @param events 可同时包含多个事件的位图。
 */
void handle_protocol_event(PD_protocol_event_t events)
{
    if (events & PD_PROTOCOL_EVENT_SRC_CAP) {
        app_pd.wait_src_cap = 0;           // 已拿到能力列表，不再发送 Get_Source_Cap。
        app_pd.get_src_cap_retry_count = 0;// 清掉能力查询重试次数。
        app_pd.wait_ps_rdy = 1;            // 协议层已经选好 PDO/APDO，后续等待供电就绪。
        app_pd.time_wait_ps_rdy = clock_ms(); // 记录超时起点。
        app_pd.state = PD_STATE_GET_CAPS; // 更新状态：已收到能力，可以请求了
    }
    if (events & PD_PROTOCOL_EVENT_REJECT) {
        if (app_pd.wait_ps_rdy) {          // 只把当前请求等待期间的 Reject 当作本轮失败。
            app_pd.wait_ps_rdy = 0;
            app_pd.state = PD_STATE_FAILED; // 更新状态：请求被拒绝
        }
    }
    if (events & PD_PROTOCOL_EVENT_PS_RDY) {
        PD_power_info_t p;
        uint8_t i, selected_power = PD_protocol_get_selected_power(&app_pd.protocol); // 取得所选 PDO/APDO 下标。
        PD_protocol_get_power_info(&app_pd.protocol, selected_power, &p);              // 解码所选电源能力。
        app_pd.wait_ps_rdy = 0;
        if (p.type == PD_PDO_TYPE_AUGMENTED_PDO) { // APDO 表示进入 PPS 可编程供电。
            // PPS mode
            FUSB302_set_vbus_sense(&fusb302_dev, 0); // 低于 4 V 的 PPS 场景关闭芯片固定阈值 VBUS 检测。
            if (app_pd.PPS_voltage_next) {
                // Two stage startup for PPS voltage < 5V
                PD_protocol_set_PPS(&app_pd.protocol, app_pd.PPS_voltage_next, app_pd.PPS_current_next, false); // 切到第二阶段目标值。
                app_pd.PPS_voltage_next = 0; // 清除待切换标记，防止重复执行。
                app_pd.send_request = 1;     // 让定时器循环再发一次 Request。
            } else {
                app_pd.time_PPS_request = clock_ms(); // PPS 保活计时从协商成功开始。
                status_power_ready(STATUS_POWER_PPS,
                PD_protocol_get_PPS_voltage(&app_pd.protocol), PD_protocol_get_PPS_current(&app_pd.protocol));
                app_pd.state = PD_STATE_SUCCESS_PPS; // 更新状态：成功协商到PPS
            }
        } else {
            FUSB302_set_vbus_sense(&fusb302_dev, 1); // 固定 PDO 恢复 VBUS 检测。
            status_power_ready(STATUS_POWER_TYP, p.max_v, p.max_i);
            app_pd.state = PD_STATE_SUCCESS_FIXED; // 更新状态：协商到了一个固定电压
        }
    }
}

/**
 * @brief 把 FUSB302 硬件事件转换成协议层输入和应用状态变化。
 * @param events FUSB302_alert() 汇总出的事件位图。
 */
void handle_FUSB302_event(FUSB302_event_t events)
{
    if (events & FUSB302_EVENT_DETACHED) {
        PD_protocol_reset(&app_pd.protocol); // 断线后清消息 ID 和当前消息处理状态。
        app_pd.cc = 0;                       // 0 表示当前没有有效 CC 方向。
        return;
    }
    if (events & FUSB302_EVENT_ATTACHED) {
        uint8_t cc1 = 0, cc2 = 0, cc = 0;
        FUSB302_get_cc(&fusb302_dev, &cc1, &cc2); // 读取 CC1/CC2 检测结果，判断插头方向和对端能力。
        PD_protocol_reset(&app_pd.protocol);      // 新连接从消息 ID 0 开始。
        if (cc1 && cc2 == 0) {
            cc = cc1;
            app_pd.cc = 1;
        } else if (cc2 && cc1 == 0) {
            cc = cc2;
            app_pd.cc = 2;
        }
        /* TODO: handle no cc detected error */
        if (cc > 1) {                // 检测值大于默认 USB 电流，按源码进入等待 PD 能力流程。
            app_pd.wait_src_cap = 1; // 超时后 fusb302_timer() 会主动发 Get_Source_Cap。
        } else {
            set_default_power();     // 没进入 PD 消息流程时按普通 5 V 供电处理。
        }
    }
    if (events & FUSB302_EVENT_RX_SOP) {
        PD_protocol_event_t protocol_event = 0;
        uint16_t header;
        uint32_t obj[7];
        FUSB302_get_message(&fusb302_dev, &header, obj); // 从 RX FIFO 取 PD 头和最多 7 个数据对象。
        PD_protocol_handle_msg(&app_pd.protocol, header, obj, &protocol_event); // 按消息类型分派。
        if (protocol_event) {
            handle_protocol_event(protocol_event);
        }
    }
    if (events & FUSB302_EVENT_GOOD_CRC_SENT) {
        uint16_t header;
        uint32_t obj[7];
        delay_ms(4);  // 给重复消息/硬件自动 GoodCRC 留处理时间，再决定是否回复。
        if (PD_protocol_respond(&app_pd.protocol, &header, obj)) {
            FUSB302_tx_sop(&fusb302_dev, header, obj);
        }
    }
}

/**
 * @brief 处理等待能力、等待 PS_RDY、PPS 保活和兜底轮询计时。
 * @return true：本轮需要读取一次 FUSB302 中断寄存器；false：暂时不用。
 */
bool fusb302_timer(void)
{
    uint32_t t = clock_ms(); // 使用无符号减法判断超时，可自然跨越 tick 回绕。
    if (app_pd.wait_src_cap && t - app_pd.time_wait_src_cap > TYPEC_SINK_WAIT_CAP) {
        app_pd.time_wait_src_cap = t;
        if (app_pd.get_src_cap_retry_count < 3) { // 前 3 次只发 Get_Source_Cap，不主动断 VBUS。
            uint16_t header;
            app_pd.get_src_cap_retry_count += 1;
            /* Try to request soruce capabilities message (will not cause power cycle VBUS) */
            PD_protocol_create_get_src_cap(&app_pd.protocol, &header);
            FUSB302_tx_sop(&fusb302_dev, header, 0);
        } else {
            app_pd.get_src_cap_retry_count = 0;
            /* Hard reset will cause the source power cycle VBUS. */
            FUSB302_tx_hard_reset(&fusb302_dev); // 连续拿不到能力后发送 Hard Reset，可能让 Source 重启 VBUS。
            PD_protocol_reset(&app_pd.protocol); // 协议消息 ID 回到初始值。
        }
    }
    if (app_pd.wait_ps_rdy) {
        if (t - app_pd.time_wait_ps_rdy > REQUEST_TO_PS_READY) {
            app_pd.wait_ps_rdy = 0; // 本轮请求超时，不再继续等。
            set_default_power();    // 软件状态退回普通 5 V，当前源码未在这里设置 FAILED。
        }
    } else if (app_pd.send_request || (app_pd.status_power == STATUS_POWER_PPS && t - app_pd.time_PPS_request > PPS_REQUEST)) {
        app_pd.wait_ps_rdy = 1; // 发出 Request 后等待 Source 返回 PS_RDY。
        app_pd.send_request = 0;// 消费一次性发送请求标志。
        app_pd.state = PD_STATE_NEGOTIATING; // 更新状态：正在协商中
        app_pd.time_PPS_request = t;
        uint16_t header;
        uint32_t obj[7];
        /* Send request if option updated or regularly in PPS mode to keep power alive */
        PD_protocol_create_request(&app_pd.protocol, &header, obj); // 根据所选 PDO/APDO 生成 RDO。
        app_pd.time_wait_ps_rdy = clock_ms();                       // 从实际发送前开始计超时。
        FUSB302_tx_sop(&fusb302_dev, header, obj);                  // 通过 TX FIFO 发 SOP 消息。
    }
    if (t - app_pd.time_polling > PD_POLLING) {
        app_pd.time_polling = t;
        return true;
    }
    return false;
}

bool is_power_ready(void) { return app_pd.status_power == STATUS_POWER_TYP; } // 查询当前软件记录是否为普通/固定供电。
bool is_PPS_ready(void)   { return app_pd.state == PD_STATE_SUCCESS_PPS; }    // 查询 PPS 协商状态。
bool is_PD_Fixed_ready(void) {return app_pd.state == PD_STATE_SUCCESS_FIXED;} // 查询固定 PDO 协商状态。
void send_power_request(void) { app_pd.send_request = 1; }                    // 只置位，下一轮 timer 才真正发包。

uint8_t fusb302_dev_init(void) {

    /*
    设置FUSB302 I²C地址0x22
        → 注册I²C读函数
        → 注册I²C写函数
        → 注册延时函数
        → 软件复位FUSB302
        → 配置CC开关和比较器
        → 配置PD自动重试
        → 配置中断掩码
        → 打开FUSB302内部功能模块
        → 读取芯片ID
        → 初始化软件PD协议状态*/
    fusb302_dev.i2c_address = 0x22<<1; // HAL 使用左移后的 8 位地址格式，原始 7 位地址是 0x22。
    fusb302_dev.i2c_read = fusb302_i2c_read;       // 注入当前板子的 I2C 读函数。
    fusb302_dev.i2c_write = fusb302_i2c_write;     // 注入当前板子的 I2C 写函数。
    fusb302_dev.delay_ms = fusb302_delay_ms_wrapper; // 注入 RTOS 兼容延时函数。
    if (FUSB302_init(&fusb302_dev) == FUSB302_SUCCESS && FUSB302_get_ID(&fusb302_dev, 0, 0) == FUSB302_SUCCESS) {
        PD_protocol_init(&app_pd.protocol); // 芯片初始化和 ID 读取都成功后，再初始化协议上下文。
        app_pd.state = PD_STATE_IDLE; // 初始状态
        return 0;
    }
    return 1;
}
