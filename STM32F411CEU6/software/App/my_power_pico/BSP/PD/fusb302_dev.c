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
#include <string.h>

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
#define REQUEST_RETRY_DELAY      20  // Request 写入失败后隔 20 ms 再试，避免 1 ms 循环持续占用 I2C。
#define PPS_REQUEST             5000 // PPS 模式每 5 s 重发一次 Request，维持可编程供电。

FUSB302_dev_t fusb302_dev;
App_PD_t app_pd;

/* I2C 首错误诊断操作类型。 */
#define PD_I2C_DIAG_OP_NONE   0U
#define PD_I2C_DIAG_OP_READ   1U
#define PD_I2C_DIAG_OP_WRITE  2U

/*
 * STEP9修复说明（相对官方基线）：
 * 保留第一次HAL I2C错误现场，便于在没有I2C测试点时通过Keil Watch定位。
 * 该结构只记录，不参与任何协议分支；正常协商时应保持valid=0、error_count=0。
 */
typedef struct
{
    volatile uint8_t valid;          // 1：已经抓到第一次错误
    volatile uint8_t op;             // 1：读；2：写
    volatile uint8_t reg;            // FUSB302寄存器地址
    volatile uint8_t len;            // 本次读写长度

    volatile uint32_t tick_ms;       // 错误发生时间
    volatile uint32_t error_count;   // 本轮累计错误次数
    volatile uint32_t hal_status;    // HAL函数返回值
    volatile uint32_t hal_error;     // hi2c1.ErrorCode快照
} PD_I2C_Diag_t;

volatile PD_I2C_Diag_t g_pd_i2c_diag;

/**
 * @brief 保存本轮第一次 I2C 错误，并累计错误次数。
 * @param op 操作类型：1为读，2为写。
 * @param reg_addr FUSB302 寄存器地址。
 * @param count 本次读写字节数。
 * @param status HAL I2C 接口返回值。
 * @return 无。
 */
static void fusb302_i2c_diag_capture(uint8_t op,
                                     uint8_t reg_addr,
                                     uint8_t count,
                                     HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
    {
        return;
    }

    g_pd_i2c_diag.error_count++;

    /* 后续错误只累计次数，不能覆盖第一次错误现场。 */
    if (g_pd_i2c_diag.valid != 0U)
    {
        return;
    }

    g_pd_i2c_diag.op         = op;
    g_pd_i2c_diag.reg        = reg_addr;
    g_pd_i2c_diag.len        = count;
    g_pd_i2c_diag.tick_ms    = HAL_GetTick();
    g_pd_i2c_diag.hal_status = (uint32_t)status;
    g_pd_i2c_diag.hal_error  = hi2c1.ErrorCode;

    /* 最后置位，表示本次错误现场已经完整保存。 */
    g_pd_i2c_diag.valid = 1U;
}

/**
 * @brief 清除上一轮 PD 协商留下的 I2C 诊断数据。
 * @return 无。
 */
void fusb302_i2c_diag_reset(void)
{
    g_pd_i2c_diag.valid       = 0U;
    g_pd_i2c_diag.op          = PD_I2C_DIAG_OP_NONE;
    g_pd_i2c_diag.reg         = 0U;
    g_pd_i2c_diag.len         = 0U;
    g_pd_i2c_diag.tick_ms     = 0U;
    g_pd_i2c_diag.error_count = 0U;
    g_pd_i2c_diag.hal_status  = (uint32_t)HAL_OK;
    g_pd_i2c_diag.hal_error   = HAL_I2C_ERROR_NONE;
}

FUSB302_ret_t fusb302_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1,
                              dev_addr,
                              reg_addr,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              count,
                              10);

    fusb302_i2c_diag_capture(PD_I2C_DIAG_OP_READ,
                             reg_addr,
                             count,
                             status);

    /*
     * STEP9修复说明（相对官方基线）：
     * 官方封装忽略HAL返回值并固定报成功，失败时上层会继续使用旧寄存器缓存。
     * 这里必须把真实读错误交给状态机，禁止把一次失败的I2C事务当成有效数据。
     */
    if (status != HAL_OK)
    {
        return FUSB302_ERR_READ_DEVICE;
    }

    return FUSB302_SUCCESS;
}

FUSB302_ret_t fusb302_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hi2c1,
                               dev_addr,
                               reg_addr,
                               I2C_MEMADD_SIZE_8BIT,
                               data,
                               count,
                               10);

    fusb302_i2c_diag_capture(PD_I2C_DIAG_OP_WRITE,
                             reg_addr,
                             count,
                             status);

    /* STEP9修复说明：只有HAL实际写成功，协议层才能认为报文已经送入FUSB302。 */
    if (status != HAL_OK)
    {
        return FUSB302_ERR_WRITE_DEVICE;
    }

    return FUSB302_SUCCESS;
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

/**
 * @brief 清除上一轮 PD 会话状态，下一轮重新从 CC 检测开始。
 * @return 无。
 * @note 只清软件状态，不重置 FUSB302 芯片，也不改已经初始化好的寄存器配置缓存。
 * @note STEP9修复说明：官方START只重置protocol子结构，外层等待标志、CC方向和
 *       FUSB302的ATTACHED状态可能残留，导致第二次START无法重新产生ATTACHED事件。
 */
void fusb302_session_reset(void)
{
    memset(&app_pd, 0, sizeof(app_pd));
    PD_protocol_init(&app_pd.protocol);
    app_pd.state = PD_STATE_IDLE;
    (void)FUSB302_session_reset(&fusb302_dev);
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
  * @brief  FUSB302 硬件事件 → PD 协议层输入 / 应用状态的总转换函数。
  *
  * @param  events : FUSB302_alert() 汇总出的事件位图，可能同时包含多个事件，
  *                  取值为 FUSB302_EVENT_ATTACHED / DETACHED / RX_SOP /
  *                  GOOD_CRC_SENT 的任意按位或组合。
  * @retval 无。
  *
  * @note   本函数是"PHY 硬件层"与"PD 协议层"之间的桥梁，由 PDUFPTask 在每次
  *         FUSB302_alert() 读出非零事件后调用。内部按事件类型分 4 段处理，
  *         各段之间用独立 if（非 else-if），允许一轮处理多个并存事件：
  *
  *   1) DETACHED（CC 断开）：
  *        - PD_protocol_reset() 清消息 ID 和消息处理状态；
  *        - app_pd.cc 清 0 表示无有效插入方向；
  *        - 直接 return，断开后不再处理本轮其他事件。
  *
  *   2) ATTACHED（检测到 Source 插入）：
  *        - 读 CC1/CC2 的 BC_LVL 电平等级判断插入方向（只可能单侧非 0），
  *          记录 app_pd.cc = 1(正向) / 2(反向)；
  *        - CC 等级含义：0=未连接，1=Rp 默认电流(500/900mA)，2=1.5A，3=3A；
  *        - cc > 1：对端是具备 1.5A/3A 能力的快充源，置 wait_src_cap=1，
  *          后续由 fusb302_timer() 超时主动发 Get_Source_Cap 索取能力表；
  *        - cc <= 1：普通 USB 口或未检测到，按默认 5V 供电处理，不发起 PD 协商。
  *
  *   3) RX_SOP（收到一个完整的 SOP PD 报文）：
  *        - FUSB302_get_message() 从 RX FIFO 取出消息头 + 最多 7 个数据对象；
  *        - PD_protocol_handle_msg() 按消息类型分派解析（Source_Cap/Accept/
  *          PS_RDY/Reject...），并把产生的协议层事件填入 protocol_event；
  *        - 有协议事件再交 handle_protocol_event() 更新应用状态
  *          （如协商成功置 PD_STATE_SUCCESS_PPS/FIXED）。
  *
  *   4) GOOD_CRC_SENT（本芯片已自动发出 GoodCRC 应答）：
  *        - 说明刚收到的报文已被 PHY 确认，此时该由协议层决定是否回业务消息；
  *        - delay_ms(4) 给重复消息检测/硬件自动应答留出处理时间；
  *        - PD_protocol_respond() 按当前消息状态生成回应（Request/Reject/
  *          Sink_Cap 等），需要回复时用 FUSB302_tx_sop() 发出，不需要则不动作。
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
            if (FUSB302_tx_sop(&fusb302_dev, header, obj) != FUSB302_SUCCESS &&
                app_pd.wait_ps_rdy) {
                /*
                 * STEP9修复说明：Source_Cap后的自动Request若未真正写入FIFO，
                 * 不能继续假装等待PS_RDY；退回send_request，由定时流程延时重发。
                 */
                app_pd.wait_ps_rdy = 0;
                app_pd.send_request = 1;
                app_pd.time_wait_ps_rdy = clock_ms();
            }
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
            /* Try to request soruce capabilities message (will not cause power cycle VBUS) */
            PD_protocol_create_get_src_cap(&app_pd.protocol, &header);
            if (FUSB302_tx_sop(&fusb302_dev, header, 0) == FUSB302_SUCCESS) {
                app_pd.get_src_cap_retry_count += 1;
            }
        } else {
            /* Hard reset will cause the source power cycle VBUS. */
            if (FUSB302_tx_hard_reset(&fusb302_dev) == FUSB302_SUCCESS) {
                app_pd.get_src_cap_retry_count = 0;
                PD_protocol_reset(&app_pd.protocol); // 协议消息 ID 回到初始值。
            }
        }
    }
    if (app_pd.wait_ps_rdy) {
        if (t - app_pd.time_wait_ps_rdy > REQUEST_TO_PS_READY) {
            app_pd.wait_ps_rdy = 0; // 本轮请求超时，不再继续等。
            set_default_power();    // 软件状态退回普通 5 V，当前源码未在这里设置 FAILED。
        }
    } else if ((app_pd.send_request && t - app_pd.time_wait_ps_rdy > REQUEST_RETRY_DELAY) ||
               (app_pd.status_power == STATUS_POWER_PPS && t - app_pd.time_PPS_request > PPS_REQUEST)) {
        uint16_t header;
        uint32_t obj[7];
        /* Send request if option updated or regularly in PPS mode to keep power alive */
        PD_protocol_create_request(&app_pd.protocol, &header, obj); // 根据所选 PDO/APDO 生成 RDO。
        /*
         * STEP9修复说明（解决“UI选中9V但VBUS仍为5V”）：
         * 只有Request成功写入FUSB302后，才消费send_request并进入wait_ps_rdy。
         * 若发送失败则保留请求，20 ms后再试，避免软件状态先于真实硬件发送状态推进。
         */
        if (FUSB302_tx_sop(&fusb302_dev, header, obj) == FUSB302_SUCCESS) {
            app_pd.wait_ps_rdy = 1; // Request 确实写入 FUSB302 后才开始等待 PS_RDY。
            app_pd.send_request = 0;// 成功后才消费一次性发送标志。
            app_pd.state = PD_STATE_NEGOTIATING;
            app_pd.time_wait_ps_rdy = clock_ms();
            app_pd.time_PPS_request = t;
        } else {
            app_pd.send_request = 1;       // 保留请求，20 ms 后重试。
            app_pd.time_wait_ps_rdy = t;
        }
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
        fusb302_session_reset(); // 芯片初始化成功后，把应用层和连接状态放到同一初始点。
        return 0;
    }
    return 1;
}
