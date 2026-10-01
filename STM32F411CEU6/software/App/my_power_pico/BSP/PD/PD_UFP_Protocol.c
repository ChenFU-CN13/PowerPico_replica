
/**
 * PD_UFP_Protocol.c
 *
 *  Updated on: Jan 25, 2021
 *      Author: Ryan Ma
 *
 * Minimalist USB PD implement with only UFP(device) sink only functionality
 * Requires PD PHY to do automatic GoodCRC response on valid SOP messages.
 * Requires only stdint.h, stdbool.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 * Support PD3.0 PPS
 * Do not support extended message. Not necessary for PD trigger and PPS.
 * 
 * Reference: USB_PD_R2_0 V1.3 - 20170112
 *            USB_PD_R3_0 V2.0 20190829 + ECNs 2020-12-10
 *            - Chapter 6. Protocol Layer
 *
 */

/*
 * 中文阅读说明：
 * 本文件只做 PD“协议与策略”，不直接读写 FUSB302 寄存器。
 *
 * 收包主线：PD_protocol_handle_msg()
 *   -> 解析 16 位消息头
 *   -> 在 ctrl/data/ext 三张表中找到 handler
 *   -> Source_Capabilities 解析并选 PDO/APDO
 *   -> 向上层产生 SRC_CAP、ACCEPT、REJECT、PS_RDY 等事件。
 *
 * 发包主线：PD_protocol_create_request()
 *   -> responder_source_cap()
 *   -> 把当前选中的 PDO/APDO 编成一个 RDO
 *   -> 上层再交给 FUSB302_UFP.c 写入 TX FIFO。
 *
 * 复刻重点不是背消息编号，而是能讲清：能力列表如何保存、如何选档、
 * 固定 PDO 与 PPS APDO 的 RDO 字段有什么不同，以及 MessageID 何时递增。
 */
 
#include <string.h>
#include "PD_UFP_Protocol.h"

#define PD_SPECIFICATION_REVISION           0x2

#define PD_CONTROL_MSG_TYPE_ACCEPT          0x3
#define PD_CONTROL_MSG_TYPE_REJECT          0x4
#define PD_CONTROL_MSG_TYPE_GET_SRC_CAP     0x7
#define PD_CONTROL_MSG_TYPE_NOT_SUPPORT     0x10
#define PD_CONTROL_MSG_TYPE_GET_PPS_STATUS  0x14

#define PD_DATA_MSG_TYPE_REQUEST            0x2
#define PD_DATA_MSG_TYPE_SINK_CAP           0x4
#define PD_DATA_MSG_TYPE_VENDOR_DEFINED     0xF

#define PD_EXT_MSG_TYPE_SINK_CAP_EXT        0xF

typedef struct {
    uint8_t type;       // 5 位消息类型。
    uint8_t spec_rev;   // 2 位 PD 规范版本字段。
    uint8_t id;         // 3 位 MessageID，范围 0~7。
    uint8_t num_of_obj; // 后续 32 位数据对象数量，范围 0~7。
} PD_msg_header_info_t;

typedef struct {
    uint16_t limit;     // 选档上限；具体比较量由下面两个开关决定。
    uint8_t use_voltage;// 为 1 时把电压纳入比较。
    uint8_t use_current;// 为 1 时把电流纳入比较。
} PD_power_option_setting_t;

struct PD_msg_state_t {
    const char * name; // 调试显示用的短名称。
    void (*handler)(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events); // 收到该消息时执行。
    bool (*responder)(PD_protocol_t * p, uint16_t * header, uint32_t * obj); // 需要回复时负责组包。
};

/* Optimize RAM usage on AVR MCU by allocate const in PROGMEM */
#if defined(__AVR__)
#include <avr/pgmspace.h>
#define SET_MSG_STAGE(d, s) do { static struct PD_msg_state_t m; memcpy_P(&m, s, sizeof(struct PD_msg_state_t)); d = &m; } while (0)
#define SET_MSG_NAME(d, s)  do { static char n[16]; strncpy_P(n, s, 15); d = n; } while (0)
#define COPY_PDO(d, s)      do { memcpy_P(&d, &s, 4); } while (0)
#else
#define PROGMEM
#define SET_MSG_STAGE(d, s) do { d = s; } while (0)
#define SET_MSG_NAME(d, s)  do { d = s; } while (0)
#define COPY_PDO(d, s)      do { d = s; } while (0)
#endif

#define T(name) static const char str_ ## name [] PROGMEM = #name

static void handler_good_crc   (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_goto_min   (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_accept     (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_reject     (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_ps_rdy     (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_source_cap (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_BIST       (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_alert      (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_vender_def (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_PPS_Status (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);

static bool responder_get_sink_cap  (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_reject        (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_soft_reset    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_source_cap    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_vender_def    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_sink_cap_ext  (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_not_support   (PD_protocol_t * p, uint16_t * header, uint32_t * obj);

T(C0); T(GoodCRC); T(GotoMin); T(Accept); T(Reject); T(Ping); T(PS_RDY); T(Get_Src_Cap);
T(Get_Sink_Cap); T(DR_Swap); T(PR_Swap); T(VCONN_Swap); T(Wait); T(Soft_Rst); T(Dat_Rst); T(Dat_Rst_Cpt);
T(NS); T(Get_Src_Ext); T(Get_Stat); T(FR_Swap); T(Get_PPS_Stat); T(Get_CC); T(Get_Sink_Ext); T(C_R);

// 控制消息分发表：没有 Data Object，按消息 type 直接索引处理函数和回复函数。
static const struct PD_msg_state_t ctrl_msg_list[] PROGMEM = {
    {.name = str_C0,            .handler = 0,                   .responder = 0},
    {.name = str_GoodCRC,       .handler = handler_good_crc,    .responder = 0},
    {.name = str_GotoMin,       .handler = handler_goto_min,    .responder = 0},
    {.name = str_Accept,        .handler = handler_accept,      .responder = 0},
    {.name = str_Reject,        .handler = handler_reject,      .responder = 0},
    {.name = str_Ping,          .handler = 0,                   .responder = 0},
    {.name = str_PS_RDY,        .handler = handler_ps_rdy,      .responder = 0},
    {.name = str_Get_Src_Cap,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Sink_Cap,  .handler = 0,                   .responder = responder_get_sink_cap},
    {.name = str_DR_Swap,       .handler = 0,                   .responder = responder_reject},
    {.name = str_PR_Swap,       .handler = 0,                   .responder = responder_not_support},
    {.name = str_VCONN_Swap,    .handler = 0,                   .responder = responder_reject},
    {.name = str_Wait,          .handler = 0,                   .responder = 0},
    {.name = str_Soft_Rst,      .handler = 0,                   .responder = responder_soft_reset},
    {.name = str_Dat_Rst,       .handler = 0,                   .responder = 0},
    {.name = str_Dat_Rst_Cpt,   .handler = 0,                   .responder = 0},
    
    {.name = str_NS,            .handler = 0,                   .responder = 0},
    {.name = str_Get_Src_Ext,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Stat,      .handler = 0,                   .responder = responder_not_support},
    {.name = str_FR_Swap,       .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_PPS_Stat,  .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_CC,        .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Sink_Ext,  .handler = 0,                   .responder = responder_sink_cap_ext},
    {.name = str_C_R,           .handler = 0,                   .responder = responder_not_support},
};

T(D0); T(Src_Cap); T(Request); T(BIST); T(Sink_Cap); T(Bat_Stat); T(Alert); T(Get_CI);
T(Enter_USB); T(D9); T(D10); T(D11); T(D12); T(D13); T(D14); T(VDM);
T(D_R); 

// 数据消息分发表：带 1~7 个 32 位 Data Object；本项目最关键的是 Source_Capabilities。
static const struct PD_msg_state_t data_msg_list[] PROGMEM = {
    {.name = str_D0,            .handler = 0,                   .responder = 0},
    {.name = str_Src_Cap,       .handler = handler_source_cap,  .responder = responder_source_cap},
    {.name = str_Request,       .handler = 0,                   .responder = responder_not_support},
    {.name = str_BIST,          .handler = handler_BIST,        .responder = 0},
    {.name = str_Sink_Cap,      .handler = 0,                   .responder = responder_not_support},
    {.name = str_Bat_Stat,      .handler = 0,                   .responder = responder_not_support},
    {.name = str_Alert,         .handler = handler_alert,       .responder = 0},
    {.name = str_Get_CI,        .handler = 0,                   .responder = responder_not_support},
    {.name = str_Enter_USB,     .handler = 0,                   .responder = 0},
    {.name = str_D9,            .handler = 0,                   .responder = 0},
    {.name = str_D10,           .handler = 0,                   .responder = 0},
    {.name = str_D11,           .handler = 0,                   .responder = 0},
    {.name = str_D12,           .handler = 0,                   .responder = 0},
    {.name = str_D13,           .handler = 0,                   .responder = 0},
    {.name = str_D14,           .handler = 0,                   .responder = 0},
    {.name = str_VDM,           .handler = handler_vender_def,  .responder = responder_vender_def},

    {.name = str_D_R,           .handler = 0,                   .responder = responder_not_support},
};

T(E0); T(Src_Cap_Ext); T(Status); T(Get_Bat_cap); T(Get_Bat_Stat); T(Bat_Cap); T(Get_Mfg_Info); T(Mfg_Info);
T(Sec_Request); T(Sec_Response); T(FU_request); T(FU_Response); T(PPS_Stat); T(Country_Info); T(Country_Code); T(Sink_Cap_Ext);
T(E_R);

// 扩展消息分发表：项目只取少量 PPS 状态字段，其余大多回复 Not_Supported。
static const struct PD_msg_state_t ext_msg_list[] PROGMEM = {
    {.name = str_E0,            .handler = 0,                   .responder = responder_not_support},
    {.name = str_Src_Cap_Ext,   .handler = 0,                   .responder = 0},
    {.name = str_Status,        .handler = 0,                   .responder = 0},
    {.name = str_Get_Bat_cap,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Bat_Stat,  .handler = 0,                   .responder = responder_not_support},
    {.name = str_Bat_Cap,       .handler = 0,                   .responder = 0},
    {.name = str_Get_Mfg_Info,  .handler = 0,                   .responder = responder_not_support},
    {.name = str_Mfg_Info,      .handler = 0,                   .responder = 0},
    {.name = str_Sec_Request,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Sec_Response,  .handler = 0,                   .responder = 0},
    {.name = str_FU_request,    .handler = 0,                   .responder = responder_not_support},
    {.name = str_FU_Response,   .handler = 0,                   .responder = 0},
    {.name = str_PPS_Stat,      .handler = handler_PPS_Status,  .responder = 0},
    {.name = str_Country_Info,  .handler = 0,                   .responder = 0},
    {.name = str_Country_Code,  .handler = 0,                   .responder = 0},
    {.name = str_Sink_Cap_Ext,  .handler = 0,                   .responder = responder_not_support},

    {.name = str_E_R,           .handler = 0,                   .responder = responder_not_support},
};

// 固定档不是“必须请求到该电压”，而是扫描能力表时选择不超过 limit 的最后一个候选项。
static const PD_power_option_setting_t power_option_setting[8] = {
    {.limit = 25,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_5V */
    {.limit = 45,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_9V */
    {.limit = 60,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_12V */
    {.limit = 75,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_15V */
    {.limit = 100,  .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_20V */
    {.limit = 100,  .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_VOLTAGE */
    {.limit = 125,  .use_voltage = 0, .use_current = 1},    /* PD_POWER_OPTION_MAX_CURRENT */
    {.limit = 12500,.use_voltage = 1, .use_current = 1},    /* PD_POWER_OPTION_MAX_POWER */  
};

/**
 * @brief 在 Source_Capabilities 中选择一个固定 PDO 或满足目标值的 PPS APDO。
 * @param p PD 协议上下文，里面保存能力列表和选档策略。
 * @param PPS_voltage PPS 目标电压，使用协议 RDO 的 20 mV 单位。
 * @param PPS_current PPS 目标电流，使用协议 RDO 的 50 mA 单位。
 * @return 选中的能力对象下标；0 也是合法的第一个 5 V PDO。
 *
 * 本函数是 PD 选档核心，逐行注释如下。固定档会保留“不超过上限”的最后一个候选，
 * PPS 则找到第一个范围覆盖目标电压且电流能力足够的 APDO 后立即返回。
 */
static uint8_t evaluate_src_cap(PD_protocol_t * p, uint16_t PPS_voltage, uint8_t PPS_current)
{
    const PD_power_option_setting_t * setting; // 当前固定档策略对应的比较配置。
    PD_power_info_t info;                      // 每轮解码一个 PDO/APDO 的临时结构。
    uint8_t option = p->power_option;          // UI 选择的 5/9/12/15/20 V 等策略枚举。
    uint8_t selected = 0;                      // 默认回退到能力表第 0 项，通常为 5 V 固定 PDO。

    /* If selected option is not available, use first PDO. Reference: 6.4.1 Capabilities Message
       The vSafe5V Fixed Supply Object Shall always be the first object. */
    if (option >= sizeof(power_option_setting) / sizeof(power_option_setting[0])) { // 防止策略数组越界。
        return 0; // 非法策略直接选第一个 PDO。
    }

    setting = &power_option_setting[option]; // 取得本次扫描使用的上限和比较方式。
    for (uint8_t n = 0; PD_protocol_get_power_info(p, n, &info); n++) { // 逐项解码，越过数量后函数返回 false。
        if (info.type == PD_PDO_TYPE_AUGMENTED_PDO) { // APDO 分支：检查目标 PPS 值是否落在能力范围内。
            uint16_t pps_v = PPS_voltage * 2;    /* 20 mV 输入换成 10 mV 中间单位，便于和 APDO 电压范围比较。 */
            uint16_t pps_i = PPS_current * 5;    /* 50 mA 输入换成 10 mA 中间单位，便于和 APDO 电流能力比较。 */
            /* PD_power_info_t: Voltage in 50mV units, Current in 10mA units */
            if (info.min_v * 5 <= pps_v && pps_v <= info.max_v * 5 && pps_i <= info.max_i) { // 电压在范围内且电流不超能力。
                return n; // 找到满足条件的 APDO，立即使用该项。
            }
        } else {
            uint8_t v = setting->use_voltage ? info.max_v >> 2 : 1; // 需要比较电压时压缩数值，否则乘数取 1。
            uint8_t i = setting->use_current ? info.max_i >> 2 : 1; // 需要比较电流时压缩数值，否则乘数取 1。
            uint16_t power = (uint16_t)v * i; // 统一形成“电压/电流/功率”比较值，具体含义由策略决定。
            if (power <= setting->limit) {    // 当前能力没有超过 UI 选择的上限。
                selected = n;                 // 保留本项，继续扫描可能更高但仍不超限的能力。
            }
        }
    }
    return selected; // 没有可用 PPS 时也会落到固定 PDO 候选，默认至少为第 0 项。
}

/** @brief 拆出 PD 16 位消息头中本协议层关心的字段。 */
static void parse_header(PD_msg_header_info_t * info, uint16_t header)
{
    /* Reference: 6.2.1.1 Message Header */ 
    info->type = (header >> 0) & 0x1F;                  /*   4...0  Message Type */
    info->spec_rev = (header >> 6) & 0x3;               /*   7...6  Specification Revision */
    info->id = (header >> 9) & 0x7;                     /*  11...9  MessageID */
    info->num_of_obj = (header >> 12) & 0x7;            /* 14...12  Number of Data Objects */
}

/** @brief 用当前 MessageID、消息类型和对象数量生成 16 位普通消息头。 */
static uint16_t generate_header(PD_protocol_t * p, uint8_t type, uint8_t obj_count)
{
    /* Reference: 6.2.1.1 Message Header */ 
    uint16_t h = ((uint16_t)type << 0) |                      /*   4...0  Message Type */
                 ((uint16_t)PD_SPECIFICATION_REVISION << 6) | /*   7...6  Specification Revision */
                 ((uint16_t)p->message_id << 9) |             /*  11...9  MessageID */
                 ((uint16_t)obj_count << 12);                 /* 14...12  Number of Data Objects */
    p->tx_msg_header = h; // 保存最近一次发送头，供调试查询。
    return h;
}

static uint16_t generate_header_ext(PD_protocol_t * p, uint8_t type, uint8_t data_size, uint32_t * obj)
{
    uint16_t h = generate_header(p, type, (data_size + 5) >> 2); /* set obj_count to fit ext header and data */
    h |= (uint16_t)1 << 15;     /* Set extended field */
    /* Reference: 6.2.1.2 Extended Message Headerr */ 
    obj[0] |= ((uint16_t)data_size << 0) |  /*   8...0  ata Size */
              /* Assume short message, set Chunk Number and Request Chunk to 0 */
              ((uint16_t)1 << 15);          /*      15  Chunked */
    p->tx_msg_header = h;
    return h;
}

/** @brief 收到对端 GoodCRC 后，把本端下一条发送消息的 MessageID 加 1，并在 0~7 回绕。 */
static void handler_good_crc(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    /* Reference: 6.2.1.3 Message ID 
       MessageIDCounter Shall be initialized to zero at power-on / reset, increment when receive GoodCRC Message */
    uint8_t message_id = p->message_id;
    if (++message_id > 7) {
        message_id = 0;
    }
    p->message_id = message_id;
}

static void handler_goto_min(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // Not implemented
}

static void handler_accept(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    if (events) {
        *events |= PD_PROTOCOL_EVENT_ACCEPT;
    }
}

static void handler_reject(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    if (events) {
        // 注意：函数名是 Reject，但当前源码置位的是 PS_RDY 事件，不是 REJECT。
        // 这会让上层走“供电已就绪”分支；复刻时应把它列为重点核验项，确认后再决定是否修正。
        *events |= PD_PROTOCOL_EVENT_PS_RDY;
    }
}

static void handler_ps_rdy(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    if (events) {
        *events |= PD_PROTOCOL_EVENT_PS_RDY;
    }
}

/** @brief 保存 Source_Capabilities 的全部 PDO/APDO，并按当前策略选择一个目标。 */
static void handler_source_cap(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    PD_msg_header_info_t h;                 // 保存拆出的消息头字段。
    parse_header(&h, header);               // 取得本帧数据对象数量。
    p->power_data_obj_count = h.num_of_obj; // 记录能力表有效项数。
    for (uint8_t i = 0; i < h.num_of_obj; i++) { // 逐项保存原始 32 位 PDO/APDO。
        p->power_data_obj[i] = obj[i];       // 后续查询和重选档位都使用这份缓存。
    }
    p->power_data_obj_selected = evaluate_src_cap(p, p->PPS_voltage, p->PPS_current); // 计算首次请求目标。
    if (events) {
        *events |= PD_PROTOCOL_EVENT_SRC_CAP; // 通知应用层能力列表已经可用。
    }
}

static void handler_BIST(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // TODO: implement BIST
}

static void handler_alert(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // TODO: implement alert
}

static void handler_vender_def(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // TODO: implement VDM parsing
}

static void handler_PPS_Status(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    /* Handle chunked Extended message,  Offset 2 byte for Extended Message Header */
    p->PPSSDB[0] = (obj[0] >> 16) & 0xFF;
    p->PPSSDB[1] = (obj[0] >> 24) & 0xFF;
    p->PPSSDB[2] = (obj[1] >>  0) & 0xFF;
    p->PPSSDB[3] = (obj[1] >>  8) & 0xFF;
    if (events) {
        *events |= PD_PROTOCOL_EVENT_PPS_STATUS;
    }
}

static bool responder_get_sink_cap(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    /* Reference: 6.4.1.2.3 Sink Fixed Supply Power Data Object */
    uint32_t data = ((uint32_t)100 << 0) |                        /* B9...0     Operational Current in 10mA units */
                    ((uint32_t)100 << 10) |                       /* B19...10   Voltage in 50mV units */
                    ((uint32_t)1 << 26) |                         /* B26        USB Communications Capable */
                    ((uint32_t)1 << 28) |                         /* B28        Higher Capability */
                    ((uint32_t)PD_PDO_TYPE_FIXED_SUPPLY << 30);   /* B31...30   Fixed supply */
    *obj = data; /* Only implement 5V 1A Fix supply PDO. Source rarely request sink cap */
    *header = generate_header(p, PD_DATA_MSG_TYPE_SINK_CAP, 1);
    return true;
}

static bool responder_sink_cap_ext(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    /* Reference: 6.5.13 Sink_Capabilities_Extended Message 
                  6.12.3 Applicability of Extended Messages  (Normative; Shall be supported) */
    #define SINK_CAP_VID                0
    #define SINK_CAP_PID                0
    #define SINK_CAP_XID                0       /* If the vendor does not have an XID, then it Shall return zero */
    #define SINK_CAP_FW_Version         1
    #define SINK_CAP_HW_Version         1
    #define SINK_CAP_SKEDB_Version      1
    #define SINK_CAP_SINK_MODE          0x3     /* Bit 0: PPS charging supported, Bit 1: VBUS powered */
    #define SINK_CAP_SINK_MIN_PDP       5       /* Minimum     PD Power in Watt */
    #define SINK_CAP_SINK_OP_PDP        5       /* Operational PD Power in Watt */
    #define SINK_CAP_SINK_MAX_PDP       100     /* Maximum     PD Power in Watt */
    static const uint32_t SKEDB[6] PROGMEM = { /* 2-byte header + 21-byte data, chunked to 6 PDO */
    /* PDO[0], data byte  0...1  */
        /* 16-bit LSB is reserved for Extended Message Header */
        ((uint32_t)SINK_CAP_VID << 16),             /* Byte  0...1  VID */
    /* PDO[1], data byte  2...5  */
        ((uint32_t)SINK_CAP_PID << 0) |             /* Byte  2...3  PID */
        (((uint32_t)SINK_CAP_XID & 0xFF) << 16),    /* Byte  4...5  XID */
    /* PDO[2], data byte  6...9  */
        (((uint32_t)SINK_CAP_XID >> 16) << 0) |     /* Byte  6...7  XID */
        ((uint32_t)SINK_CAP_FW_Version << 16) |     /* Byte      8  FW Version */
        ((uint32_t)SINK_CAP_HW_Version << 24),      /* Byte      9  HW Version */
    /* PDO[3], data byte 10...13 */
        ((uint32_t)SINK_CAP_SKEDB_Version << 0),    /* Byte     10  SKEDB Version */
        /* Not set Byte 11 Load Step, Byte 13..12 Sink Load Characteristics */
    /* PDO[4], data byte 14...17 */
        /* Not set Byte 14 Compliance, Byte 15 Touch Temp, Byte 16 Battery Info */
        ((uint32_t)SINK_CAP_SINK_MODE << 24),       /* Byte     17  Sink Modes */
    /* PDO[5], data byte 18...20 */
        ((uint32_t)SINK_CAP_SINK_MIN_PDP << 0) |    /* Byte     18  Minimum PDP */
        ((uint32_t)SINK_CAP_SINK_OP_PDP << 8) |     /* Byte     19  Operational PDP */
        ((uint32_t)SINK_CAP_SINK_MAX_PDP << 16)     /* Byte     20  Maximum PDP */
    };
    uint8_t i;
    for (i = 0; i < 6; i++) {
        COPY_PDO(obj[i], SKEDB[i]);
    }
    *header = generate_header_ext(p, PD_EXT_MSG_TYPE_SINK_CAP_EXT, 21, obj);
    return false;
}

static bool responder_reject(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_REJECT, 0);
    return true;
}

static bool responder_not_support(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_NOT_SUPPORT, 0);
    return true;
}

static bool responder_soft_reset(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_ACCEPT, 0);
    return true;
}

/**
 * @brief 根据已选 PDO/APDO 生成一个 Request Data Object 和对应消息头。
 * @return true，表示已经生成需要发送的回复。
 */
static bool responder_source_cap(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    PD_power_info_t info; // 解码后的所选电源能力。
    uint32_t data, pos = p->power_data_obj_selected + 1; // RDO 对象位置从 1 开始，数组下标从 0 开始。
    PD_protocol_get_power_info(p, p->power_data_obj_selected, &info); // 取得类型、电压、电流/功率字段。
    /* Reference: 6.4.2 Request Message */
    if (info.type == PD_PDO_TYPE_AUGMENTED_PDO) { // PPS APDO 使用电压 20 mV、电流 50 mA 的 RDO 字段。
        /* NOTE: To compatible PD2.0 PHY, do not set Unchunked Extended Messages Supported */
        data = ((uint32_t)p->PPS_current << 0) |    /* B6 ...0    Operating Current 50mA units */
               ((uint32_t)p->PPS_voltage << 9) |    /* B19...9    Output Voltage in 20mV units */
               ((uint32_t)1 << 25) |                /* B25        USB Communication Capable */
               ((uint32_t)pos << 28);               /* B30...28   Object position (000b is Reserved and Shall Not be used) */
    } else {
        uint32_t req = info.max_i ? info.max_i : info.max_p; // 固定/可变源用电流；电池 PDO 没电流时改用功率。
        data = ((uint32_t)req << 0) |    /* B9 ...0    Max Operating Current 10mA units / Max Operating Power in 250mW units */
               ((uint32_t)req << 10) |   /* B19...10   Operating Current 10mA units / Operating Power in 250mW units */
               ((uint32_t)1 << 25) |     /* B25        USB Communication Capable */
               ((uint32_t)pos << 28);    /* B30...28   Object position (000b is Reserved and Shall Not be used) */
    }
    *obj = data; // 输出 1 个 32 位 Request Data Object。
    *header = generate_header(p, PD_DATA_MSG_TYPE_REQUEST, 1); // 生成“Request + 1 个对象”的消息头。
    return true;
}

static bool responder_vender_def(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    // TODO: implement VDM respond
    return false;
}

/**
 * @brief 按消息头把收到的 PD 包分派给扩展、数据或控制消息处理函数。
 */
void PD_protocol_handle_msg(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    /* ===== 三张消息分发表的"最大合法下标"宏 =====
     *
     * 背景：PD 消息按"扩展位(bit15) + 有无数据对象"被分到三张表：
     *   ext_msg_list   扩展消息表（bit15=1）
     *   data_msg_list  数据消息表（bit15=0 且 num_of_obj != 0）
     *   ctrl_msg_list  控制消息表（bit15=0 且 num_of_obj == 0）
     *
     * 每张表的布局相同：
     *   下标 0~15  → 与 PD 协议规定的 5bit 消息 type(0~15) 一一对应
     *   下标 16    → 协议保留(Reserved)消息的统一兜底项 C_R/D_R/E_R
     *                （该项 handler=0、responder=Not_Supported，收到未识别消息统一拒绝）
     *
     * sizeof(表)/sizeof(表[0]) = 数组总项数 = 17
     * 再 -1 得到 16，即"兜底项下标"，作为查表时允许的最大下标 LIMIT。
     *
     * 配合下方三元表达式 h.type > LIMIT ? LIMIT : h.type 使用：
     *   - h.type 为 0~16 ：正常按下标查表
     *   - h.type 为 17~31（type 字段共 5bit，可能出现越界的保留值）：
     *     钳位到 16，走 Reserved 兜底项，杜绝数组越界访问。
     */
    #define EXT_MSG_LIMIT   (sizeof(ext_msg_list) / sizeof(ext_msg_list[0]) - 1)  // 扩展表最大下标=16(兜底项)
    #define DATA_MSG_LIMIT  (sizeof(data_msg_list) / sizeof(data_msg_list[0]) - 1) // 数据表最大下标=16(兜底项)
    #define CTRL_MSG_LIMIT  (sizeof(ctrl_msg_list) / sizeof(ctrl_msg_list[0]) - 1) // 控制表最大下标=16(兜底项)

    const struct PD_msg_state_t * state; // 本帧最终匹配到的分发表项。
    PD_msg_header_info_t h;              // 拆出的 type、版本、MessageID 和对象数。
    parse_header(&h, header);            // 先解析固定头。
    p->rx_msg_header = header;           // 保存最近一次接收头，供调试查询。
    if ((header >> 15) & 0x1) {          // bit15=1 表示扩展消息。
        state = &ext_msg_list[h.type > EXT_MSG_LIMIT ? EXT_MSG_LIMIT : h.type];
    } else if (h.num_of_obj) {           // 普通消息且对象数不为 0，属于数据消息。
        state = &data_msg_list[h.type > DATA_MSG_LIMIT ? DATA_MSG_LIMIT : h.type];
    } else {                             // 没有数据对象，属于控制消息。
        state =&ctrl_msg_list[h.type > CTRL_MSG_LIMIT ? CTRL_MSG_LIMIT : h.type];
    }
    SET_MSG_STAGE(p->msg_state, state);   // STM32 直接保存表项指针；AVR 兼容分支会从 PROGMEM 复制。
    if (p->msg_state->handler) {         // 不是所有 PD 消息都需要本项目处理。
        p->msg_state->handler(p, header, obj, events); // 执行具体解析并向应用层置事件位。
    }
}

bool PD_protocol_respond(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    if (p && p->msg_state && p->msg_state->responder && header && obj) {
        return p->msg_state->responder(p, (uint16_t *)header, obj);
    }
    return false;
}

void PD_protocol_create_get_src_cap(PD_protocol_t * p, uint16_t * header)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_GET_SRC_CAP, 0);
}

void PD_protocol_create_get_PPS_status(PD_protocol_t *p, uint16_t *header)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_GET_PPS_STATUS, 0);
}

void PD_protocol_create_request(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    responder_source_cap(p, header, obj);
}

/**
 * @brief 把一个原始 32 位 PDO/APDO 解码为统一的电压、电流和功率结构。
 * @return true：下标有效且解码完成；false：参数或下标无效。
 */
bool PD_protocol_get_power_info(PD_protocol_t * p, uint8_t index, PD_power_info_t * power_info)
{
    if (p && index < p->power_data_obj_count && power_info) {
        uint32_t obj = p->power_data_obj[index]; // 取出 Source_Capabilities 中对应的原始对象。
        power_info->type = obj >> 30;            // 最高 2 位决定 Fixed/Battery/Variable/Augmented。
        switch (power_info->type) {
        case PD_PDO_TYPE_FIXED_SUPPLY:
            /* Reference: 6.4.1.2.3 Source Fixed Supply Power Data Object */
            power_info->min_v = 0;
            power_info->max_v = (obj >> 10) & 0x3FF;    /*  B19...10  Voltage in 50mV units */
            power_info->max_i = (obj >>  0) & 0x3FF;    /*  B9 ...0   Max Current in 10mA units */
            power_info->max_p = 0;
            break;
        case PD_PDO_TYPE_BATTERY:
            /* Reference: 6.4.1.2.5 Battery Supply Power Data Object */
            power_info->min_v = (obj >> 10) & 0x3FF;    /*  B19...10  Min Voltage in 50mV units */
            power_info->max_v = (obj >> 20) & 0x3FF;    /*  B29...20  Max Voltage in 50mV units */
            power_info->max_i = 0;
            power_info->max_p = (obj >>  0) & 0x3FF;    /*  B9 ...0   Max Allowable Power in 250mW units */
            break;
        case PD_PDO_TYPE_VARIABLE_SUPPLY:
            /* Reference: 6.4.1.2.4 Variable Supply (non-Battery) Power Data Object */
            power_info->min_v = (obj >> 10) & 0x3FF;    /*  B19...10  Min Voltage in 50mV units */
            power_info->max_v = (obj >> 20) & 0x3FF;    /*  B29...20  Max Voltage in 50mV units */
            power_info->max_i = (obj >>  0) & 0x3FF;    /*  B9 ...0   Max Current in 10mA units */
            power_info->max_p = 0;
            break;
        case PD_PDO_TYPE_AUGMENTED_PDO:
            /* Reference: 6.4.1.3.4 Programmable Power Supply Augmented Power Data Object */
            power_info->max_v = ((obj >> 17) & 0xFF) * 2;   /* 字段单位 100 mV，乘 2 后统一成结构体的 50 mV 单位。 */
            power_info->min_v = ((obj >>  8) & 0xFF) * 2;   /* 字段单位 100 mV，乘 2 后统一成结构体的 50 mV 单位。 */
            power_info->max_i = ((obj >>  0) & 0x7F) * 5;   /* 字段单位 50 mA，乘 5 后统一成结构体的 10 mA 单位。 */
            power_info->max_p = 0;
            break;
        }
        return true;
    }
    return false;
}

bool PD_protocol_get_msg_info(uint16_t header, PD_msg_info_t * msg_info)
{
    PD_msg_header_info_t h;
    parse_header(&h, header);
    if (msg_info) {
        const char * name;
        const struct PD_msg_state_t * state;
        uint8_t type = h.type;
        SET_MSG_STAGE(state, header & 0x8000 ? &ext_msg_list[type] :
                        h.num_of_obj ? &data_msg_list[type] : &ctrl_msg_list[type]);
        SET_MSG_NAME(name, state->name);
        msg_info->name = name;
        msg_info->id = h.id;
        msg_info->spec_rev = h.spec_rev;
        msg_info->num_of_obj = h.num_of_obj;
        msg_info->extended = header >> 15;
        return true;
    }
    return false;
}

bool PD_protocol_get_PPS_status(PD_protocol_t *p, PPS_status_t * PPS_status)
{
    if (p && PPS_status) {
        /* Reference: 6.5.10 PPS_Status Message */
        PPS_status->output_voltage = ((uint16_t)p->PPSSDB[1] << 8) | p->PPSSDB[0];
        PPS_status->output_current = p->PPSSDB[2];
        PPS_status->flag_PTF = (p->PPSSDB[3] >> 1) & 0x3;   /* Bit 1 ... 2 */
        PPS_status->flag_OMF = (p->PPSSDB[3] >> 3) & 0x1;   /* Bit 3 */
        return true;
    }
    return false;
}

/** @brief 切到固定 PDO 选档策略；已有能力表时立即重选并要求上层重发 Request。 */
bool PD_protocol_set_power_option(PD_protocol_t * p, enum PD_power_option_t option)
{
    p->power_option = option; // 保存 5/9/12/15/20 V 等策略。
    p->PPS_voltage = 0;       // 清零 PPS 目标，明确切回固定档选择。
    p->PPS_current = 0;
    if (p->power_data_obj_count > 0) {
        p->power_data_obj_selected = evaluate_src_cap(p, p->PPS_voltage, p->PPS_current);
        return true;    /* need to re-send request */
    }
    return false;
}

bool PD_protocol_select_power(PD_protocol_t * p, uint8_t index)
{
    if (index < p->power_data_obj_count) {
        p->power_data_obj_selected = index;
        return true;    /* need to re-send request */
    }
    return false;
}

/**
  * @brief  设置 PPS（Programmable Power Supply）目标电压/电流，并从充电器广播的
  *         能力表中挑选一个能覆盖该目标的 PPS APDO（Augmented PDO）。
  *
  * @param  p           : PD 协议实例指针，内部保存着最近收到的 Source_Capabilities
  *                       能力表（power_data_obj[]）及当前选中对象下标。
  * @param  PPS_voltage : 目标电压，单位 = 20 mV/步（PPS RDO 协议规定的单位）。
  *                       例如请求 5.00 V 应传入 250；请求 9.00 V 传入 450。
  *                       上层一般用宏 PPS_V(5.0) 把伏特换算成该单位。
  * @param  PPS_current : 目标电流，单位 = 50 mA/步（PPS RDO 协议规定的单位）。
  *                       例如请求 1.0 A 应传入 20；请求 3.0 A 传入 60。
  *                       上层一般用宏 PPS_A(1.0) 把安培换算成该单位。
  * @param  strict      : 匹配策略开关。
  *                       - true  : 能力表中找不到能覆盖目标的 APDO 时，拒绝本次更新，
  *                                 保留旧目标（用于用户手动调电压时的合法性校验）。
  *                       - false : 即使找不到（selected=0，回退到第 0 项 5V Fixed PDO），
  *                                 也照样保存新目标值（用于刚 START、能力表还没到时预填，
  *                                 等 Source_Capabilities 到达后协议层会重新匹配）。
  *
  * @retval true  : 目标值发生了变化，且已更新选中对象，调用方需要重发一次 Request
  *                 消息让充电器按新参数调整输出（对应 PDUFPTask 里的 send_power_request()）。
  * @retval false : 新旧目标完全相同，或 strict=true 且无匹配 APDO，无需重发。
  *
  * @note   执行过程：
  *           1) 比较新旧目标，完全相同则直接返回 false，避免无意义的重复请求；
  *           2) 调 evaluate_src_cap() 扫描能力表：逐项解码 PDO/APDO，找到第一个
  *              "电压范围覆盖目标 且 电流能力足够"的 APDO，返回其下标；找不到返回 0；
  *           3) strict=false 或确实找到匹配项时，把目标值与选中下标写入协议实例；
  *           4) 返回 true 通知上层"参数变了，该发 Request 了"。
  */
bool PD_protocol_set_PPS(PD_protocol_t * p, uint16_t PPS_voltage, uint8_t PPS_current, bool strict)
{
    if (p->PPS_voltage != PPS_voltage || p->PPS_current != PPS_current) { // 值没变就不重复申请。
        uint8_t selected = evaluate_src_cap(p, PPS_voltage, PPS_current); // 查找匹配 APDO，下标 0 同时承担“没找到”含义。
        if (selected || !strict) {             // strict=false 时即使没找到也接受新目标并使用回退项。
            p->PPS_voltage = PPS_voltage;      // 保存目标电压（单位 20mV/LSB），后续组 PPS RDO 的 B19~B9 字段时直接使用。
            p->PPS_current = PPS_current;      // 保存目标电流（单位 50mA/LSB），后续组 PPS RDO 的 B8~B0 字段时直接使用。
            p->power_data_obj_selected = selected; // 保存后续 RDO 使用的对象下标（Object Position B30~B28 要填它+1）。
            return true;    /* need to re-send request */            
        }
    }
    return false;
}

/** @brief 软复位消息处理状态和 MessageID，保留已配置的选档参数与能力缓存。 */
void PD_protocol_reset(PD_protocol_t * p)
{
    p->msg_state = &ctrl_msg_list[0];
    p->message_id = 0;
}

/** @brief 首次初始化协议上下文，清空能力表、PPS 目标、消息 ID 和状态。 */
void PD_protocol_init(PD_protocol_t * p)
{
    memset(p, 0, sizeof(PD_protocol_t));
    p->msg_state = &ctrl_msg_list[0];
}
