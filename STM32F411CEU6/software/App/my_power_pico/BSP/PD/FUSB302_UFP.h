
/**
  ******************************************************************************
  * @file    FUSB302_UFP.h
  * @brief   FUSB302 PD 协议 PHY 驱动 —— 头文件（设备/外设侧 UFP 实现）
  *
  * 【文件作用】
  *   定义 FUSB302 芯片的对外接口、状态枚举、事件枚举、设备结构体。
  *   只实现 UFP(USB Device/Sink 侧) 功能，不实现 DFP(Source) 侧。
  *   驱动上层（PD_UFP_Protocol.c）通过本头文件调用底层硬件操作。
  *
  * 【设计要点】
  *   - 只依赖 stdint.h / string.h，跨平台移植性强
  *   - 不使用位域(bit-field)，避免不同编译器的位域布局差异
  *   - FUSB302 支持 PD3.0，但有限制（见下方说明）
  *
  * 【PD3.0 限制说明】
  *   - FIFO 不足以装下 unchunked(未分片) 报文，必须用 chunked(分片) 报文
  *   - VBUS 检测低阈值是 4V，若请求 PPS 电压低于 4V 应禁用 vbus_sense
  *
  *  Updated on: Jan 4, 2021
  *      Author: Ryan Ma
  ******************************************************************************
  */

#ifndef FUSB302_UFP_H
#define FUSB302_UFP_H

#include <stdint.h>

/* FUSB302 函数返回值枚举（位掩码，便于一次返回多个错误状态）
 *  SUCCESS=0           : 成功
 *  BUSY                 : 设备忙（如正在发送/接收）
 *  ERR_PARAM            : 参数错误
 *  ERR_DEVICE_ID        : 器件 ID 不匹配（不是 FUSB302）
 *  ERR_READ_DEVICE      : I2C 读设备失败
 *  ERR_WRITE_DEVICE     : I2C 写设备失败
 */
enum {
    FUSB302_SUCCESS             = 0,
    FUSB302_BUSY                = (1 << 0),
    FUSB302_ERR_PARAM           = (1 << 1),
    FUSB302_ERR_DEVICE_ID       = (1 << 2),
    FUSB302_ERR_READ_DEVICE     = (1 << 3),
    FUSB302_ERR_WRITE_DEVICE    = (1 << 4)
};
typedef uint8_t FUSB302_ret_t;   // 返回值类型（8 位足以容纳 5 个错误位）

/* FUSB302 事件标志枚举（位掩码，由 FUSB302_alert 返回）
 *  EVENT_ATTACHED       : 检测到对端连接（CC 上拉/下拉被识别）
 *  EVENT_DETACHED       : 对端断开
 *  EVENT_RX_SOP         : 收到 SOP 报文（需要调用 FUSB302_get_message 读取）
 *  EVENT_GOOD_CRC_SENT  : GoodCRC 已发送（自动回复对端）
 */
#define FUSB302_EVENT_ATTACHED          (1 << 0)
#define FUSB302_EVENT_DETACHED          (1 << 1)
#define FUSB302_EVENT_RX_SOP            (1 << 2)
#define FUSB302_EVENT_GOOD_CRC_SENT     (1 << 3)
typedef uint8_t FUSB302_event_t;

/* FUSB302 设备结构体 —— 每个外挂 FUSB302 对应一个实例
 *  使用方法：
 *    1) 由用户填入 i2c_address / i2c_read / i2c_write / delay_ms（HAL 回调）
 *    2) 调用 FUSB302_init() 后，库内部使用 err_msg / rx_* / reg_* / state 等字段
 */
typedef struct {
    /* ===== 用户必须初始化的字段（移植接口） ===== */
    uint8_t i2c_address;   // FUSB302 的 I2C 设备地址（7 位地址，不含 R/W 位）
    // I2C 读回调：从 dev_addr 器件的 reg_addr 寄存器读 count 字节到 data
    FUSB302_ret_t (*i2c_read)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count);
    // I2C 写回调：向 dev_addr 器件的 reg_addr 寄存器写 count 字节 data
    FUSB302_ret_t (*i2c_write)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count);
    // 毫秒级延时回调（用于等待复位、防抖等）
    FUSB302_ret_t (*delay_ms)(uint32_t t);

    /* ===== 库内部使用的字段（用户无需手动维护） ===== */
    const char * err_msg;       // 最近一次错误信息字符串（便于调试）
    uint16_t rx_header;         // 最近收到的 PD 报文头（含类型/ID/数据对象数）
    uint8_t rx_buffer[32];     // 最近收到的 PD 报文数据（最多 4 个 32 位对象 = 16 字节，留余量）
    uint8_t reg_control[15];   // 控制类寄存器缓存（0x01~0x0F 共 15 字节，REG_xxx 宏映射）
    uint8_t reg_status[7];      // 状态类寄存器缓存（0x3C~0x42 共 7 字节，REG_xxx 宏映射）

    uint8_t interrupta;         // 中断状态 A'（I_TXSENT/I_HARDRST 等）的缓存
    uint8_t interruptb;         // 中断状态 B（I_GCRCSENT）的缓存
    uint8_t cc1;                // CC1 引脚电压等级（0~3，对应 BC_LVL_* 区间）
    uint8_t cc2;                // CC2 引脚电压等级
    uint8_t state;              // 当前连接状态（FUSB302_STATE_UNATTACHED/ATTACHED）
    uint8_t vbus_sense;         // VBUS 检测使能标志（请求 PPS<4V 时应置 0）
} FUSB302_dev_t;

/* 获取最近一次错误信息字符串（调试用，inline 避免函数调用开销） */
static inline const char * FUSB302_get_last_err_msg(FUSB302_dev_t *dev) { return dev->err_msg; }

/* ===== 对外 API（上层 PD_UFP_Protocol.c 调用） ===== */

// 初始化 FUSB302：上电、复位、读 ID 校验、配置寄存器默认值
FUSB302_ret_t FUSB302_init            (FUSB302_dev_t *dev);

// PD 逻辑复位：仅复位 PD 状态机，保留寄存器配置
FUSB302_ret_t FUSB302_pd_reset        (FUSB302_dev_t *dev);

// 清除一次连接会话的软件缓存，保留芯片寄存器配置和板级回调
FUSB302_ret_t FUSB302_session_reset   (FUSB302_dev_t *dev);

// 使能/关闭 CC1/CC2 内部下拉电阻（作为 Sink 时使能，作为 Source 或空闲时关闭）
FUSB302_ret_t FUSB302_pdwn_cc         (FUSB302_dev_t *dev, uint8_t enable);

// 使能/关闭 VBUS 检测（请求 PPS 低于 4V 时应关闭，避免误触发 VBUSOK 中断）
FUSB302_ret_t FUSB302_set_vbus_sense  (FUSB302_dev_t *dev, uint8_t enable);

// 读取器件 ID：version_ID=芯片版本号，revision_ID=修订号（用于硬件兼容性判断）
FUSB302_ret_t FUSB302_get_ID          (FUSB302_dev_t *dev, uint8_t *version_ID, uint8_t *revision_ID);

// 读取 CC1/CC2 引脚电压等级（0~3），用于判断接入方向（CC1 还是 CC2 连接）和对端角色
FUSB302_ret_t FUSB302_get_cc          (FUSB302_dev_t *dev, uint8_t *cc1, uint8_t *cc2);

// 读取 VBUS 电压等级（0/1：0=低于 4V，1=高于 4V）
FUSB302_ret_t FUSB302_get_vbus_level  (FUSB302_dev_t *dev, uint8_t *vbus);

// 读取最近收到的 SOP 报文：header=报文头，data=数据对象数组（最多 4 个 uint32_t）
FUSB302_ret_t FUSB302_get_message     (FUSB302_dev_t *dev, uint16_t *header, uint32_t *data);

// 发送 SOP 报文：header=报文头，data=数据对象数组（Request / Get_Source_Cap 等）
FUSB302_ret_t FUSB302_tx_sop          (FUSB302_dev_t *dev, uint16_t header, const uint32_t *data);

// 发送 Hard Reset（硬复位，强制对端回到默认 5V 状态）
FUSB302_ret_t FUSB302_tx_hard_reset   (FUSB302_dev_t *dev);

// 事件轮询接口：读取中断状态并解析为高层事件（ATTACHED/DETACHED/RX_SOP/GOOD_CRC_SENT）
//  通常在主循环或定时器周期调用，是整个驱动的事件入口
FUSB302_ret_t FUSB302_alert           (FUSB302_dev_t *dev, FUSB302_event_t *events);

#endif /* FUSB302_H */
