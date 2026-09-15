
/**
 * FUSB302_UFP.c
 *
 *  Updated on: Jan 4, 2021
 *      Author: Ryan Ma
 *
 * Minimalist USB PD implement with only UFP(device) functionality
 * Requires only stdint.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 * FUSB302 can support PD3.0 with limitations and workarounds
 * - Do not have enough FIFO for unchunked message, use chunked message instead
 * - VBUS sense low threshold at 4V, disable vbus_sense if request PPS below 4V
 *
 */

/*
 * 中文阅读说明：
 * 本文件是 FUSB302B 的“寄存器/FIFO 驱动层”，不负责决定要请求几伏。
 * - FUSB302_state_unattached()：检测 VBUS 和 CC1/CC2，确认插头方向。
 * - FUSB302_state_attached()：读取中断、判断断开、取出 RX FIFO 数据。
 * - FUSB302_tx_sop()：把 PD 消息头和 Data Object 包装成 FUSB302 TX FIFO token。
 * - FUSB302_alert()：按当前 attached/unattached 状态分派处理函数。
 *
 * 寄存器位定义已按 FUSB302B 官方 datasheet 的 Register Definitions 表核对。
 * 复刻时建议先观察 I2C 读写和 INT_N，再进入上层 PDO/APDO 策略调试。
 */

#include <string.h>
#include "FUSB302_UFP.h"

/* Switches0 : 02h */
#define PU_EN2          (0x01 << 7)
#define PU_EN1          (0x01 << 6)
#define VCONN_CC2       (0x01 << 5)
#define VCONN_CC1       (0x01 << 4)
#define MEAS_CC2        (0x01 << 3)
#define MEAS_CC1        (0x01 << 2)
#define PDWN2           (0x01 << 1)
#define PDWN1           (0x01 << 0)

/* Switches1 : 03h */
#define POWERROLE       (0x01 << 7)
#define SPECREV1        (0x01 << 6)
#define SPECREV0        (0x01 << 5)
#define DATAROLE        (0x01 << 4)
#define AUTO_CRC        (0x01 << 2)
#define TXCC2           (0x01 << 1)
#define TXCC1           (0x01 << 0)

/* Measure : 04h */
#define MEAS_VBUS       (0x01 << 6)

/* Control0 : 06h */
#define TX_FLUSH        (0x01 << 6)
#define INT_MASK        (0x01 << 5)
#define HOST_CUR_MASK   (0x03 << 2)
#define HOST_CUR_3A0    (0x03 << 2)
#define HOST_CUR_1A5    (0x02 << 2)
#define HOST_CUR_USB    (0x01 << 2)
#define AUTO_PRE        (0x01 << 1)
#define TX_START        (0x01 << 0)

/* Control1 : 07h */
#define ENSOP2DB        (0x01 << 6)
#define ENSOP1DB        (0x01 << 5)
#define BIST_MODE2      (0x01 << 4)
#define RX_FLUSH        (0x01 << 2)
#define ENSOP2          (0x01 << 1)
#define ENSOP1          (0x01 << 0)

/* Control2 : 08h */
#define WAKE_EN         (0x01 << 3)
#define MODE_MASK       (0x03 << 1)
#define MODE_DFP        (0x03 << 1)
#define MODE_UFP        (0x02 << 1)
#define MODE_DRP        (0x01 << 1)
#define TOGGLE          (0x01 << 0)

/* Control3 : 09h */
#define SEND_HARDRESET  (0x01 << 6)
#define BIST_TMODE      (0x01 << 5)     /* on FUSB302B only */
#define AUTO_HARDRESET  (0x01 << 4)
#define AUTO_SOFTRESET  (0x01 << 3)
#define N_RETRIES_MASK  (0x03 << 1)
#define N_RETRIES(n)    ((n)  << 1)
#define AUTO_RETRY      (0x01 << 0)

/* Mask : 0Ah */
#define M_VBUSOK        (0x01 << 7)
#define M_ACTIVITY      (0x01 << 6)
#define M_COMP_CHNG     (0x01 << 5)
#define M_CRC_CHK       (0x01 << 4)
#define M_ALERT         (0x01 << 3)
#define M_WAKE          (0x01 << 2)
#define M_COLLISION     (0x01 << 1)
#define M_BC_LVL        (0x01 << 0)

/* Power : 0Bh */
#define PWR_INT_OSC     (0x01 << 3) 	/* Enable internal oscillator */
#define PWR_MEASURE     (0x01 << 2) 	/* Measure block powered */
#define PWR_RECEIVER    (0x01 << 1) 	/* Receiver powered and current reference for Measure block */
#define PWR_BANDGAP     (0x01 << 0) 	/* Bandgap and wake circuitry */

/* Reset : 0Ch */
#define PD_RESET        (0x01 << 1)
#define SW_RES          (0x01 << 0)

/* Maska : 0Eh */
#define M_OCP_TEMP      (0x01 << 7)
#define M_TOGDONE       (0x01 << 6)
#define M_SOFTFAIL      (0x01 << 5)
#define M_RETRYFAIL     (0x01 << 4)
#define M_HARDSENT      (0x01 << 3)
#define M_TXSENT        (0x01 << 2)
#define M_SOFTRST       (0x01 << 1)
#define M_HARDRST       (0x01 << 0)

/* Maskb : 0Fh */
#define M_GCRCSENT      (0x01 << 0)

/* Status0a : 3Ch */
#define SOFTFAIL        (0x01 << 5)
#define RETRYFAIL       (0x01 << 4)
#define POWER3_2        (0x01 << 2)
#define SOFTRST         (0x01 << 1)
#define HARDRST         (0x01 << 0)

/* Status1a : 3Dh */
#define TOGSS_MASK      (0x07 << 3)
#define TOGSS_RUNNING   (0x00 << 3)
#define TOGSS_SRC1      (0x01 << 3)
#define TOGSS_SRC2      (0x02 << 3)
#define TOGSS_SNK1      (0x05 << 3)
#define TOGSS_SNK2      (0x06 << 3)
#define TOGSS_AUDIOA    (0x07 << 3)
#define RXSOP2DB        (0x01 << 2)
#define RXSOP1DB        (0x01 << 1)
#define RXSOP           (0x01 << 0)

/* Interrupta : 3Eh */
#define I_OCP_TEMP      (0x01 << 7)
#define I_TOGDONE       (0x01 << 6)
#define I_SOFTFAIL      (0x01 << 5)
#define I_RETRYFAIL     (0x01 << 4)
#define I_HARDSENT      (0x01 << 3)
#define I_TXSENT        (0x01 << 2)
#define I_SOFTRST       (0x01 << 1)
#define I_HARDRST       (0x01 << 0)

/* Interruptb : 3Fh */
#define I_GCRCSENT      (0x01 << 0)

/* Status0 : 40h */
#define VBUSOK          (0x01 << 7)
#define ACTIVITY        (0x01 << 6)
#define COMP            (0x01 << 5)
#define CRC_CHK         (0x01 << 4)
#define ALERT           (0x01 << 3)
#define WAKE            (0x01 << 2)
#define BC_LVL_MASK     (0x03 << 0)
#define BC_LVL_LT200    (0x00 << 0)
#define BC_LVL_200_660  (0x01 << 0)
#define BC_LVL_660_1230 (0x02 << 0)
#define BC_LVL_GT1230   (0x03 << 0)

/* Status1 : 41h */
#define RXSOP2          (0x01 << 7)
#define RXSOP1          (0x01 << 6)
#define RX_EMPTY        (0x01 << 5)
#define RX_FULL         (0x01 << 4)
#define TX_EMPTY        (0x01 << 3)
#define TX_FULL         (0x01 << 2)
#define OVRTEMP         (0x01 << 1)
#define OCP             (0x01 << 0)

/* Interrupt : 42h */
#define I_VBUSOK        (0x01 << 7)
#define I_ACTIVITY      (0x01 << 6)
#define I_COMP_CHNG     (0x01 << 5)
#define I_CRC_CHK       (0x01 << 4)
#define I_ALERT         (0x01 << 3)
#define I_WAKE          (0x01 << 2)
#define I_COLLISION     (0x01 << 1)
#define I_BC_LVL        (0x01 << 0)

#define ADDRESS_DEVICE_ID   0x01
#define ADDRESS_SWITCHES0   0x02
#define ADDRESS_SWITCHES1   0x03
#define ADDRESS_MEASURE     0x04
#define ADDRESS_SLICE       0x05
#define ADDRESS_CONTROL0    0x06
#define ADDRESS_CONTROL1    0x07
#define ADDRESS_CONTROL2    0x08
#define ADDRESS_CONTROL3    0x09
#define ADDRESS_MASK        0x0A
#define ADDRESS_POWER       0x0B
#define ADDRESS_RESET       0x0C
#define ADDRESS_MASKA       0x0E
#define ADDRESS_MASKB       0x0F
#define ADDRESS_STATUS0A    0x3C
#define ADDRESS_STATUS1A    0x3D
#define ADDRESS_INTERRUPTA  0x3E
#define ADDRESS_INTERRUPTB  0x3F
#define ADDRESS_STATUS0     0x40
#define ADDRESS_STATUS1     0x41
#define ADDRESS_INTERRUPT   0x42
#define ADDRESS_FIFOS       0x43

#define REG_DEVICE_ID       dev->reg_control[ADDRESS_DEVICE_ID  - ADDRESS_DEVICE_ID]
#define REG_SWITCHES0       dev->reg_control[ADDRESS_SWITCHES0  - ADDRESS_DEVICE_ID]
#define REG_SWITCHES1       dev->reg_control[ADDRESS_SWITCHES1  - ADDRESS_DEVICE_ID]
#define REG_MEASURE         dev->reg_control[ADDRESS_MEASURE    - ADDRESS_DEVICE_ID]
#define REG_SLICE           dev->reg_control[ADDRESS_SLICE      - ADDRESS_DEVICE_ID]
#define REG_CONTROL0        dev->reg_control[ADDRESS_CONTROL0   - ADDRESS_DEVICE_ID]
#define REG_CONTROL1        dev->reg_control[ADDRESS_CONTROL1   - ADDRESS_DEVICE_ID]
#define REG_CONTROL2        dev->reg_control[ADDRESS_CONTROL2   - ADDRESS_DEVICE_ID]
#define REG_CONTROL3        dev->reg_control[ADDRESS_CONTROL3   - ADDRESS_DEVICE_ID]
#define REG_MASK            dev->reg_control[ADDRESS_MASK       - ADDRESS_DEVICE_ID]
#define REG_POWER           dev->reg_control[ADDRESS_POWER      - ADDRESS_DEVICE_ID]
#define REG_RESET           dev->reg_control[ADDRESS_RESET      - ADDRESS_DEVICE_ID]
#define REG_MASKA           dev->reg_control[ADDRESS_MASKA      - ADDRESS_DEVICE_ID]
#define REG_MASKB           dev->reg_control[ADDRESS_MASKB      - ADDRESS_DEVICE_ID]
#define REG_STATUS0A        dev->reg_status[ADDRESS_STATUS0A    - ADDRESS_STATUS0A]
#define REG_STATUS1A        dev->reg_status[ADDRESS_STATUS1A    - ADDRESS_STATUS0A]
#define REG_INTERRUPTA      dev->reg_status[ADDRESS_INTERRUPTA  - ADDRESS_STATUS0A]
#define REG_INTERRUPTB      dev->reg_status[ADDRESS_INTERRUPTB  - ADDRESS_STATUS0A]
#define REG_STATUS0         dev->reg_status[ADDRESS_STATUS0     - ADDRESS_STATUS0A]
#define REG_STATUS1         dev->reg_status[ADDRESS_STATUS1     - ADDRESS_STATUS0A]
#define REG_INTERRUPT       dev->reg_status[ADDRESS_INTERRUPT   - ADDRESS_STATUS0A]

enum FUSB302_transmit_data_tokens_t {
    TX_TOKEN_TXON    = 0xA1,
    TX_TOKEN_SOP1    = 0x12,
    TX_TOKEN_SOP2    = 0x13,
    TX_TOKEN_SOP3    = 0x1B,
    TX_TOKEN_RESET1  = 0x15,
    TX_TOKEN_RESET2  = 0x16,
    TX_TOKEN_PACKSYM = 0x80,
    TX_TOKEN_JAM_CRC = 0xFF,
    TX_TOKEN_EOP     = 0x14,
    TX_TOKEN_TXOFF   = 0xFE,
};

enum FUSB302_state_t {
    FUSB302_STATE_UNATTACHED = 0,
    FUSB302_STATE_ATTACHED
};

#define FUSB302_ERR_MSG(s)  s

#define REG_READ(addr, data, count) do { \
    if (reg_read(dev, addr, data, count) != FUSB302_SUCCESS) { return FUSB302_ERR_READ_DEVICE; } \
} while(0)

#define REG_WRITE(addr, data, count) do { \
    if (reg_write(dev, addr, data, count) != FUSB302_SUCCESS) { return FUSB302_ERR_WRITE_DEVICE; } \
} while(0)

static inline FUSB302_ret_t reg_read(FUSB302_dev_t *dev, uint8_t address, uint8_t *data, uint8_t count)
{
    FUSB302_ret_t ret = dev->i2c_read(dev->i2c_address, address, data, count); // 调用板级注入的 I2C 读函数。
    if (ret != FUSB302_SUCCESS) {
        dev->err_msg = FUSB302_ERR_MSG("Fail to read register");
    }
    return ret;
}

static inline FUSB302_ret_t reg_write(FUSB302_dev_t *dev, uint8_t address, uint8_t *data, uint8_t count)
{
    FUSB302_ret_t ret = dev->i2c_write(dev->i2c_address, address, data, count); // 调用板级注入的 I2C 写函数。
    if (ret != FUSB302_SUCCESS) {
        dev->err_msg = FUSB302_ERR_MSG("Fail to write register");
    }
    return ret;
}

/** @brief 连续读取 6 次 BC_LVL，只有结果稳定一致才接受本次 CC 电平。 */
static FUSB302_ret_t FUSB302_read_cc_lvl(FUSB302_dev_t *dev, uint8_t * cc_value)
{
    /*  00: < 200 mV          : vRa
        01: >200 mV, <660 mV  : vRd-USB
        10: >660 mV, <1.23 V  : vRd-1.5
        11: >1.23 V           : vRd-3.0  */
    uint8_t cc, cc_verify;
    REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
    cc = REG_STATUS0 & BC_LVL_MASK;
    for (uint8_t i = 0; i < 5; i++) {
        REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
        cc_verify = REG_STATUS0 & BC_LVL_MASK;
        if (cc != cc_verify) {
            return FUSB302_BUSY;
        }
    }
    *cc_value = cc;
	return FUSB302_SUCCESS;
}

/** @brief 从 RX FIFO 取 SOP token、消息头、Data Object 和 CRC，并产生收包事件。 */
static FUSB302_ret_t FUSB302_read_incoming_packet(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    uint8_t len, b[3];                                      // b[0] 是 SOP token，b[1..2] 是消息头。
    REG_READ(ADDRESS_FIFOS, b, 3);                          // 先读 token 和 16 位小端消息头。
    dev->rx_header = ((uint16_t)b[2] << 8) | b[1];          // 合成主机端 16 位消息头。
    len = (dev->rx_header >> 12) & 0x7;                     // 头部 bit14..12 给出 32 位对象数量。
    REG_READ(ADDRESS_FIFOS, dev->rx_buffer, len * 4 + 4);   // 每个对象 4 字节，最后再读出 4 字节 CRC。

    if (events) {
        *events |= FUSB302_EVENT_RX_SOP;
    }
    return FUSB302_SUCCESS;
}

/**
 * @brief 未连接状态下检测 VBUS，并依次测量 CC1/CC2 后选择 PD 收发引脚。
 */
static FUSB302_ret_t FUSB302_state_unattached(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
    if (REG_STATUS0 & VBUSOK) { // 官方寄存器 VBUSOK 置位后，才开始 CC 检测和接收机配置。
        /* enable internal oscillator */
        REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE | PWR_INT_OSC;
        REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);
        dev->delay_ms(2);

        /* read cc1 */
        REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC1; // CC1/CC2 保持 Rd，下次测量选择 CC1。
        REG_SWITCHES1 = SPECREV0;
        REG_MEASURE = 49;
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);
        dev->delay_ms(2);
        while (FUSB302_read_cc_lvl(dev, &dev->cc1) != FUSB302_SUCCESS) { // 电平连续读取不稳定就稍后重试。
            dev->delay_ms(2);
        }

        /* read cc2 */
        REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC2; // 改为测量 CC2。
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 1);
        dev->delay_ms(2);
        while (FUSB302_read_cc_lvl(dev, &dev->cc2) != FUSB302_SUCCESS) {
            dev->delay_ms(2);
        }

        /* clear interrupt */
        REG_READ(ADDRESS_INTERRUPTA, &REG_INTERRUPTA, 2);
        dev->interrupta = 0;
        dev->interruptb = 0;

        /* enable tx on cc pin */
        if (dev->cc1 > 0) { // 插头方向落在 CC1：测量和 BMC 发送都选 CC1。
            REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC1;
            REG_SWITCHES1 = SPECREV0 | AUTO_CRC | TXCC1;
            //REG_SWITCHES1 = SPECREV0 | TXCC1;
        } else if (dev->cc2 > 0) { // 插头方向落在 CC2：测量和 BMC 发送都选 CC2。
            REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC2;
            REG_SWITCHES1 = SPECREV0 | AUTO_CRC | TXCC2;
            //REG_SWITCHES1 = SPECREV0 | TXCC2;
        } else {
            REG_SWITCHES0 = PDWN1 | PDWN2;
            REG_SWITCHES1 = SPECREV0;
        }
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 2);

        /* update state */
        dev->state = FUSB302_STATE_ATTACHED; // 后续 FUSB302_alert() 改走已连接处理函数。
        if (events) {
            *events |= FUSB302_EVENT_ATTACHED;
        }
    }
    return FUSB302_SUCCESS;
}

/**
 * @brief 已连接状态下汇总中断、检测掉线、处理 Hard Reset 和读取 RX FIFO。
 */
static FUSB302_ret_t FUSB302_state_attached(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    REG_READ(ADDRESS_STATUS0A, &REG_STATUS0A, 7); // 连续读状态和中断寄存器；中断寄存器读取后清除。
    dev->interrupta |= REG_INTERRUPTA;           // 先累计到软件位图，避免一次只消费一个事件。
    dev->interruptb |= REG_INTERRUPTB;
    if (dev->vbus_sense && ((REG_STATUS0 & VBUSOK) == 0)) { // 固定 PDO 模式下 VBUS 消失视为断开。
        /* reset cc pins to pull down */
        REG_SWITCHES0 = PDWN1 | PDWN2;
        REG_SWITCHES1 = SPECREV0;
        REG_MEASURE = 49;
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);

        /* turn off internal oscillator */
        REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE;
        REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);

        /* update state */
        dev->state = FUSB302_STATE_UNATTACHED; // 回到未连接状态，等待下一次 VBUS/CC 检测。
        if (events) {
            *events |= FUSB302_EVENT_DETACHED;
        }
        return FUSB302_SUCCESS;
    }
    if (REG_STATUS0A & HARDRST) {
        uint8_t reg_control = PD_RESET;
        REG_WRITE(ADDRESS_RESET, &reg_control, 1);
        return FUSB302_SUCCESS;
    }
    if (dev->interruptb & I_GCRCSENT) { // FUSB302 已自动发出 GoodCRC，协议层现在可以处理/回复收到的消息。
        dev->interruptb &= ~I_GCRCSENT;
        if (events) {
            *events |= FUSB302_EVENT_GOOD_CRC_SENT;
        }
    }
    if ((REG_STATUS1 & RX_EMPTY) == 0) { // RX FIFO 非空时读取一条消息。
        if (FUSB302_read_incoming_packet(dev, events) != FUSB302_SUCCESS) {
            uint8_t rx_flush = REG_CONTROL1 | RX_FLUSH;
            reg_write(dev, ADDRESS_CONTROL1, &rx_flush, 1);
        }
    }
    return FUSB302_SUCCESS;
}

/**
 * @brief 校验板级接口，复位并配置 FUSB302B 的 CC、自动重试、中断和电源模块。
 * @return FUSB302_SUCCESS 成功；其他值表示参数、I2C 或器件 ID 错误。
 */
FUSB302_ret_t FUSB302_init(FUSB302_dev_t *dev)
{
    if (dev->i2c_address == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid i2c address");
        return FUSB302_ERR_PARAM;
    }
    if (dev->i2c_read == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid i2c_read function");
        return FUSB302_ERR_PARAM;
    }
    if (dev->i2c_write == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid i2c_write function");
        return FUSB302_ERR_PARAM;
    }

    if (reg_read(dev, ADDRESS_DEVICE_ID, &dev->reg_control[1], 1) != FUSB302_SUCCESS) {
        dev->err_msg = FUSB302_ERR_MSG("Device not found");
        return FUSB302_ERR_READ_DEVICE;
    }

	if ((dev->reg_control[1] & 0x80) == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid device version");
        return FUSB302_ERR_DEVICE_ID;
    }

    dev->state = FUSB302_STATE_UNATTACHED; // 软件状态从未连接开始。
    dev->rx_header = 0;
    memset(dev->rx_buffer, 0, sizeof(dev->rx_buffer));

    /* restore default settings */
    REG_RESET = SW_RES; // 软件复位 FUSB302B 寄存器状态。
    REG_WRITE(ADDRESS_RESET, &REG_RESET, 1);

    /* fetch all R/W registers */
    REG_READ(ADDRESS_DEVICE_ID, &REG_DEVICE_ID, 15);

    /* configure switchs and comparators */
    REG_SWITCHES0 = PDWN1 | PDWN2; // 作为 Sink，在 CC1/CC2 打开下拉 Rd。
    REG_SWITCHES1 = SPECREV0;
    REG_MEASURE = 49;
	REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);

    /* configure auto retries */
    REG_CONTROL3 &= ~N_RETRIES_MASK;
    REG_CONTROL3 |= N_RETRIES(3) | AUTO_RETRY; // 让芯片自动重试发送，次数字段设为 3。
    REG_WRITE(ADDRESS_CONTROL3, &REG_CONTROL3, 1);

    /* configure interrupt mask */
    REG_MASK = 0xFF;
    REG_MASK &= ~(M_VBUSOK | M_ACTIVITY | M_COLLISION | M_ALERT | M_CRC_CHK);
    REG_WRITE(ADDRESS_MASK, &REG_MASK, 1);

    /* configure interrupt maska/maskb */
    REG_MASKA = 0xFF;
    REG_MASKA &= ~(M_RETRYFAIL | M_HARDSENT | M_TXSENT | M_HARDRST);
    REG_WRITE(ADDRESS_MASKA, &REG_MASKA, 1);
    REG_MASKB = 0xFF;
    REG_MASKB &= ~(M_GCRCSENT);
    REG_WRITE(ADDRESS_MASKB, &REG_MASKB, 1);

    /* enable interrupt */
    REG_CONTROL0 &= ~INT_MASK; // INT_MASK 清零，允许 INT_N 输出未屏蔽事件。
    REG_WRITE(ADDRESS_CONTROL0, &REG_CONTROL0, 1);

    /* Power on, enable VUSB detection */
    REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE;
    REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);

    dev->vbus_sense = 1;
    dev->err_msg = FUSB302_ERR_MSG("");
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_pd_reset(FUSB302_dev_t *dev)
{
    uint8_t reg = PD_RESET;
    REG_WRITE(ADDRESS_RESET, &reg, 1);
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_pdwn_cc(FUSB302_dev_t *dev, uint8_t enable)
{
    REG_SWITCHES0 = enable ? (PDWN1 | PDWN2) : 0;
	REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 1);
    return FUSB302_SUCCESS;
}

/** @brief 打开或关闭 VBUSOK 中断参与掉线判断；PPS 低电压时上层会关闭它。 */
FUSB302_ret_t FUSB302_set_vbus_sense(FUSB302_dev_t *dev, uint8_t enable)
{
    if (dev->vbus_sense != enable) {
        if (enable) {
            REG_MASK &= ~M_VBUSOK;  /* enable VBUSOK interrupt */
        } else {
            REG_MASK |= M_VBUSOK;   /* disable VBUSOK interrupt */
        }
        REG_WRITE(ADDRESS_MASK, &REG_MASK, 1);
        dev->vbus_sense = enable;
    }
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_get_ID(FUSB302_dev_t *dev, uint8_t * version_ID, uint8_t * revision_ID)
{
    if (dev && (REG_DEVICE_ID & 0x80)) {
        if (version_ID) {
            *version_ID = (REG_DEVICE_ID >> 4) & 0x7;
        }
        if (revision_ID) {
            *revision_ID = (REG_DEVICE_ID >> 0) & 0xF;
        }
        return FUSB302_SUCCESS;
    }
    return FUSB302_ERR_PARAM;
}

FUSB302_ret_t FUSB302_get_cc(FUSB302_dev_t *dev, uint8_t *cc1, uint8_t *cc2)
{
    if (cc1) {
        *cc1 = dev->cc1;
    }
    if (cc2) {
        *cc2 = dev->cc2;
    }
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_get_vbus_level(FUSB302_dev_t *dev, uint8_t *vbus)
{
    uint8_t reg_control;
    REG_READ(ADDRESS_STATUS0, &reg_control, 1);
    *vbus = reg_control & VBUSOK ? 1 : 0;
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_get_message(FUSB302_dev_t *dev, uint16_t * header, uint32_t * data)
{
    if (header) {
        *header = dev->rx_header;
    }
    if (data) {
        uint8_t len = (dev->rx_header >> 12) & 0x7;
        memcpy(data, dev->rx_buffer, len * 4);
    }
	return FUSB302_SUCCESS;
}

/**
 * @brief 把一条 SOP 消息编码为 FUSB302 TX FIFO token 序列并启动发送。
 * @param dev 设备上下文。
 * @param header 16 位 PD 消息头。
 * @param data 0~7 个 32 位 Data Object；对象数由 header 给出。
 * @return FUSB302_SUCCESS 或寄存器写错误。
 */
FUSB302_ret_t FUSB302_tx_sop(FUSB302_dev_t *dev, uint16_t header, const uint32_t *data)
{
    uint8_t buf[40];                              // 最大 7 个对象时仍能容纳全部 token 和数据。
    uint8_t * pbuf = buf;                         // 始终指向下一个待写位置。
    uint8_t obj_count = ((header >> 12) & 7);     // 从消息头取 Data Object 数量。
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;             // 依次写入 SOP 起始 token。
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;
    *pbuf++ = (uint8_t)TX_TOKEN_SOP2;
    *pbuf++ = (uint8_t)TX_TOKEN_PACKSYM | ((obj_count << 2) + 2); // PACKSYM 长度=头2字节+对象字节数。
    *pbuf++ = header & 0xFF; header >>= 8;         // PD 消息头低字节先入 FIFO。
    *pbuf++ = header & 0xFF;                       // 再写消息头高字节。
    for (uint8_t i = 0; i < obj_count; i++) {     // 逐个写 32 位 Data Object。
        uint32_t d = *data++;                     // 取当前对象，同时移动到下一个对象。
        *pbuf++ = d & 0xFF; d >>= 8;              // 小端顺序写第 0 字节。
        *pbuf++ = d & 0xFF; d >>= 8;              // 第 1 字节。
        *pbuf++ = d & 0xFF; d >>= 8;              // 第 2 字节。
        *pbuf++ = d & 0xFF;                       // 第 3 字节。
    }
    *pbuf++ = (uint8_t)TX_TOKEN_JAM_CRC;           // 让芯片追加 CRC。
    *pbuf++ = (uint8_t)TX_TOKEN_EOP;               // 写入包结束 token。
    *pbuf++ = (uint8_t)TX_TOKEN_TXOFF;             // 按 FUSB302 token 流结束发送。
    *pbuf++ = (uint8_t)TX_TOKEN_TXON;              // 启动本次 token 序列发送。
    REG_WRITE(ADDRESS_FIFOS, buf, pbuf - buf);      // 一次性把完整 token 序列写入 TX FIFO。
    dev->delay_ms(2);                              // 给硬件发送留出短暂时间。
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_tx_hard_reset(FUSB302_dev_t *dev)
{
    uint8_t reg_control = REG_CONTROL3;
    reg_control |= SEND_HARDRESET;
    REG_WRITE(ADDRESS_CONTROL3, &reg_control, 1);
    dev->delay_ms(6);
    reg_control = PD_RESET;
    REG_WRITE(ADDRESS_RESET, &reg_control, 1);
    return FUSB302_SUCCESS;
}

/** @brief 按当前连接状态分派事件处理函数，是上层轮询 FUSB302 的统一入口。 */
FUSB302_ret_t FUSB302_alert(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    FUSB302_ret_t (* const handler[]) (FUSB302_dev_t *, FUSB302_event_t *) = {
        FUSB302_state_unattached,
        FUSB302_state_attached
    };
    if (dev->state < sizeof(handler) / sizeof(handler[0])) {
        return handler[dev->state](dev, events);
    }
    dev->state = FUSB302_STATE_UNATTACHED;
    return FUSB302_SUCCESS;
}
